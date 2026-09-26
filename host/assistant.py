"""Local, adaptive screen-target capture assistant. No nominal metric scale."""
import argparse
import fcntl
import io
import json
import queue
import socket
import struct
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor
import time
from pathlib import Path
import tkinter as tk
from PIL import Image, ImageTk, ImageOps
import cv2
import numpy as np
from host.calibrate import board_contained

PROJECT = Path(__file__).resolve().parents[1]
SHAPE = (9, 6)


# Compatibility exports for existing capture scripts and tests.
from host.transport import receive_exact, read_pair


def detect(gray, previous=None):
    # Try the last board region first; full-frame detection stays off the preview thread.
    if previous is not None:
        low=previous.min(axis=0).ravel();high=previous.max(axis=0).ravel()
        pad=max(high-low)*.35
        x0,y0=np.maximum(0,low-pad).astype(int)
        x1,y1=np.minimum(gray.shape[::-1],high+pad).astype(int)
        roi=gray[y0:y1,x0:x1]
        if roi.size:
            ok,p=cv2.findChessboardCorners(roi,SHAPE,cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
            if ok:
                p+=np.array([x0,y0],np.float32)
                return cv2.cornerSubPix(gray,p,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,30,.01))
    # Fast live locator; final accuracy is independently checked offline.
    ok, points = cv2.findChessboardCorners(gray, SHAPE,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
    if not ok:
        return None
    return cv2.cornerSubPix(gray, points, (5, 5), (-1, -1),
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .01))


def align(points, reference):
    if reference is None:
        return points
    return points[::-1].copy() if np.linalg.norm(points[::-1]-reference) < np.linalg.norm(points-reference) else points


def shape_features(points):
    grid = points.reshape(6, 9, 2)
    tl, tr, bl, br = grid[0, 0], grid[0, -1], grid[-1, 0], grid[-1, -1]
    vertical = np.log(np.linalg.norm(tr-br) / np.linalg.norm(tl-bl))
    horizontal = np.log(np.linalg.norm(bl-br) / np.linalg.norm(tl-tr))
    angle = np.arctan2(*(tr-tl)[::-1])
    return np.array([vertical, horizontal, angle])


class BurstSelector:
    """Rank a short burst; small hand tremor never restarts a hold countdown.

    Motion risk is a sampling-based heuristic, NOT an epipolar accuracy claim.
    Before/after samples are required; final calibration still uses held-out views.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.samples = []
        self.start = None

    def update(self, packet, points, sharpness):
        h = packet[0]
        t = h['20']['image']['imageTimestampNs']/1e9
        if self.samples and t <= self.samples[-1]['time']:
            return 0., None
        if self.start is None:
            self.start = t
        self.samples = [s for s in self.samples if t-s['time'] <= 2.5][-19:]
        self.samples.append(dict(time=t,packet=packet,points=[p.copy() for p in points],sharpness=sharpness))
        progress = min(1., (t-self.start)/1.5)
        if progress < 1:
            return progress,None
        ranked = []
        for before, sample, after in zip(self.samples,self.samples[1:],self.samples[2:]):
            header = sample['packet'][0]
            if not header.get('paired',True) or abs(header['20']['image']['imageTimestampNs']-header['21']['image']['imageTimestampNs']) > 20_000_000:
                continue
            if min(sample['sharpness']) < 30:
                continue
            risks=[]
            for index,camera in enumerate(('20','21')):
                motion=[]
                for neighbor in (before,after):
                    dt=abs(neighbor['packet'][0][camera]['image']['imageTimestampNs']-header[camera]['image']['imageTimestampNs'])/1e9
                    if not .04 <= dt <= .6:
                        motion.append(float('inf'))
                    else:
                        distance=np.percentile(np.linalg.norm(neighbor['points'][index]-sample['points'][index],axis=2),90)
                        motion.append(float(distance)/dt)
                capture=header[camera]['capture']
                # Include sensor offset, exposure and rolling scan in the risk estimate.
                window=(header['deltaNs']+(capture.get('exposureTimeNs') or 20_000_000)+(capture.get('rollingShutterSkewNs') or 33_000_000))/1e9
                risks.append(max(motion)*window)
            risk=max(risks)
            if risk <= 3.0:
                ranked.append((risk+1/np.sqrt(min(sample['sharpness'])),sample,risks))
        if not ranked:
            return progress,None
        _,best,risks=min(ranked,key=lambda x:x[0])
        best=dict(best,quality={'method':'burst-neighbor-motion-and-sharpness-v1',
            'estimatedMotionRiskPx':risks,'boardLaplacianVariance':best['sharpness'],
            'candidateCount':len(ranked),'sampleCount':len(self.samples),
            'note':'Heuristic selection; independent calibration accuracy not yet verified'})
        return progress,best


def board_sharpness(gray, points):
    x0,y0=np.maximum(0,np.floor(points.min(axis=0).ravel())).astype(int)
    x1,y1=np.minimum(gray.shape[::-1],np.ceil(points.max(axis=0).ravel())).astype(int)
    roi=gray[y0:y1,x0:x1]
    return float(cv2.Laplacian(roi,cv2.CV_32F).var()) if roi.size else 0.


def save_pair(directory, packet, points, pose, target):
    directory.mkdir(parents=True, exist_ok=True)
    header, blobs = packet
    for c, blob, corners in zip(('20', '21'), blobs, points):
        row = dict(header[c]['image'])
        row['sampleFile'] = f"camera_{c}_{row['sequence']}_{row['imageTimestampNs']}.jpg"
        (directory/row['sampleFile']).write_bytes(blob)
        for name, value in [('images', row), ('metadata', header[c]['capture'])]:
            with (directory/f'camera_{c}_{name}.jsonl').open('a') as out:
                out.write(json.dumps(value)+'\n')
    with (directory/'selected-poses.jsonl').open('a') as out:
        out.write(json.dumps({'pose': pose, 'deltaNs': header['deltaNs'], 'target': target,
            'leftTimestampNs': header['20']['image']['imageTimestampNs'],
            'rightTimestampNs': header['21']['image']['imageTimestampNs'],
            'corners': [p.tolist() for p in points], 'shapeFeatures': [shape_features(p).tolist() for p in points],
            'sourceRun': header.get('sourceRun'), 'metricScaleVerified': False})+'\n')


class Assistant:
    def __init__(self, root, demo=False, session_directory=None, resume=False):
        self.root, self.demo = root, demo
        self.closed = threading.Event()
        self.messages = queue.Queue(maxsize=8)
        self.preview_queue = queue.Queue(maxsize=1)
        self.last_preview_at = 0.
        self.preview_count = 0
        self.restart_requested = threading.Event()
        self.started = self.armed = self.invalidated = False
        self.index = 0
        self.gate = BurstSelector()
        self.previous = [None, None]
        self.reference = None
        self.saved_points = []
        self.last_ts = 0
        self.cooldown = 0.
        self.session = Path(session_directory) if session_directory else PROJECT/'data'/'guided'/time.strftime('%Y%m%d_%H%M%S')
        self.session.mkdir(parents=True, exist_ok=True)
        self.manifest = {'target':'screen-checkerboard','innerCorners':[9,6],
            'measuredSquareMm':None,'metricScaleVerified':False,'session':str(self.session),
            'demo':demo,'poseCount':0,'state':'preparing','screenPixels':[root.winfo_screenwidth(),root.winfo_screenheight()]}
        if resume:
            old=json.loads((self.session/'session.json').read_text())
            if old.get('state')=='invalidated-by-target-resize':
                raise ValueError('Target size changed; cannot resume this session')
            rows=[]
            for part in ('training','validation'):
                file=self.session/part/'selected-poses.jsonl'
                if file.exists():rows.extend(json.loads(line) for line in file.read_text().splitlines() if line.strip())
            if [r['pose'] for r in rows] != list(range(len(rows))):
                raise ValueError('Pose records are not contiguous; inspect before resuming')
            self.index=len(rows)
            self.saved_points=[np.array(r['corners'][0],np.float32) for r in rows]
            if self.saved_points:self.reference=shape_features(self.saved_points[0])
            self.manifest.update(old,poseCount=self.index,state='paused',selectionMode='adaptive-burst')
        self.write_manifest()
        root.title('Stereo Rehber · USB kamera kalibrasyonu')
        root.configure(bg='#101827')
        root.attributes('-fullscreen', True)
        root.bind('<Escape>', lambda _: root.attributes('-fullscreen', False))
        root.bind('<F11>', lambda _: root.attributes('-fullscreen', not root.attributes('-fullscreen')))
        root.bind('<space>', lambda _: self.toggle())
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.canvas = tk.Canvas(root, bg='#f2f2f2', highlightthickness=0)
        self.canvas.place(relx=0, rely=0, relwidth=.69, relheight=1)
        self.canvas.bind('<Configure>', self.draw_board)
        panel = tk.Frame(root, bg='#101827', padx=26, pady=20)
        panel.place(relx=.69, rely=0, relwidth=.31, relheight=1)
        def label(text, size, color='#f8fafc'):
            widget = tk.Label(panel, text=text, font=('DejaVu Sans', size), fg=color,
                bg='#101827', wraplength=int(root.winfo_screenwidth()*.27), justify='left', anchor='w')
            widget.pack(fill='x', pady=7)
            return widget
        label('TELEFONUN GÖRDÜĞÜ GÖRÜNTÜ →', 14, '#8da4bc')
        self.step = label('Önce telefonu yerleştirelim', 24)
        self.instruction = label('Arka kameraları damaya çevirin. Telefonu yaklaşık bir kol mesafesinde tutun; tüm dama iki küçük görüntüde de görünsün.', 18)
        self.diagram = tk.Canvas(panel, bg='#101827', height=105, highlightthickness=0)
        self.diagram.pack(fill='x')
        self.status = label('Bağlantı hazırlanıyor…', 16, '#fbbf24')
        self.bar = tk.Canvas(panel, height=12, bg='#253449', highlightthickness=0)
        self.bar.pack(fill='x', pady=8)
        self.previews = []
        for name in ('BÜYÜK GÖRÜNTÜ · Telefonun üst kamerası', 'GENİŞ AÇI · Telefonun alt kamerası'):
            label(name, 11, '#8da4bc')
            view = tk.Label(panel, bg='#050a12')
            view.pack(fill='x')
            self.previews.append(view)
        self.button = tk.Button(panel, text='Başlat', command=self.toggle, font=('DejaVu Sans', 17),
            bg='#32d5a4', fg='#08221b', relief='flat', pady=10)
        self.button.pack(fill='x', pady=10)
        label('Dama sabit. Telefonu siz hafifçe eğin.\nKüçük titremeler normal. İyi kareyi kendisi seçer.\nBoşluk: duraklat · Esc: pencere · F11: tam ekran', 11, '#a4b4c9')
        tk.Button(panel, text='Kapat', command=self.close, bg='#253449', fg='white', relief='flat').pack(fill='x')
        if 0<self.index<40:
            self.update_instruction()
            self.button.config(text='Kaldığım yerden devam et')
        self.draw_phone(self.pose()[1] if self.index<40 else 'flat')
        threading.Thread(target=self.worker, daemon=True).start()
        root.after(100, self.poll)

    def write_manifest(self):
        (self.session/'session.json').write_text(json.dumps(self.manifest, indent=2, ensure_ascii=False))

    def draw_board(self, event=None):
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        if w < 100:
            return
        # Geometry stays constant throughout this session. Equal integer-sized square edges.
        side = int(min(w*.86/10, h*.80/7))
        x, y = (w-10*side)//2, (h-7*side)//2
        self.canvas.delete('all')
        self.canvas.create_rectangle(x-side*.4, y-side*.4, x+10.4*side, y+7.4*side, fill='white', outline='')
        for row in range(7):
            for col in range(10):
                self.canvas.create_rectangle(x+col*side, y+row*side, x+(col+1)*side, y+(row+1)*side,
                    fill='black' if (row+col)%2 == 0 else 'white', outline='')
        self.canvas.create_text(w/2, h-42, text='9 × 6 iç köşe · Ekrandaki dama gerçek bir düzlem olarak sabit kalır',
            fill='#64748b', font=('DejaVu Sans', 13))
        old = self.manifest.get('squarePixels')
        if old is not None and old != side and self.index:
            self.armed = False
            self.invalidated = True
            self.status.config(text='Pencere boyutu değişti. Bu oturum bitti; yeniden açın.', fg='#ff8d8d')
            self.button.config(state='disabled')
            self.manifest['state'] = 'invalidated-by-target-resize'
        self.manifest['squarePixels'] = side
        self.write_manifest()

    def pose(self):
        # 30 training views, then 10 separately labelled validation views.
        centers = [(.5,.5),(.4,.4),(.6,.6),(.4,.6),(.6,.4)]
        modes = ['flat','side','up','roll','side','up']
        if self.index < 30:
            return centers[self.index%5], modes[self.index//5]
        centers = [(.44,.44),(.56,.56),(.44,.56),(.56,.44),(.5,.5)]
        return centers[(self.index-30)%5], ['side','up'][(self.index-30)//5]

    def draw_phone(self, mode):
        c = self.diagram
        c.delete('all')
        if mode == 'side':
            poly = [95,25,160,10,160,125,95,110]
            arrow = '↶  TELEFONU HAFİFÇE SAĞA / SOLA EĞİN'
        elif mode == 'up':
            poly = [90,35,160,35,173,110,77,110]
            arrow = '↕  ÜST KENARI HAFİFÇE ÖNE / ARKAYA EĞİN'
        elif mode == 'roll':
            poly = [105,10,175,30,145,125,75,105]
            arrow = '↻  TELEFONU AZICIK SAAT YÖNÜNDE ÇEVİRİN'
        else:
            poly = [90,15,160,15,160,115,90,115]
            arrow = 'EKRANA BAKAN TELEFON · RAHATÇA TUTUN'
        c.create_polygon(poly, fill='#254a61', outline='#64e5c0', width=3)
        c.create_oval(poly[0]+8,poly[1]+10,poly[0]+18,poly[1]+20,fill='#64e5c0',outline='')
        c.scale('all',0,0,.65,.65)
        c.create_text(140,40,text=arrow.replace(' · ','\n').replace(' / ',' /\n'),width=350,anchor='w',fill='#64e5c0',font=('DejaVu Sans',11))

    def toggle(self):
        if self.index >= 40 or self.invalidated:
            return
        self.armed = not self.armed
        self.gate.reset()
        self.button.config(text='Duraklat' if self.armed else 'Devam et')
        self.manifest['state'] = 'capturing' if self.armed else 'paused'
        self.write_manifest()
        if self.armed:
            self.update_instruction()

    def update_instruction(self):
        center, mode = self.pose()
        phase = 'Eğitim' if self.index < 30 else 'Bağımsız doğrulama'
        self.step.config(text=f'{phase} · {self.index+1} / 40')
        self.instruction.config(text='İlk küçük görüntüde damanın merkezini yeşil kutuya getirin. '+{
            'flat':'Telefonu rahatça tutun; küçük titremeleri dert etmeyin.',
            'side':'Telefonun sağ veya sol kenarını biraz ekrana yaklaştırın.',
            'up':'Telefonun üst veya alt kenarını biraz ekrana yaklaştırın.',
            'roll':'Telefonu azıcık saat yönünde çevirin.'}[mode])
        self.draw_phone(mode)

    def post(self, item):
        try:
            self.messages.put_nowait(item)
        except queue.Full:
            pass

    def worker(self):
        try:
            if self.demo:
                self.post(('error','Arayüz önizlemesi · telefon kaydı yapılmıyor.'))
                return
            def adb(*args):
                return subprocess.run(['adb',*args],check=True,capture_output=True,text=True,timeout=20)
            adb('get-state')
            def start_capture():
                adb('shell','am','force-stop','org.research.phonestereo')
                adb('forward','tcp:8765','tcp:8765')
                adb('shell','am','start','-n','org.research.phonestereo/.MainActivity',
                    '--es','ids','20,21','--ei','seconds','3600','--ei','width','1280','--ei','height','960',
                    '--ez','fixed','true','--ez','live','true','--ef','focus','1.4','--ei','iso','400')
                return time.monotonic()
            capture_start = start_capture()
            generation = int(self.index>=30)
            executor = ThreadPoolExecutor(max_workers=1,thread_name_prefix='board-detection')
            pending = None
            tracked = [None,None]
            self.started = True
            while not self.closed.is_set():
                begin = time.monotonic()
                if pending is not None and pending.done():
                    packet_done, gray_done, detected, begin_done = pending.result()
                    tracked = detected
                    self.post(('pair',packet_done,gray_done,detected,begin_done))
                    pending = None
                if self.restart_requested.is_set():
                    self.restart_requested.clear()
                    self.post(('error','Doğrulama için kameralar yeniden açılıyor…'))
                    capture_start = start_capture()
                    generation += 1
                try:
                    packet = read_pair()
                    if packet:
                        header, blobs = packet
                        header['hostCaptureGeneration'] = generation
                        if time.monotonic()-capture_start < 4:
                            self.post(('error','Kamera odağının yerleşmesi bekleniyor…'))
                            self.closed.wait(.2)
                            continue
                        gray = [cv2.imdecode(np.frombuffer(b,np.uint8),0) for b in blobs]
                        if any(g is None for g in gray):
                            raise ValueError('JPEG decode failed')
                        try:
                            self.preview_queue.get_nowait()
                        except queue.Empty:
                            pass
                        self.preview_queue.put_nowait((packet,gray,begin))
                        if pending is None:
                            def inspect(p,images,prior,t):
                                return p,images,[detect(g,old) for g,old in zip(images,prior)],t
                            pending=executor.submit(inspect,packet,gray,tracked,begin)
                    else:
                        self.post(('error','Kameraların yeni bir görüntü çifti bekleniyor…'))
                except (OSError,ValueError) as error:
                    self.post(('error',f'USB görüntüsü bekleniyor… {error}'))
                self.closed.wait(max(.015,.10-(time.monotonic()-begin)))
            executor.shutdown(wait=False,cancel_futures=True)
        except Exception as error:
            self.post(('error',f'Bağlantı kurulamadı: {error}'))

    def render_preview(self,packet,gray,received):
        if time.monotonic()-received > .8:
            return
        self.last_preview_at=time.monotonic()
        self.preview_count+=1
        center,mode=self.pose() if self.index < 40 else ((.5,.5),'flat')
        width = int(self.root.winfo_screenwidth()*.285)
        for i in range(2):
            display = cv2.cvtColor(gray[i],cv2.COLOR_GRAY2RGB)

            if i == 0:
                x,y = np.array(center)*np.array(gray[i].shape[::-1])
                tol = .065 if self.index < 30 else .035
                dx,dy = np.array(gray[i].shape[::-1])*tol
                cv2.rectangle(display,(int(x-dx),int(y-dy)),(int(x+dx),int(y+dy)),(60,220,140),3)

            display = cv2.rotate(display, cv2.ROTATE_90_CLOCKWISE)
            photo = ImageTk.PhotoImage(ImageOps.contain(Image.fromarray(display),(width,int(self.root.winfo_screenheight()*(.31 if i==0 else .16)))))
            self.previews[i].config(image=photo)
            self.previews[i].image = photo
    def process(self, packet, gray, points, received):
        if packet[0].get('hostCaptureGeneration') != int(self.index >= 30) or self.invalidated:
            return
        ts = packet[0]['20']['image']['imageTimestampNs']
        if ts <= self.last_ts or time.monotonic()-received > .8:
            return
        self.last_ts = ts
        for i in range(2):
            if points[i] is not None:
                points[i] = align(points[i],self.previous[i])
                self.previous[i] = points[i]
        center, mode = self.pose() if self.index < 40 else ((.5,.5),'flat')
        progress = 0
        message, color = 'Hazır olduğunuzda Başlat’a basın.', '#a4b4c9'
        good = all(p is not None for p in points)
        if self.armed and self.index < 40:
            if not packet[0].get('paired',True):
                message = 'Canlı görüntü akıyor. Otomatik kayıt için burada biraz bekleyin.'
                good = False
            elif not all(40_000_000 <= packet[0][c]['image'].get('precedingIntervalNs',0) <= 100_000_000 for c in ('20','21')):
                message = 'Görüntü akışının düzenli hale gelmesi bekleniyor…'
                good = False
            elif not good:
                message = 'Damanın tamamını iki görüntüye de getirin. Biraz uzaklaşmayı deneyin.'
            elif not all(board_contained(p,SHAPE,g.shape[::-1],18) for p,g in zip(points,gray)):
                message = 'Dama kenara çok yakın. Biraz uzaklaşın; beyaz dış kenar da görünsün.'
                good = False
            elif np.max(np.abs(points[0].mean(axis=0).ravel()/np.array(gray[0].shape[::-1])-center)) > (.065 if self.index < 30 else .035):
                message = 'İlk küçük görüntüde damayı yeşil kutuya doğru kaydırın.'
                good = False
            else:
                features = shape_features(points[0])
                if self.reference is None:
                    self.reference = features.copy()
                difference = np.abs(features-self.reference)
                difference[2] = abs(np.arctan2(np.sin(features[2]-self.reference[2]),np.cos(features[2]-self.reference[2])))
                tilt_ok = mode == 'flat' or (mode == 'side' and difference[0]>.065) or (mode == 'up' and difference[1]>.065) or (mode == 'roll' and difference[2]>.12)
                if not tilt_ok:
                    message = 'Çizimdeki gibi biraz eğin. Damanın tamamı içeride kalsın.'
                    good = False
                elif any(np.mean(np.linalg.norm(points[0]-old,axis=2))<12 for old in self.saved_points):
                    message = 'Bu konum kaydedildi. Telefonu azıcık farklı bir konuma alın.'
                    good = False
                elif time.monotonic() < self.cooldown:
                    message = 'Kaydedildi ✓ Şimdi yeni konuma geçin.'
                    good = False
                else:
                    sharpness=[board_sharpness(g,p) for g,p in zip(gray,points)]
                    progress, chosen = self.gate.update(packet,points,sharpness)
                    message, color = ('Kareleri topluyorum; küçük titremeler sorun değil…' if progress<1 else 'En iyi anı seçiyorum. Telefonu rahatça tutun.'), '#64e5c0'
                    if chosen is not None:
                        directory = self.session/('training' if self.index<30 else 'validation')
                        save_pair(directory,chosen['packet'],chosen['points'],self.index,{'center':center,'mode':mode,'selection':chosen['quality']})
                        self.saved_points.append(chosen['points'][0].copy())
                        self.index += 1
                        self.manifest['poseCount'] = self.index
                        if self.index == 30:
                            self.restart_requested.set()
                        self.write_manifest()
                        self.root.bell()
                        self.gate.reset()
                        self.cooldown = time.monotonic()+2
                        if self.index == 40:
                            self.armed = False
                            self.step.config(text='Çekim tamamlandı ✓')
                            self.instruction.config(text='30 eğitim ve 10 doğrulama çifti kaydedildi. Kalibrasyon doğruluğu ayrıca hesaplanacak; çekimin tamamlanması doğruluk kanıtı değildir.')
                            self.button.config(text='Tamamlandı',state='disabled')
                            self.manifest['state']='capture-complete';self.write_manifest()
                        else:
                            self.update_instruction()
            # A missed target or hand tremor never erases the burst; samples expire by age.
            if not good and self.gate.start is not None:
                progress=min(1.,max(0.,(ts/1e9-self.gate.start)/1.5))
        self.status.config(text=message,fg=color)
        self.bar.delete('all')
        self.bar.create_rectangle(0,0,max(0,self.bar.winfo_width()*progress),12,fill='#64e5c0',outline='')

    def poll(self):
        try:
            self.render_preview(*self.preview_queue.get_nowait())
        except queue.Empty:
            pass
        try:
            while True:
                item = self.messages.get_nowait()
                if item[0] == 'error':
                    self.status.config(text=item[1],fg='#fbbf24')
                else:
                    self.process(*item[1:])
        except queue.Empty:
            pass
        if not self.closed.is_set():
            self.root.after(35,self.poll)

    def close(self):
        self.closed.set()
        self.manifest['state'] = 'invalidated-by-target-resize' if self.invalidated else ('capture-complete' if self.index == 40 else 'closed-incomplete')
        self.write_manifest()
        self.root.destroy()
        if self.started:
            subprocess.run(['adb','shell','am','force-stop','org.research.phonestereo'],timeout=10,capture_output=True)
            subprocess.run(['adb','forward','--remove','tcp:8765'],timeout=10,capture_output=True)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--demo',action='store_true')
    parser.add_argument('--resume',type=Path,help='Resume a session without losing saved poses')
    parser.add_argument('--new',action='store_true',help='Start a new session instead of resuming saved poses')
    args=parser.parse_args()
    if args.resume is None and not args.new and not args.demo:
        for manifest in sorted((PROJECT/'data'/'guided').glob('*/session.json'),reverse=True):
            old=json.loads(manifest.read_text())
            if 0<old.get('poseCount',0)<40 and old.get('state')!='invalidated-by-target-resize' and not old.get('demo'):
                args.resume=manifest.parent
                break
    cv2.setNumThreads(2)
    lock=(PROJECT/'work'/'assistant.lock').open('w')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        from tkinter import messagebox
        messagebox.showinfo('Stereo Rehber','Uygulama zaten açık. Açık pencereyi kullanın.')
        return
    root=tk.Tk()
    app=Assistant(root,args.demo,session_directory=args.resume,resume=args.resume is not None)
    import signal
    signal.signal(signal.SIGTERM,lambda *_: root.after(0,app.close))
    root.mainloop()


if __name__ == '__main__':
    main()
