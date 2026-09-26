"""Native recorded-result viewer + bounded live SGBM pipeline over USB."""
import argparse,json,subprocess,threading,time,fcntl,queue,uuid,hashlib,sys
from pathlib import Path
import tkinter as tk
import cv2,numpy as np
from PIL import Image,ImageTk
from host.contracts import check_geometry
from host.pipeline import LivePipeline, LatestOutputs
from host.diagnostics import Diagnostics
from host.device import run_command,timing_args
from host.depth import StereoProcessor
from host.pointcloud import save_cloud,disparity_support_mask,points_from_z

PROJECT=Path(__file__).resolve().parents[1]

def region_stats(z,valid,x,y,radius=20):
    x,y=int(x),int(y)
    if not (0<=x<z.shape[1] and 0<=y<z.shape[0]):return None
    sl=np.s_[max(0,y-radius):min(z.shape[0],y+radius+1),max(0,x-radius):min(z.shape[1],x+radius+1)]
    mask=valid[sl]&np.isfinite(z[sl])&(z[sl]>0)
    values=z[sl][mask]
    if len(values)<20:return None
    return {'median':float(np.median(values)),'validPixels':int(len(values)),'areaPixels':int(mask.size)}


class Viewer:
    def __init__(self,root,calibration,results,live=False,logdir=None,wide_fps=15,depth_scale=1.):
        if depth_scale not in (1.,.5):raise ValueError('Depth scale must be 1 or 0.5')
        self.depth_scale=depth_scale
        self.timing_args=timing_args(wide_fps);self.wide_fps=wide_fps
        self.root=root;self.results=results;self.c=np.load(calibration)
        if str(self.c['lengthUnit'])!='checker_square':raise ValueError('Bu arayüz sürümü yalnız checker_square birimini destekler; metre kalibrasyonu için birim arayüzü güncellenmeli.')
        self.report=json.loads(calibration.with_suffix('.json').read_text())
        if self.report['heldoutVerticalErrorPx']['p95']>1.5:raise ValueError('Kalibrasyon kabul sınırını aşmış.')
        self.processor=StereoProcessor(self.c,output_scale=depth_scale)
        self.calibration_hash=hashlib.sha256(calibration.read_bytes()).hexdigest()
        self.cloud_frame=None;self.current_row=None;self.export_busy=False;self.export_events=queue.Queue(maxsize=1)
        self.last_cloud_path=None;self.cloud_process=None
        self.live_cloud=None;self.last_cloud_sequence=None
        self.messages=LatestOutputs();self.stop=threading.Event();self.worker=None
        self.pipeline=None;self.last_callback_age=0.
        self.pending_action=None;self.stop_poll_pending=False;self.closing=False
        self.mode='recorded';self.method='raft';self.position=None;self.z=None;self.mask=None
        self.last_result=0.;self.transform=None;self.depth_rgb=None;self.photos=[];self.color_limits=(15.,150.)
        self.pair_rect=None;self.pair_description='';self.inspector=None
        self.logdir=Path(logdir) if logdir is not None else PROJECT/'data/live'/time.strftime('%Y%m%d_%H%M%S');self.logdir.mkdir(parents=True,exist_ok=True)
        root.title('Stereo Derinlik · Gerçek kamera sonuçları')
        root.geometry('1800x1100');root.configure(bg='#111827')
        root.protocol('WM_DELETE_WINDOW',self.close)
        top=tk.Frame(root,bg='#111827');top.pack(fill='x',padx=20,pady=12)
        tk.Label(top,text='STEREO DERİNLİK',font=('DejaVu Sans',23),fg='white',bg='#111827').pack(side='left',padx=10)
        for title,command in [('Çift kontrolü',self.open_inspector),('Kayıtlı RAFT',lambda:self.recorded('raft')),('Kayıtlı SGBM',lambda:self.recorded('sgbm')),('Canlı SGBM',self.live),('Kapat',self.close)]:
            tk.Button(top,text=title,command=command,bg='#254257',fg='white',font=('DejaVu Sans',13),relief='flat',padx=14,pady=8).pack(side='right',padx=5)
        self.status=tk.Label(root,text='',fg='#64e5c0',bg='#111827',font=('DejaVu Sans',15));self.status.pack(fill='x')
        self.pair_note=tk.Label(root,text='Derinlik ve kamera görüntüleri aynı hesaplanan çifte aittir.',fg='#b6c7d9',bg='#111827',font=('DejaVu Sans',11));self.pair_note.pack(fill='x')
        body=tk.Frame(root,bg='#111827');body.pack(fill='both',expand=True,padx=20,pady=10)
        cameras=tk.Frame(body,bg='#111827',width=440);cameras.pack(side='left',fill='y');cameras.grid_propagate(False)
        cameras.columnconfigure(0,weight=1)
        for row in (1,3):cameras.rowconfigure(row,weight=1,uniform='camera')
        self.camera_views=[]
        for i,text in enumerate(['ÜST KAMERA · telefoto','ALT KAMERA · geniş açı']):
            tk.Label(cameras,text=text,fg='#b6c7d9',bg='#111827',font=('DejaVu Sans',13)).grid(row=i*2,column=0,pady=4)
            view=tk.Label(cameras,bg='#080f1b',width=1,height=1);view.grid(row=i*2+1,column=0,sticky='nsew',pady=8);self.camera_views.append(view)
        depth=tk.Frame(body,bg='#111827');depth.pack(side='left',fill='both',expand=True,padx=20)
        tk.Label(depth,text='DERİNLİK · bir bölgeye tıklayın',fg='white',bg='#111827',font=('DejaVu Sans',16)).pack()
        self.canvas=tk.Canvas(depth,bg='#080f1b',highlightthickness=0,width=600,height=760);self.canvas.pack(fill='both',expand=True)
        self.canvas.bind('<Button-1>',self.select)
        self.legend=tk.Label(depth,text='',fg='#b6c7d9',bg='#111827',font=('DejaVu Sans',12));self.legend.pack()
        details=tk.Frame(body,bg='#111827',width=340);details.pack(side='right',fill='y')
        self.readout=tk.Label(details,text='Haritadan bir bölge seçin.',fg='#64e5c0',bg='#111827',font=('DejaVu Sans',21),wraplength=320,justify='left');self.readout.pack(pady=40)
        self.quality_readout=tk.Label(details,text='',fg='#b6c7d9',bg='#111827',font=('DejaVu Sans',12),wraplength=320,justify='left');self.quality_readout.pack(pady=10)
        tk.Button(details,text='Nokta bulutunu kaydet',command=self.export_cloud).pack(pady=8)
        tk.Button(details,text='Uzak noktaları eleyip kaydet',command=lambda:self.export_cloud(8.)).pack(pady=4)
        tk.Button(details,text='Kaydedilmiş bulutu 3B aç',command=self.open_saved_cloud).pack(pady=4)
        tk.Button(details,text='Canlı 3B aç',command=self.open_live_cloud).pack(pady=4)
        self.export_note=tk.Label(details,text='',fg='#b6c7d9',bg='#111827',wraplength=320,justify='left');self.export_note.pack()
        tk.Label(details,text='Birim: dama karesi\n\nMetre ölçeği ölçülmedi.\nGösterilen değer, sol kameranın optik ekseni boyunca Z derinliğidir.\n\nKoyu bölgelerde geçerli eşleşme yoktur.\n\nRenkler: mor daha yakın, sarı daha uzak.\n\nSabit veya yavaş sahneler için araştırma prototipi. Hareketli sahne doğruluğu ölçülmedi.',fg='#a7b8cb',bg='#111827',font=('DejaVu Sans',13),wraplength=320,justify='left').pack()
        root.after(100,lambda:self.live() if live else self.recorded('raft'))
        root.after(50,self.poll)
        root.after(200,self.poll_export)
        root.after(500,self.poll_live_cloud)

    def put(self,item):
        self.messages.put(item)

    def stop_then(self, action):
        self.stop.set()
        if self.pipeline is not None:self.pipeline.request_stop()
        self.pending_action=action
        if not self.stop_poll_pending:self.poll_stop()

    def poll_stop(self):
        if self.worker is not None and self.worker.is_alive():
            self.stop_poll_pending=True
            self.status.config(text='Akış kapatılıyor…')
            self.root.after(50,self.poll_stop)
            return
        self.stop_poll_pending=False;self.worker=None
        action=self.pending_action;self.pending_action=None
        if action is not None:action()

    def recorded(self,method):
        if not self.closing:self.stop_then(lambda:self.show_recorded(method))

    def show_recorded(self,method):
        self.mode='recorded';self.method=method
        self.messages.clear()
        self.clear_pair()
        rect=[cv2.imread(str(self.results/f'rectified-{c}.png'),0) for c in ('left','right')]
        z=np.load(self.results/method/'depth_z_checker_square.npy');mask=np.isfinite(z)&(z>0)
        if any(im is None for im in rect):raise ValueError('Kayıtlı görüntüler bulunamadı.')
        self.status.config(text=f'KAYITLI {method.upper()} · geçerli eşleşme %{mask.mean()*100:.1f} · metre doğruluğu ölçülmedi')
        self.color_limits=tuple(np.percentile(z[mask],[2,98])) if mask.any() else (15.,150.)
        self.pair_description=f'Kayıtlı {method.upper()} · hesaplanan düzeltilmiş çift'
        self.show(rect,z,mask)

    def live(self):
        if not self.closing:self.stop_then(self.start_live)

    def start_live(self):
        self.messages.clear();self.stop=threading.Event();self.mode='live';self.method='sgbm';self.last_result=0
        self.clear_pair()
        self.readout.config(text='Canlı bağlantı hazırlanıyor…');self.color_limits=(15.,150.)
        self.worker=threading.Thread(target=self.capture,daemon=True);self.worker.start()

    def capture(self):
        started=False
        session_dir=self.logdir/time.strftime('%Y%m%d_%H%M%S')
        session_dir.mkdir(parents=True,exist_ok=True)
        diagnostics=Diagnostics(session_dir/'events.jsonl')
        diagnostics.event('configuration',requestedWideFps=self.wide_fps,depthScale=self.depth_scale,
                          depthSize=list(self.processor.size),sourceSize=list(self.processor.input_size))
        def adb(*args):return run_command(['adb',*args],self.stop)
        def process(blobs):
            images=[cv2.imdecode(np.frombuffer(b,np.uint8),0) for b in blobs]
            if any(im is None for im in images):raise ValueError('JPEG çözülemedi.')
            rect=self.processor.rectify(images)
            _,z,mask,ms,quality=self.processor.compute_diagnostics(rect,disparities=int(128*self.depth_scale),
                lr_tolerance=self.depth_scale,speckle_window=int(100*self.depth_scale**2))
            return rect,z,mask,ms,quality
        pipeline=LivePipeline(process,lambda h:check_geometry(h,self.report,self.processor.input_size),
                              diagnostics=diagnostics,outputs=self.messages)
        self.pipeline=pipeline
        try:
            self.put(('status','Telefon kameraları hazırlanıyor…'))
            adb('shell','am','force-stop','org.research.phonestereo');adb('forward','tcp:8765','tcp:8765')
            started=True
            adb('shell','am','start','-n','org.research.phonestereo/.MainActivity','--es','ids','20,21','--ei','seconds','3600',
                '--ei','width','1280','--ei','height','960','--ez','fixed','true','--ez','live','true','--ef','focus','1.4','--ei','iso','400',*self.timing_args)
            if not self.stop.is_set():pipeline.start()
            while not self.stop.wait(.1) and not pipeline.stop.is_set():pass
        except Exception as error:
            if not self.stop.is_set():self.put(('status',f'Canlı akış durdu: {error}'))
        finally:
            pipeline.request_stop()
            try:
                pipeline.join()
                (session_dir/'session.json').write_text(json.dumps(diagnostics.snapshot(),indent=2))
            finally:
                diagnostics.close()
                if started:
                    for args in [('shell','am','force-stop','org.research.phonestereo'),('forward','--remove','tcp:8765')]:
                        try:subprocess.run(['adb',*args],capture_output=True,timeout=3)
                        except subprocess.TimeoutExpired:pass
                self.pipeline=None

    def camera_images(self,rect):
        for view,im in zip(self.camera_views,rect):
            image=Image.fromarray(cv2.rotate(im,cv2.ROTATE_90_CLOCKWISE));image.thumbnail((max(160,view.winfo_width()),max(140,view.winfo_height())))
            photo=ImageTk.PhotoImage(image);view.config(image=photo);view.image=photo

    def show(self,rect,z,mask):
        self.pair_rect=rect
        # Keep native coordinates and exact pair references; display rotation is unrelated to 3D.
        cal=self.processor.c if self.mode=='live' else self.c
        provenance=dict(unit=str(cal['lengthUnit']),method=self.method,source=self.mode,
                        calibrationSha256=self.calibration_hash,frameMetadata=self.current_row if self.mode=='live' else None,
                        rectificationP2=cal['P2'].tolist(),outputScale=float(cal['size'][0]/self.c['size'][0]))
        self.cloud_frame=(z,mask,rect[0],cal['P1'],provenance)
        self.pair_note.config(text=self.pair_description)
        self.camera_images(rect)
        self.z=cv2.rotate(z,cv2.ROTATE_90_CLOCKWISE);self.mask=cv2.rotate(mask.astype(np.uint8),cv2.ROTATE_90_CLOCKWISE)>0
        lo,hi=self.color_limits
        scaled=np.clip((np.nan_to_num(self.z,nan=lo)-lo)/max(.01,hi-lo)*255,0,255).astype(np.uint8)
        self.legend.config(text=f'Renk aralığı: {lo:.1f}–{hi:.1f} dama karesi · koyu alanlar maskeli')
        rgb=cv2.cvtColor(cv2.applyColorMap(scaled,cv2.COLORMAP_VIRIDIS),cv2.COLOR_BGR2RGB);rgb[~self.mask]=[8,15,27]
        self.depth_rgb=rgb
        if self.position is None:self.position=(self.z.shape[1]//2,self.z.shape[0]//2)
        self.draw_depth();self.update_region()
        self.draw_inspector()

    def clear_pair(self):
        if getattr(self,'live_cloud',None) is not None:self.live_cloud.mailbox.clear()
        self.last_result=0.;self.z=None;self.mask=None;self.depth_rgb=None;self.transform=None
        self.position=None
        self.cloud_frame=None;self.current_row=None
        self.pair_rect=None;self.pair_description='Hesaplanmış kamera çifti bekleniyor.'
        self.pair_note.config(text=self.pair_description)
        self.quality_readout.config(text='')
        self.readout.config(text='Uygun çift bekleniyor…')
        self.canvas.delete('all')
        for view in self.camera_views:view.config(image='');view.image=None
        self.draw_inspector()

    def export_cloud(self,min_disparity=0.):
        if self.export_busy:return
        if self.cloud_frame is None:
            self.export_note.config(text='Önce uygun bir derinlik çifti bekleyin.');return
        if self.mode=='live':
            row=self.current_row
            p=self.pipeline
            if row is None or p is None or time.monotonic()-self.last_result+self.last_callback_age>1:
                self.export_note.config(text='Son çift güncel değil; yeni çift bekleyin.');return
            with p.condition:
                if p.stop.is_set() or row['epoch']!=p.epoch or row['sourceRun']!=p.source:
                    self.export_note.config(text='Kamera oturumu değişti; yeni çift bekleyin.');return
        snapshot=self.cloud_frame
        directory=PROJECT/'data/pointcloud'/('snapshot_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
        self.export_busy=True;self.export_note.config(text='Nokta bulutu kaydediliyor…')
        def write():
            try:
                z,mask,left,p1,provenance=snapshot
                if min_disparity:
                    mask,stats=disparity_support_mask(z,mask,provenance['rectificationP2'],provenance['outputScale'],min_disparity)
                    provenance=dict(provenance,cloudFilter=stats)
                report=save_cloud(directory,z,mask,left,p1,provenance)
                self.export_events.put(('ok',directory,report))
            except Exception as error:self.export_events.put(('error',str(error),0))
        threading.Thread(target=write,name='pointcloud-export',daemon=False).start()

    def poll_export(self):
        try:
            kind,value,result=self.export_events.get_nowait();self.export_busy=False
            if kind=='ok':self.last_cloud_path=value
            if kind=='ok':
                stats=result['provenance'].get('cloudFilter')
                extra=f"\n{stats['removed']:,} uzak nokta elendi ({stats['originalPixelThreshold']:g} px eşiği)." if stats else ''
                self.export_note.config(text=f"{result['points']:,} nokta kaydedildi.{extra}\n{value}/cloud.ply")
            else:self.export_note.config(text=f'Kaydedilemedi: {value}')
        except queue.Empty:pass
        if not self.closing:self.root.after(200,self.poll_export)

    def open_saved_cloud(self):
        if self.cloud_process is not None and self.cloud_process.poll() is None:
            self.export_note.config(text='3B pencere zaten açık. Başka bulut açmak için önce onu kapatın.');return
        directory=self.last_cloud_path
        if directory is None:
            from tkinter import filedialog
            selected=filedialog.askopenfilename(parent=self.root,title='Kaydedilmiş cloud.ply seçin',
                initialdir=PROJECT/'data/pointcloud',filetypes=[('Nokta bulutu','cloud.ply')])
            if not selected:return
            directory=Path(selected).parent
        try:
            from host.pointcloud import read_cloud
            read_cloud(directory)
            self.cloud_process=subprocess.Popen([sys.executable,'-m','host.cloud_view',str(directory)],cwd=PROJECT)
            self.export_note.config(text='Kaydedilmiş tek kare ayrı 3B pencerede açıldı.')
        except (OSError,ValueError,KeyError) as error:self.export_note.config(text=f'Bulut açılamadı: {error}')

    def open_live_cloud(self):
        if self.live_cloud is not None:
            if self.live_cloud.alive():return
            if self.live_cloud.process.is_alive():
                self.export_note.config(text='Önceki 3B pencere kapanıyor…');return
            self.live_cloud.process.join(timeout=0)
        from host.live_cloud import LiveCloudWindow
        self.live_cloud=LiveCloudWindow();self.last_cloud_sequence=None

    def poll_live_cloud(self):
        try:
            window=self.live_cloud
            if window is not None and window.alive():
                row=self.current_row;pipeline=self.pipeline
                valid=self.mode=='live' and self.cloud_frame is not None and row is not None and pipeline is not None
                expires=0.
                if valid:
                    expires=row['hostCompletionMonotonic']+1.-row.get('freshnessAgeUpperMs',row['callbackAgeUpperMs'])/1000
                    with pipeline.condition:
                        valid=not pipeline.stop.is_set() and row['epoch']==pipeline.epoch and row['sourceRun']==pipeline.source
                    valid=valid and expires>time.monotonic()
                if not valid:
                    window.mailbox.clear();self.last_cloud_sequence=None
                else:
                    sequence=(row['sourceRun'],row['epoch'],row['leftTimestampNs'],row['rightTimestampNs'])
                    if sequence!=self.last_cloud_sequence:
                        z,mask,left,p1,provenance=self.cloud_frame
                        mask,stats=disparity_support_mask(z,mask,provenance['rectificationP2'],provenance['outputScale'],8.)
                        xyz,_,_=points_from_z(z,mask,left,p1)
                        if len(xyz)>window.mailbox.capacity:
                            xyz=xyz[np.linspace(0,len(xyz)-1,window.mailbox.capacity,dtype=int)]
                        # Expiry includes conversion and IPC. It is never reset to arrival time.
                        with pipeline.condition:
                            if pipeline.stop.is_set() or row['epoch']!=pipeline.epoch or row['sourceRun']!=pipeline.source:
                                window.mailbox.clear();self.last_cloud_sequence=None
                            elif window.mailbox.publish(xyz,expires,stats['kept'],stats['before']):
                                self.last_cloud_sequence=sequence
        except (ValueError,OSError,RuntimeError) as error:
            if self.live_cloud is not None:self.live_cloud.close()
            self.export_note.config(text=f'Canlı 3B durdu: {error}')
        if not self.closing:self.root.after(500,self.poll_live_cloud)

    def open_inspector(self):
        if self.inspector is not None and self.inspector.winfo_exists():
            self.inspector.lift();return
        self.inspector=tk.Toplevel(self.root);self.inspector.title('Aynı çift · hizalama kontrolü')
        self.inspector_label=tk.Label(self.inspector,text='',bg='#111827',fg='white')
        self.inspector_label.pack()
        self.inspector_note=tk.Label(self.inspector,text='',wraplength=1280,justify='left')
        self.inspector_note.pack(fill='x')
        self.draw_inspector()

    def draw_inspector(self):
        if self.inspector is None or not self.inspector.winfo_exists():return
        self.inspector_note.config(text=self.pair_description+'\nSol: telefoto; sağ: geniş açı. İşleme yönü korunur. Aynı nesne iki görüntüde aynı renkli yatay çizgiye denk gelmeli.')
        if self.pair_rect is None:
            self.inspector_label.config(image='',text='Uygun çift bekleniyor…');self.inspector_label.image=None;return
        # Use the depth worker's exact remapped pair; never the independent raw preview.
        images=[cv2.resize(im,(640,480),interpolation=cv2.INTER_AREA) for im in self.pair_rect]
        rgb=cv2.cvtColor(np.hstack(images),cv2.COLOR_GRAY2RGB)
        for i,y in enumerate(range(30,480,40)):
            cv2.line(rgb,(0,y),(1279,y),[(255,170,50),(70,220,240)][i%2],1)
        photo=ImageTk.PhotoImage(Image.fromarray(rgb))
        self.inspector_label.config(image=photo,text='');self.inspector_label.image=photo

    def draw_depth(self):
        im=Image.fromarray(self.depth_rgb);w,h=self.canvas.winfo_width(),self.canvas.winfo_height();im.thumbnail((max(100,w),max(100,h)))
        x,y=(w-im.width)//2,(h-im.height)//2;self.transform=(x,y,im.width/self.depth_rgb.shape[1])
        self.canvas.delete('all');photo=ImageTk.PhotoImage(im);self.canvas.create_image(x,y,image=photo,anchor='nw');self.canvas.image=photo
        px,py=self.position;scale=self.transform[2];r=20*scale
        self.canvas.create_rectangle(x+px*scale-r,y+py*scale-r,x+px*scale+r,y+py*scale+r,outline='#ff8b54',width=2)

    def select(self,event):
        if self.transform is None:return
        x,y,scale=self.transform;px,py=(event.x-x)/scale,(event.y-y)/scale
        if 0<=px<self.z.shape[1] and 0<=py<self.z.shape[0]:self.position=(int(px),int(py));self.draw_depth();self.update_region()

    def update_region(self):
        if self.mode=='live' and time.monotonic()-self.last_result+self.last_callback_age>1:
            self.pair_note.config(text='Son hesaplanan çift güncel değil; yeni uygun çift bekleniyor.')
            if self.inspector is not None and self.inspector.winfo_exists():
                self.inspector_note.config(text='SON ÇİFT GÜNCEL DEĞİL · '+self.pair_description)
            self.readout.config(text='Yeni uygun çift bekleniyor.\nSon harita güncel değil.');return
        stats=region_stats(self.z,self.mask,*self.position)
        self.readout.config(text=(f"Bölge medyanı\n{stats['median']:.2f}\ndama karesi\n\n{stats['validPixels']} / {stats['areaPixels']} piksel geçerli" if stats else 'Bu bölgede yeterli\ngeçerli eşleşme yok.'))

    def poll(self):
        items=self.messages.drain()
        if 'reset' in items:
            self.clear_pair()
        if 'depth' in items:
            _,rect,z,mask,row=items['depth']
            self.current_row=row
            self.last_result=row['hostCompletionMonotonic'];self.last_callback_age=row.get('freshnessAgeUpperMs',row['callbackAgeUpperMs'])/1000
            self.pair_description=f"Aynı hesaplanan çift · {rect[0].shape[1]}×{rect[0].shape[0]} · çift farkı {row['deltaMs']:.1f} ms"
            self.show(rect,z,mask)
            quality=row.get('quality')
            if quality:
                n=quality['totalPixels'];stages=quality['stages']
                self.quality_readout.config(text='Tüm görüntüde kalan pikseller\n'+
                    '\n'.join(f'{label}: %{100*stages[key]/n:.1f}' for key,label in [('leftMatcher','İlk eşleşme'),('leftRight','İki yönde tutarlı'),('positiveFiniteZ','Geçerli derinlik')]))
            self.status.config(text=f"CANLI SGBM · çift farkı {row['deltaMs']:.1f} ms · eşleştirici {row['matcherMs']:.0f} ms · geçerli %{row['validPixelRatio']*100:.1f}")
        # Preview traffic is still measured by the receiver, but cannot overwrite
        # either image belonging to the displayed depth. Reset clears all three.
        if 'status' in items:self.status.config(text=items['status'][1])
        if self.z is not None and self.mode=='live':self.update_region()
        self.root.after(60,self.poll)

    def close(self):
        self.closing=True
        if self.live_cloud is not None:self.live_cloud.close()
        self.stop_then(self.root.destroy)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--live',action='store_true');ap.add_argument('--run-seconds',type=int)
    ap.add_argument('--calibration',type=Path,default=PROJECT/'data/calibration/screen_20260910/calibration.npz')
    ap.add_argument('--results',type=Path,default=PROJECT/'data/depth/screen-result')
    ap.add_argument('--wide-fps',type=int,choices=(15,30),default=15,help='Opt-in camera21 timing profile; tele camera remains15FPS')
    ap.add_argument('--depth-scale',type=float,choices=(1.,.5),default=1.,help='Opt-in 640x480 depth; camera source remains1280x960')
    args=ap.parse_args()
    cv2.setNumThreads(2)
    lock=(PROJECT/'work/assistant.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    root=tk.Tk();viewer=Viewer(root,args.calibration,args.results,args.live,wide_fps=args.wide_fps,depth_scale=args.depth_scale)
    if args.run_seconds:root.after(args.run_seconds*1000,viewer.close)
    root.mainloop()
if __name__=='__main__':main()
