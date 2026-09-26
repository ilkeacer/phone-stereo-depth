"""Local Tk control panel for bounded ROS phone experiments."""
import json
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import tempfile
import threading
import time
import tkinter as tk
from tkinter import ttk
from PIL import Image,ImageTk
from host.ros_guidance import (mapping_guidance, IDLE_TEXT, PREPARATION, STEPS,
                               ROOM_PREPARATION, ROOM_STEPS, TEST_SECONDS, ROOM_SECONDS)
from host.ros_map_catalog import available,saved_maps,localization_maps,display_name
from host.ros_scale import default_scale,scale_label
from host.scene_quality import brightness_signal

PROJECT=Path(__file__).resolve().parents[1]


def current_scale_label():
    try:return scale_label(default_scale(PROJECT/'data/calibration/screen_20260910/calibration.npz'))
    except (OSError,ValueError,KeyError,TypeError):return 'Ölçek ayarı okunamadı · Başlatmadan önce yapılandırmayı kontrol et'


def latest(q,value):
    try:q.get_nowait()
    except queue.Empty:pass
    q.put_nowait(value)


class Dashboard:
    def __init__(self,root):
        self.root=root;self.process=None;self.directory=None;self.closing=False;self.stopping_at=None
        self.messages=queue.Queue(maxsize=1);self.images=queue.Queue(maxsize=1);self.shutdown=threading.Event()
        self.mode='live';self.localize_map_db=None;self.viewers=[];self.last_quality=None;self.last_odom_time=0
        self.guide_dialog=None;self.guide_duration=None;self.guide_engine=None;self.depth_engine='auto';self.last_guide_title=None
        self.capture_seconds=TEST_SECONDS
        self.last_image=0;self.last_scene=None;self.preview_seen=False;self.preview_rotated=True
        self.log_position=0;self.status_position=0;self.tracking=0;self.lost=0;self.photo=None;self.last_message=''
        root.title('Telefon stereo · ROS haritalama');root.geometry('1160x1050');root.minsize(1040,1010)
        root.configure(bg='#14202e')
        tk.Label(root,text='Telefonla haritalama',font=('DejaVu Sans',24,'bold'),bg='#14202e',fg='white').pack(anchor='w',padx=24,pady=(20,4))
        self.scale_status=tk.Label(root,text=current_scale_label(),font=('DejaVu Sans',11),bg='#14202e',fg='#c4d0de')
        self.scale_status.pack(anchor='w',padx=24)
        serial=os.environ.get('PHONE_ADB_SERIAL')
        self.link_status=tk.Label(root,text='Telefon bağlantısı: Wi-Fi ADB' if serial and ':' in serial else
                                  'Telefon bağlantısı: USB / otomatik ADB',font=('DejaVu Sans',11),
                                  bg='#14202e',fg='#c4d0de')
        self.link_status.pack(anchor='w',padx=24)
        body=tk.Frame(root,bg='#14202e');body.pack(fill='both',expand=True,padx=24,pady=18)
        left=tk.Frame(body,bg='#14202e');left.pack(side='left',fill='both',expand=True,padx=(0,20))
        self.preview=tk.Label(left,text='Kamera başlatılmadı',bg='#0b131e',fg='#c4d0de',font=('DejaVu Sans',14))
        self.preview.pack(fill='both',expand=True)
        preview_controls=tk.Frame(left,bg='#14202e');preview_controls.pack(fill='x',pady=(8,4))
        self.preview_button=tk.Button(preview_controls,text='Kamera önizlemesi · 30 sn (harita yok)',command=self.start_preview,
                                      font=('DejaVu Sans',12),pady=5)
        self.preview_button.pack(side='left',fill='x',expand=True)
        self.rotate_button=tk.Button(preview_controls,text='Yatay göster',command=self.toggle_preview_rotation,
                                     font=('DejaVu Sans',11),pady=5)
        self.rotate_button.pack(side='left',padx=(8,0))
        tk.Label(left,text='EKRANDAKİ ADIMI TAKİP ET',bg='#14202e',fg='#6ce4b2',font=('DejaVu Sans',14,'bold')).pack(anchor='w',pady=(16,5))
        self.detail=tk.Label(left,text=IDLE_TEXT,bg='#14202e',fg='white',font=('DejaVu Sans',14),justify='left',wraplength=650,anchor='nw')
        self.detail.pack(fill='x',pady=(0,12))
        left.bind('<Configure>',lambda event:self.detail.config(wraplength=max(300,event.width-8)))
        self.route=tk.Label(left,text='0–5 sn: SABİT  →  5–35: YANA  →  35–55: İLERİ\n55–85: GERİ DÖN  →  85–90: SABİT',bg='#14202e',fg='#c4d0de',font=('DejaVu Sans',12),justify='left')
        self.route.pack(anchor='w')
        panel=tk.Frame(body,bg='#14202e',width=345);panel.pack(side='right',fill='y');panel.pack_propagate(False)
        self.state=tk.Label(panel,text='Hazır',bg='#14202e',fg='#6ce4b2',font=('DejaVu Sans',20,'bold'),wraplength=320)
        self.state.pack(anchor='w',pady=(0,8))
        self.counter=tk.Label(panel,text='—',bg='#14202e',fg='white',font=('DejaVu Sans',22,'bold'),wraplength=335);self.counter.pack(anchor='w')
        self.total=tk.Label(panel,text='90 saniye · 5 aşama',bg='#14202e',fg='#c4d0de',font=('DejaVu Sans',12));self.total.pack(anchor='w',pady=(6,4))
        self.progress=ttk.Progressbar(panel,maximum=TEST_SECONDS);self.progress.pack(fill='x',pady=(0,6))
        self.health=tk.Label(panel,text='Takip sonucu bekleniyor',bg='#14202e',fg='#c4d0de',font=('DejaVu Sans',12),justify='left',wraplength=315);self.health.pack(anchor='w',pady=6)
        self.buttons=[]
        for label,diagnostic,engine,duration in [('90 sn rehberli oda testi',False,'sgbm',TEST_SECONDS),
                ('3 dk tüm odayı dolaş',False,'sgbm',ROOM_SECONDS),
                ('90 sn önceki stereo (alternatif)',False,'stereo',TEST_SECONDS),
                ('30 sn tanılama (ayrı test)',True,'auto',30)]:
            button=tk.Button(panel,text=label,command=lambda d=diagnostic,e=engine,s=duration:
                             self.start(True) if d else self.show_test_guide(e,s),font=('DejaVu Sans',12),pady=4)
            button.pack(fill='x',pady=2);self.buttons.append(button)
        self.buttons.append(self.preview_button)
        replay=tk.Button(panel,text='Kayıttan SGBM harita oluştur',command=lambda:self.start(False,replay=True,depth_engine='sgbm'),font=('DejaVu Sans',12),pady=4)
        replay.pack(fill='x',pady=3);self.buttons.append(replay)
        localization=tk.Button(panel,text='Kayıtlı haritada yerimi bul',command=self.choose_localization_map,font=('DejaVu Sans',12),pady=4)
        localization.pack(fill='x',pady=3);self.buttons.append(localization)
        tk.Button(panel,text='Test yönergelerini aç',command=self.show_test_guide,font=('DejaVu Sans',12),pady=4).pack(fill='x',pady=2)
        self.stop_button=tk.Button(panel,text='Durdur ve kaydet',command=self.stop,state='disabled',font=('DejaVu Sans',13),pady=5)
        self.stop_button.pack(fill='x',pady=3)
        tk.Button(panel,text='Bu haritayı 3B aç',command=self.open_map,font=('DejaVu Sans',13),pady=5).pack(fill='x',pady=3)
        tk.Button(panel,text='Kayıtlı haritalar',command=self.choose_map,font=('DejaVu Sans',13),pady=5).pack(fill='x',pady=3)
        tk.Button(panel,text='Anlık 3B derinlik (RViz)',command=self.open_local_cloud,font=('DejaVu Sans',13),pady=5).pack(fill='x',pady=3)
        tk.Button(panel,text='Biriken ROS haritası (RViz)',command=self.open_ros,font=('DejaVu Sans',13),pady=5).pack(fill='x',pady=3)
        self.footer=tk.Label(root,text='Başlat düğmesine basana kadar kameralar açılmaz.',bg='#14202e',fg='#c4d0de',font=('DejaVu Sans',10),wraplength=1040,justify='left')
        self.footer.pack(anchor='w',padx=24,pady=(0,18))
        root.protocol('WM_DELETE_WINDOW',self.close)
        self.receiver=threading.Thread(target=self.receive_images,daemon=True)
        self.receiver.start()
        root.after(200,self.poll)

    def show_test_guide(self,depth_engine='sgbm',duration=TEST_SECONDS):
        if duration not in (TEST_SECONDS,ROOM_SECONDS):raise ValueError('Unsupported guide duration')
        if self.guide_dialog is not None and self.guide_dialog.winfo_exists():
            if self.guide_duration==duration and self.guide_engine==depth_engine:self.guide_dialog.lift();return
            self.guide_dialog.destroy()
        dialog=tk.Toplevel(self.root);self.guide_dialog=dialog;self.guide_duration=duration;self.guide_engine=depth_engine
        room=duration==ROOM_SECONDS
        dialog.title('3 dakikalık tüm oda turu · hazırlık ve hareket rehberi' if room else
                     '90 saniyelik oda testi · hazırlık ve hareket rehberi')
        dialog.geometry('920x880');dialog.minsize(820,720);dialog.transient(self.root)
        dialog.configure(bg='#14202e')
        tk.Label(dialog,text='Önce oku, sonra testi başlat',font=('DejaVu Sans',21,'bold'),bg='#14202e',fg='white').pack(anchor='w',padx=20,pady=(18,6))
        tk.Label(dialog,text='Yeni dama çekimi yok · Cetvel gerekmiyor · Kamera henüz açılmadı' if self.process is None else 'Test sürüyor · Hareket yönergesi ana pencerede',
                 font=('DejaVu Sans',12),bg='#14202e',fg='#6ce4b2').pack(anchor='w',padx=20,pady=(0,12))
        tabs=ttk.Notebook(dialog);tabs.pack(fill='both',expand=True,padx=20,pady=(0,12))
        preparation='\n\n'.join(f'{i+1}. {text}' for i,text in enumerate(ROOM_PREPARATION if room else PREPARATION))
        preparation+='\n\nGEREKENLER: ADB bağlı telefon, aydınlık eşyalı sahne, başlangıç işareti ve '+('güvenli oda yolu' if room else 'kısa boş alan')+'.\n\n'+(
            'Bu test hareket sırasında bağlantılı bir harita oluşmasını deneyecek. '+
            current_scale_label()+'. '+
            'Tam oda ve santimetre doğruluğu bu tek kayıttan garanti edilemez.\n\n'
            'Yandaki “2 · Hareket sırası” sekmesini de oku. Başlatınca ana ekrandaki büyük adım ve sayacı takip et.')
        stages=ROOM_STEPS if room else STEPS
        movement='\n\n'.join(f'{i+1}/{len(stages)} · {start}–{end} sn · {title}\n{text}' for i,(start,end,title,text) in enumerate(stages))
        if depth_engine=='sgbm':
            movement+=('\n\nBaşlatınca iki RViz penceresi otomatik açılır. Panel önde kalır; 3B görünümü Alt+Tab ile seçebilirsin. '
                       'Anlık 3B derinlik kamera çevresindeki tek karelik noktaları, diğer pencere biriken haritayı gösterir. '
                       'Takip kaybı kaydedilir; deneysel harita, görüntü kaydı ve anlık 3B görüntü sürer. '
                       'Kayıp sonrası harita tek kesintisiz oda haritası sayılmaz; testi yeniden başlatma. ')
        else:
            movement+='\n\nTakip kaybında harita birikimi durur, görüntü kaydı sürer; testi yeniden başlatma. '
        movement+='DUR / AKIŞ KESİLDİ yazarsa hareketi durdur. '
        movement+='Erken bitirmek için “Durdur ve kaydet”. Normal bitişte kayıt otomatik durur, ardından harita kaydedilir. '
        movement+='Sonuç görünmeden pencereyi kapatma. Sonra “Bu haritayı 3B aç” düğmesini kullan.'
        for title,text in [('1 · Hazırlık',preparation),('2 · Hareket sırası',movement)]:
            frame=tk.Frame(tabs,bg='#1d2c3d');tabs.add(frame,text=title)
            scroll=ttk.Scrollbar(frame);scroll.pack(side='right',fill='y')
            content=tk.Text(frame,wrap='word',font=('DejaVu Sans',13),bg='#1d2c3d',fg='white',padx=18,pady=16,
                            relief='flat',spacing1=3,spacing3=3,yscrollcommand=scroll.set)
            content.pack(fill='both',expand=True);scroll.config(command=content.yview)
            content.insert('1.0',text);content.config(state='disabled')
        # Reserve controls before the expandable notebook, even at minimum size.
        footer=tk.Frame(dialog,bg='#14202e');footer.pack(side='bottom',fill='x',padx=20,pady=(0,18),before=tabs)
        def launch():
            dialog.destroy();self.guide_dialog=None
            if room:self.start(False,depth_engine=depth_engine,seconds=duration)
            else:self.start(False,depth_engine=depth_engine)
            if depth_engine=='sgbm':self.root.after(1500,self.open_live_viewers)
        if self.process is None:
            tk.Button(footer,text=('Hazırım · 3 dk tüm oda turunu başlat' if room else 'Hazırım · 90 sn testi başlat'+(' (önceki stereo)' if depth_engine=='stereo' else ' (SGBM)')),
                      command=launch,font=('DejaVu Sans',13,'bold'),pady=10,bg='#6ce4b2').pack(side='right')
        tk.Button(footer,text='Kapat · ana ekrana dön',command=dialog.destroy,font=('DejaVu Sans',12),pady=10).pack(side='left')

    def receive_images(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Image as RosImage
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('phone_dashboard')
        def receive(msg):
            if msg.encoding=='mono8' and 0<msg.width<=1280 and 0<msg.height<=960:
                latest(self.images,(bytes(msg.data),msg.width,msg.height,msg.step,time.monotonic()))
        node.create_subscription(RosImage,'/phone/left/image_rect',receive,qos_profile_sensor_data)
        try:
            while not self.shutdown.is_set():rclpy.spin_once(node,timeout_sec=.1)
        finally:node.destroy_node();rclpy.shutdown()

    def toggle_preview_rotation(self):
        """Rotate only the displayed image; ROS stereo pixels stay calibrated."""
        self.preview_rotated=not self.preview_rotated
        self.rotate_button.config(text='Yatay göster' if self.preview_rotated else 'Dik göster')

    def start(self,diagnostic,replay=False,depth_engine='auto',seconds=TEST_SECONDS,localize_map_db=None):
        if self.process is not None:return
        if seconds not in (TEST_SECONDS,ROOM_SECONDS):raise ValueError('Unsupported mapping duration')
        if self.guide_dialog is not None and self.guide_dialog.winfo_exists():self.guide_dialog.destroy()
        self.mode='replay' if replay else 'live'
        self.scale_status.config(text='Ölçek: seçilen kaydın kendi ayarı kullanılacak' if replay else current_scale_label())
        self.depth_engine=depth_engine;self.localize_map_db=localize_map_db;self.last_guide_title=None;self.capture_seconds=seconds
        self.directory=None;self.log_position=0;self.status_position=0;self.tracking=0;self.lost=0;self.stopping_at=None;self.last_quality=None;self.last_odom_time=0
        self.diagnostic=diagnostic;self.started=time.monotonic()
        env=dict(os.environ,PHONE_DIAGNOSTIC='1' if diagnostic else '0',
                 PHONE_CAPTURE_SECONDS='45' if diagnostic else str(seconds),PHONE_RTABMAP_VIZ='false',PHONE_DEPTH_ENGINE=depth_engine,
                 PHONE_TRACKING_LOSS_POLICY='continue-provisional' if depth_engine=='sgbm' and not diagnostic else 'keep-capture')
        if localize_map_db is not None:env['PHONE_LOCALIZE_MAP_DB']=str(localize_map_db)
        command=[str(PROJECT/('scripts/start_ros_replay.sh' if replay else 'scripts/start_ros_live_mapping.sh'))]
        if replay:command+=['--depth-engine',depth_engine,'--tracking-loss-policy',env['PHONE_TRACKING_LOSS_POLICY']]
        self.process=subprocess.Popen(command,cwd=PROJECT,env=env,
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
        for button in self.buttons:button.config(state='disabled')
        self.stop_button.config(state='normal',text='Durdur ve kaydet');self.state.config(text='Kameralar hazırlanıyor',fg='#ffd478');self.counter.config(text='Isınma…')
        self.health.config(text='Yeni oturum · takip sonucu bekleniyor')
        self.progress['maximum']=seconds;self.progress['value']=0
        self.total.config(text='30 saniye · 3 aşama' if diagnostic else 'Hazırlık süresi kayda dahil değil' if not replay else 'Kayıtlı görüntüler işleniyor')
        route=('0–5 sn: SABİT  →  5–145: ODADA YAVAŞ TUR\n145–175: BAŞLANGICA YAKLAŞ  →  175–180: SABİT'
               if seconds==ROOM_SECONDS else
               '0–5 sn: SABİT  →  5–35: YANA  →  35–55: İLERİ\n55–85: GERİ DÖN  →  85–90: SABİT')
        self.route.config(text=route if not diagnostic and not replay else '')
        self.detail.config(text=mapping_guidance(None,time.monotonic(),duration_seconds=seconds)[2] if not diagnostic
                           else 'Hazırlık sırasında sabit tut. Sonra üç aşamanın yönergelerini takip et.')
        if localize_map_db is not None:
            self.state.config(text='Eski haritada konum aranıyor')
            self.detail.config(text='Telefonu kayıtlı haritanın gördüğü aydınlık ve eşyalı bölgeye yönelt. Yavaşça hareket et. Eski haritayla kabul edilmiş görsel eşleşme olmadan konum doğrulanmış sayılmaz.')
            self.route.config(text='90 sn yavaş ve aydınlık oda turu · eşleşme durumunu izle')
        if replay:
            self.state.config(text='ROS hazırlanıyor');self.detail.config(text='Kayıtlı hareket kullanılacak. Telefon gerekmiyor.')
        def read(process):
            for line in process.stdout:
                if self.process is not process:continue
                line=line.strip()
                if line.startswith('Oturum kaydı: '):self.directory=Path(line.split(': ',1)[1].strip())
                self.last_message=line
                latest(self.messages,line)
        threading.Thread(target=read,args=(self.process,),daemon=True).start()

    def start_preview(self):
        """Publish a short camera view to the existing panel without a mapper."""
        if self.process is not None:return
        self.mode='preview';self.diagnostic=False;self.stopping_at=None;self.last_guide_title=None
        self.preview_seen=False;self.last_scene=None
        self.directory=Path(tempfile.mkdtemp(prefix='ros-preview-',dir=PROJECT/'work'))
        self.log_position=0;self.status_position=0;self.tracking=0;self.lost=0
        command=[os.sys.executable,'-m','host.ros_live',
                 '--calibration',str(PROJECT/'data/calibration/screen_20260910/calibration.npz'),
                 '--seconds','30','--report',str(self.directory/'capture.json')]
        self.process=subprocess.Popen(command,cwd=PROJECT,env=dict(os.environ),
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
        for button in self.buttons:button.config(state='disabled')
        self.stop_button.config(state='normal',text='Önizlemeyi durdur')
        self.state.config(text='Kamera önizlemesi hazırlanıyor',fg='#ffd478')
        self.counter.config(text='30 sn');self.total.config(text='Harita oluşturulmayacak')
        self.health.config(text='Telefon görüntüsü bekleniyor')
        self.detail.config(text='Telefonu DİK tut; ekrandaki önizleme de dik görünmeli. Gerekirse Yatay göster / Dik göster düğmesini kullan, telefonu görüntüye uydurmak için çevirme. Aydınlık ve eşyalı oda bölümüne bak. Perde, boş duvar ve parlak ekranı kadrajın çoğundan çıkar. Görüntü uygunsa 90 sn veya 3 dk harita testini başlatabilirsin.')
        self.route.config(text='Bu adımda telefonu sabit tutabilirsin · yalnız görüntü kontrolü')
        def read(process):
            for line in process.stdout:
                if self.process is process:latest(self.messages,line.strip())
        threading.Thread(target=read,args=(self.process,),daemon=True).start()

    def stop(self):
        if self.process is not None and self.process.poll() is None and self.stopping_at is None:
            self.stopping_at=time.monotonic();os.killpg(self.process.pid,signal.SIGINT)
            self.state.config(text='Durduruluyor…');self.stop_button.config(state='disabled')
            self.detail.config(text=('Kamera kapatılıyor. Önizleme sona erene kadar bekle.' if self.mode=='preview' else
                                     'Hareketi durdur. Kamera kapatılıyor, mevcut kayıt ve uygun harita verisi kaydediliyor. Sonuç görünene kadar bekle.'))

    def map_folder(self):
        if self.directory:
            reference=self.directory/'reference-map.json'
            if reference.exists():
                folder=Path(json.loads(reference.read_text())['referenceSession'])/'export'
                if available(folder):return folder
            folder=self.directory/'export'
            if available(folder):return folder
            folder=self.directory/'diagnostic-export'
            if available(folder):return folder
            self.footer.config(text='Bu oturumda açılabilir harita yok. Önceki sonuçlar için Kayıtlı haritalar düğmesini kullan.');return
        folders=self.saved_maps()
        if folders:return folders[0]
        self.footer.config(text='Kaydedilmiş harita bulunamadı.')

    def saved_maps(self):
        return saved_maps(PROJECT/'work')

    def show_map(self,folder,ros=False):
        command=([os.sys.executable,'-m','host.ros_map_publish'] if ros
                 else [str(PROJECT/'.venv/bin/python'),'-m','host.ros_map_view'])
        command += [str(folder/'map_cloud.ply'),str(folder/'map_poses.txt')]
        if ros:command+=['--rviz']
        self.viewers.append(subprocess.Popen(command,cwd=PROJECT,start_new_session=True))
        self.footer.config(text='Açılan harita: '+display_name(folder))

    def open_map(self):
        folder=self.map_folder()
        if folder:self.show_map(folder)

    def open_ros(self):
        if self.process is not None:
            self.viewers.append(subprocess.Popen(['rviz2','-d',str(PROJECT/'configs/phone-map.rviz')],cwd=PROJECT,start_new_session=True))
        else:
            folder=self.map_folder()
            if folder:self.show_map(folder,ros=True)

    def open_local_cloud(self):
        if self.process is None or self.depth_engine!='sgbm' or self.mode=='preview':
            self.footer.config(text='Anlık 3B derinlik için SGBM ile canlı oturum veya kayıt oynatımı başlatın.')
            return
        self.viewers.append(subprocess.Popen(['rviz2','-d',str(PROJECT/'configs/phone-live-depth.rviz')],cwd=PROJECT,start_new_session=True))
        self.footer.config(text='Anlık 3B: kameranın o anda gördüğü noktalar. Biriken oda haritası ayrı ROS görünümündedir.')

    def open_live_viewers(self):
        if self.process is None or self.process.poll() is not None or self.mode!='live' or self.depth_engine!='sgbm':return
        self.open_local_cloud();self.open_ros()
        self.root.after(300,self.root.lift)
        self.footer.config(text='İki RViz penceresi açıldı. Panel hareket adımlarını önde gösterir; 3B pencereler için Alt+Tab kullanabilirsiniz.')

    def choose_map(self):
        folders=self.saved_maps()
        if not folders:self.footer.config(text='Henüz kaydedilmiş harita yok.');return
        dialog=tk.Toplevel(self.root);dialog.title('Kayıtlı haritalar');dialog.geometry('720x380')
        tk.Label(dialog,text='Bir oturum seç. Her harita kendi kaydına aittir.',font=('DejaVu Sans',12)).pack(pady=12)
        listing=tk.Listbox(dialog,font=('DejaVu Sans',11));listing.pack(fill='both',expand=True,padx=12)
        for folder in folders:
            result_path=folder.parent/('diagnostic-export-result.json' if folder.name=='diagnostic-export' else 'export-result.json')
            result=json.loads(result_path.read_text()) if result_path.exists() else {}
            kind='deneysel parça · robot için geçersiz' if result.get('status')=='diagnostic_fragment' else 'parça' if result.get('status')=='partial' else 'harita'
            listing.insert('end',f"{display_name(folder)}   ·   {result.get('points','?')} nokta   ·   {result.get('poses','?')} poz   ·   {kind}")
        listing.selection_set(0)
        def show(ros):
            if listing.curselection():self.show_map(folders[listing.curselection()[0]],ros)
        tk.Button(dialog,text='3B aç',command=lambda:show(False)).pack(side='left',padx=12,pady=12)
        tk.Button(dialog,text='ROS / RViz içinde aç',command=lambda:show(True)).pack(side='left',padx=12,pady=12)

    def choose_localization_map(self):
        choices=localization_maps(PROJECT/'work')
        if not choices:
            self.footer.config(text='Konum bulma için kabul edilmiş, veritabanı bulunan bağlantılı harita yok.');return
        dialog=tk.Toplevel(self.root);dialog.title('Kayıtlı haritada konum bul');dialog.geometry('760x360')
        tk.Label(dialog,text='Aynı oda için kabul edilmiş haritayı seç. Kamera yalnız Başlat ile açılır.',
                 font=('DejaVu Sans',12)).pack(pady=12)
        listing=tk.Listbox(dialog,font=('DejaVu Sans',11));listing.pack(fill='both',expand=True,padx=12)
        for folder,_ in choices:
            listing.insert('end',('Canlı kayıt · ' if folder.parent.name.startswith('ros-live-') else 'Kayıttan oynatma · ')+display_name(folder))
        listing.selection_set(0)
        def launch():
            if not listing.curselection():return
            database=choices[listing.curselection()[0]][1]
            dialog.destroy()
            guide=tk.Toplevel(self.root);guide.title('Kayıtlı haritada konum bulma · 90 sn');guide.geometry('780x360')
            message=('Haritanın daha önce gördüğü aydınlık, eşyalı bölgeden başla. Telefonu dik tut; 5 saniye sabit kal, '
                     'sonra o bölge içinde yavaşça hareket et. RViz’de sarı çizgi kayıtlı rota, mor çizgi ise yalnız '
                     'eski haritayla kabul edilmiş eşleşmeden sonraki rotadır. Takip kaybı olursa kayda devam edebilirsin; '
                     'sonuçta kabul edilen eşleşme sayısı ayrı gösterilir. Bu test robotta santimetre doğruluğunu ölçmez.')
            tk.Label(guide,text=message,wraplength=730,justify='left',font=('DejaVu Sans',13)).pack(padx=20,pady=24)
            def begin():
                guide.destroy();self.start(False,depth_engine='sgbm',seconds=TEST_SECONDS,localize_map_db=database)
                self.root.after(1500,self.open_live_viewers)
            tk.Button(guide,text='Hazırım · konum bulmayı başlat',command=begin,font=('DejaVu Sans',13,'bold'),pady=10).pack(pady=10)
        tk.Button(dialog,text='Seçilen haritada konum bul',command=launch,font=('DejaVu Sans',12),pady=6).pack(pady=10)

    def poll(self):
        try:
            data,w,h,stride,stamp=self.images.get_nowait()
            image=Image.frombytes('L',(w,h),data,'raw','L',stride)
            self.last_scene=brightness_signal(data,w,h,stride)
            if self.mode=='preview' and self.process is not None:self.preview_seen=True
            if self.preview_rotated:image=image.transpose(Image.ROTATE_270)
            image.thumbnail((max(100,self.preview.winfo_width()),max(100,self.preview.winfo_height())))
            self.photo=ImageTk.PhotoImage(image);self.preview.config(image=self.photo,text='');self.last_image=stamp
        except queue.Empty:pass
        if self.last_image and time.monotonic()-self.last_image>2:
            self.preview.config(image='',text='Güncel görüntü yok');self.photo=None;self.last_image=0;self.last_scene=None
        try:
            message=self.messages.get_nowait();self.footer.config(text=message)
            if 'Toplam:' in message:
                parts=message.split('|');self.state.config(text=parts[0].strip())
                self.counter.config(text=parts[1].strip().replace('Aşama: ',''))
                self.detail.config(text='\n'.join(p.strip() for p in parts[2:]))
            elif 'TEST TAMAMLANDI' in message:self.state.config(text='Tanılama tamamlandı')
        except queue.Empty:pass
        if self.directory and (self.directory/'mapping.log').exists():
            with (self.directory/'mapping.log').open(errors='replace') as log:
                log.seek(self.log_position);text=log.read();self.log_position=log.tell()
            values=[int(v) for v in re.findall(r'Odom: quality=(\d+)',text)]
            observed_status=self.directory/'tracking-status-observed.jsonl'
            if observed_status.exists():
                with observed_status.open() as stream:
                    stream.seek(self.status_position);lines=stream.readlines()
                    # Do not consume an incomplete last JSON line.
                    complete=[line for line in lines if line.endswith('\n')]
                    self.status_position+=sum(len(line.encode()) for line in complete)
                values=[0 if json.loads(line)['lost'] else 1 for line in complete]
            self.tracking+=sum(v>0 for v in values);self.lost+=sum(v==0 for v in values)
            if values:self.last_quality=values[-1];self.last_odom_time=time.monotonic()
            state='Takip bekleniyor' if self.last_quality is None else 'Takip var' if self.last_quality>0 else 'Takip kayıp: hareketi durdur.'
            capture_only=(self.directory/'mapping-fallback.json').exists()
            if self.process is None:state='Tamamlanan oturum'
            elif time.monotonic()-self.last_odom_time>3:state='Yeni takip sonucu bekleniyor'
            if self.process is not None and capture_only:state='Harita durdu · anlık 3B ve kayıt sürüyor' if self.depth_engine=='sgbm' else 'Harita durdu · görüntü kaydı sürüyor'
            point_status=self.directory/'mapping-status.json'
            points=json.loads(point_status.read_text()).get('cloudPoints',0) if point_status.exists() else 0
            depth_path=self.directory/'depth/status.json';depth_text=''
            if depth_path.exists():
                depth=json.loads(depth_path.read_text());n=depth.get('counts',{}).get('publishedDepth',0);timing=depth.get('computeMs')
                cloud=depth.get('counts',{}).get('localCloudLatestPoints',0)
                depth_text=f'\nSGBM: {n} derinlik karesi · anlık 3B: {cloud:,} nokta'+(f" · {timing['median']:.0f} ms" if timing else '')
                skipped=sum(depth.get('counts',{}).get(k,0) for k in ('supersededPairs','staleBefore','staleAfter'))
                if skipped:depth_text+=f'\nGüncellik için atlanan: {skipped}'
            localization_text=''
            if self.localize_map_db is not None and self.directory:
                match_log=self.directory/'reference-matches.jsonl'
                matches=sum(1 for _ in match_log.open()) if match_log.exists() else 0
                localization_text=f'\nEski harita eşleşmesi: {matches}'
            self.health.config(text=f'{state}\nTakip: {self.tracking} · Kayıp: {self.lost}\nROS haritası: {points:,} nokta'+localization_text+depth_text)
            if 'process has died' in text and not capture_only:self.state.config(text='Haritalama hatası');self.stop()
        if self.process is not None:
            if self.mode=='preview' and not self.stopping_at:
                status=None
                if self.directory and (self.directory/'live-status.json').exists():
                    status=json.loads((self.directory/'live-status.json').read_text())
                if status and status.get('stage')=='mapping':
                    self.state.config(text='Kamera önizlemesi · harita yok',fg='#6ce4b2')
                    self.counter.config(text=f"{max(0,int(status.get('remainingSeconds',0)))} sn kaldı")
                    scene=self.last_scene
                    if scene:
                        light=f"Panel görüntüsü: medyan {scene['medianGray']:.0f}/255 · <50: %{scene['pixelsBelow50Percent']:.0f}"
                        if scene['lowLight']:light+='\nKaranlık sahne · ışığı aç veya kadrajı değiştir'
                    else:light='Panel görüntüsü bekleniyor'
                    self.health.config(text=f"Yayınlanan: {status.get('publishedPairs',0)} stereo çift\n"+light)
            if self.mode!='preview' and not self.diagnostic and not self.stopping_at:
                status=None
                if self.directory and (self.directory/'live-status.json').exists():
                    status=json.loads((self.directory/'live-status.json').read_text())
                provisional=bool(self.directory and (self.directory/'mapping-diagnostic.json').exists())
                failed=bool(self.directory and (self.directory/'hybrid-tracking-failure.json').exists()) and not provisional
                capture_only=bool(self.directory and (self.directory/'mapping-fallback.json').exists())
                title,counter,detail=mapping_guidance(status,time.monotonic(),tracking_failed=failed,
                                                       capture_continues=capture_only,duration_seconds=self.capture_seconds)
                if self.mode=='replay' and not status:
                    title,counter,detail='ROS hazırlanıyor','Bekleniyor…','Telefon kullanılmıyor. Kayıtlı hareket birazdan işlenecek.'
                if self.localize_map_db is not None:
                    if status and status.get('stage')=='finished':
                        title,counter,detail='Konum arama kaydı bitti','Sonuç hazırlanıyor','Telefonu indirebilirsin. Kabul edilmiş eski harita eşleşmeleri kontrol ediliyor; kaynak harita değişmeyecek.'
                    else:
                        detail+='\n\nEski haritada konum ancak kabul edilmiş görsel eşleşmeden sonra doğrulanır. Mor yol bunu izler.'
                self.state.config(text=title,fg='#ff8d8d' if title.startswith('DUR') else '#ffd478' if 'SABİT' in title or capture_only else '#6ce4b2')
                self.counter.config(text=counter);self.detail.config(text=detail)
                if provisional:
                    self.health.config(text='DENEYSEL HARİTA · takip kesintisi kaydedildi\n'+self.health.cget('text'))
                if title!=self.last_guide_title:
                    if self.last_guide_title is not None and self.mode=='live':self.root.bell()
                    self.last_guide_title=title
                if status and status.get('stage')=='mapping':
                    elapsed=status.get('elapsedSeconds',self.capture_seconds-status.get('remainingSeconds',self.capture_seconds))
                    self.progress['value']=min(self.capture_seconds,max(0,elapsed))
                    self.total.config(text=f"Toplam {status['remainingSeconds']} sn kaldı / {self.capture_seconds} sn")
                    if capture_only:self.route.config(text=('Görüntü kaydı ve anlık 3B sürüyor · biriken harita durdu.\n' if self.depth_engine=='sgbm' else
                                                            'Görüntü kaydı sürüyor · biriken harita durdu.\n')+
                                                           'Yavaşça devam edebilir veya Durdur ve kaydet ile bitirebilirsin.')
            if self.stopping_at and time.monotonic()-self.stopping_at>100 and self.process.poll() is None:
                os.killpg(self.process.pid,signal.SIGTERM)
            if self.process.poll() is not None:
                code=self.process.returncode;self.process=None;self.stop_button.config(state='disabled',text='Durdur ve kaydet')
                self.total.config(text='Kayıt sonlandı');self.route.config(text='Sonuç hazır olduğunda: Bu haritayı 3B aç\nTest bittiyse sohbetten “test bitti” yazabilirsin.' if self.localize_map_db is None else 'Kaynak harita korunur · konum bulma sonucu aşağıda')
                for button in self.buttons:button.config(state='normal')
                self.counter.config(text='Bitti' if code==0 else 'Başlatılamadı / hata')
                if self.mode=='preview':
                    report_path=self.directory/'capture.json' if self.directory else None
                    report=json.loads(report_path.read_text()) if report_path and report_path.exists() else {}
                    self.state.config(text='Önizleme bitti' if code==0 and not report.get('error') else 'Önizleme hatası',
                                      fg='#6ce4b2' if code==0 and not report.get('error') else '#ff8d8d')
                    preview_note=('Panelde görüntü görüldü.' if self.preview_seen else 'Panelde görüntü doğrulanmadı; bağlantıyı kontrol et.')
                    self.detail.config(text=(f"{report.get('publishedPairs',0)} stereo çift yayınlandı. {preview_note} Görüntü ve sahne uygunsa 90 sn veya 3 dk oda testini başlat. Bu önizlemede harita oluşturulmadı."
                                             if code==0 and not report.get('error') else
                                             str(report.get('error') or 'Kamera bağlantısı tamamlanamadı.')))
                    self.route.config(text='Önizleme tamamlandı · harita testi ayrı düğmeden başlar')
                elif self.directory and (self.directory/'summary.json').exists():
                    result=json.loads((self.directory/'summary.json').read_text())
                    diagnostic_path=self.directory/'diagnostic-export-result.json'
                    diagnostic_export=json.loads(diagnostic_path.read_text()) if diagnostic_path.exists() else None
                    localization=result.get('localization')
                    if localization is not None:
                        matches=localization['acceptedReferenceMatches']
                        active=localization['localized']
                        self.state.config(text='Eski haritada konum bulundu' if active else
                                          'Eşleşme vardı · son konum belirsiz' if matches else 'Eski haritada eşleşme bulunamadı',
                                          fg='#6ce4b2' if active else '#ffd478')
                        self.detail.config(text=f"Konum mesajları: {localization['poseMessages']}\nKabul edilen eski harita eşleşmesi: {matches}\nEşleşmeden sonraki yol: {localization['publishedPathPoses']} poz\n\nKayıtlı harita değişmedi. Eşleşme, santimetre doğruluğu kanıtı değildir.")
                        self.route.config(text='Kayıtlı harita korundu · yeni harita dışa aktarılmadı')
                    else:
                        self.state.config(text='Deneysel harita parçası' if diagnostic_export and diagnostic_export.get('status')=='diagnostic_fragment' else
                                          'Parçalı harita kaydedildi' if result.get('mapState')=='partial' else
                                          'Harita verisi oluştu' if result['accumulatedGraphPresent'] else 'Harita oluşmadı')
                        exported_path=self.directory/'export-result.json'
                        exported=json.loads(exported_path.read_text()) if exported_path.exists() else {}
                        if diagnostic_export and diagnostic_export.get('status')=='diagnostic_fragment':exported=diagnostic_export
                        self.scale_status.config(text=scale_label(result))
                        self.detail.config(text=f"Bağlantılı poz: {result['graph'].get('largestComponentNodes',0)}\nAyrı parça: {result['graph'].get('connectedComponents',0)}\nDışa aktarılan: {exported.get('points',0)} nokta / {exported.get('poses',0)} poz\n"+scale_label(result)+
                                           ('\nTakip kesintili parça; robota uygun kesintisiz harita değil.' if diagnostic_export else ''))
                    if result.get('sessionError') or not result.get('shutdownClean',True):
                        self.state.config(text='Oturum tamamlanamadı')
                        self.footer.config(text=result.get('sessionError') or 'İşlem zorla kapandı; kayıt günlüğünü kontrol edin.')
                    if result.get('captureContinuedAfterTrackingLoss'):
                        self.route.config(text='Harita kabul edilmedi · mevcut kayıt korunuyor.\nYeniden çekimden önce bu kayıt incelenebilir.')
                        guard_error=(result.get('hybridTrackingFailure') or {}).get('error')
                        capture_error=result.get('captureError') or (result.get('sessionError') if result.get('sessionError')!=guard_error else None)
                        if not result.get('captureReportPresent'):capture_error=capture_error or 'Görüntü kaydı raporu oluşmadı.'
                        if not result.get('shutdownClean',True):capture_error=capture_error or 'İşlemler temiz kapanmadı.'
                        if capture_error:
                            self.state.config(text='KAYIT HATASI · takip de kesildi',fg='#ff8d8d')
                            self.counter.config(text='Kayıt tamamlanamadı')
                            self.detail.config(text=f"Takip kaybından sonra görüntü kaydı da tamamlanamadı.\n\nHata: {capture_error}\n\nMevcut dosyalar inceleme için saklandı; tam kayıt olarak değerlendirilmez.")
                        elif self.mode=='replay':
                            self.state.config(text='Oynatım bitti · harita takibi kesildi',fg='#ffd478')
                            self.counter.config(text='Harita kabul edilmedi')
                            self.detail.config(text=f"İzleyiciye {result.get('leftImagesObserved',0)} görüntü ulaştı.\n\nTakip kaybından sonra kaynak kayıt oynatımı sürdü. Kaynak dosyalar korundu. Telefon kullanılmadı.")
                        else:
                            self.state.config(text='Kayıt tamamlandı · harita takibi kesildi' if result.get('recordingComplete') else 'Kısmi kayıt saklandı · takip kesildi',fg='#ffd478')
                            self.counter.config(text='Kayıt saklandı')
                            detail=f"{result.get('recordingPairs',0)} stereo çift saklandı.\n\nTakip kaybından sonra görüntü kaydı devam etti. Bu oturumun haritası kabul edilmedi. Aynı hareketi yeniden yapmadan bu kayıt üzerinde inceleme yapılabilir."
                            if result.get('recoveryPlanAvailable'):
                                plan=json.loads((self.directory/'recovery-plan.json').read_text())
                                counts=', '.join(f"{part['name']}: {part['pairs']} çift" for part in plan['sections'] if part['available'])
                                detail+=f'\n\nAyrı ham kayıt bölümleri hazır: {counts}. Haritalar ayrı başlangıçlarla işlenecek; otomatik birleşmiş sayılmayacak.'
                            self.detail.config(text=detail)
                elif code!=0:
                    self.state.config(text='Oturum başlatılamadı')
                    self.detail.config(text=self.last_message or 'Başlatma günlüğünde ayrıntı bulunamadı.')
        if self.closing and self.process is None:
            self.shutdown.set()
            if self.receiver.is_alive():
                self.root.after(100,self.poll);return
            self.receiver.join()
            self.root.destroy();return
        self.root.after(200,self.poll)

    def close(self):
        self.closing=True;self.stop()
        for viewer in self.viewers:
            if viewer.poll() is None:
                try:os.killpg(viewer.pid,signal.SIGINT)
                except ProcessLookupError:pass


if __name__=='__main__':
    root=tk.Tk();dashboard=Dashboard(root)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:root.after(0,dashboard.close))
    if '--guide' in os.sys.argv:root.after(300,dashboard.show_test_guide)
    root.mainloop()
