"""Tk presentation for the motion guide; camera work never runs on the UI thread."""
import queue
import time
import tkinter as tk
from PIL import Image,ImageOps,ImageTk
from host.motion_guide import STEPS

BG='#0e1726';CARD='#172638';TEXT='#eef4fa';MUTED='#a4b6c9';GREEN='#63e2b8'


def latest(channel,value):
    try:channel.get_nowait()
    except queue.Empty:pass
    try:channel.put_nowait(value)
    except queue.Full:pass


class GuideView:
    def build(self):
        r=self.root;r.title('Stereo Rehber · Hareket kaydı');r.configure(bg=BG)
        r.geometry('1280x860');r.minsize(1040,760)
        r.protocol('WM_DELETE_WINDOW',self.close)
        r.bind('<space>',lambda event:self.action() if not isinstance(event.widget,tk.Button) else None)
        r.bind('<F11>',lambda _:r.attributes('-fullscreen',not r.attributes('-fullscreen')))
        r.bind('<Escape>',lambda _:r.attributes('-fullscreen',False))
        header=tk.Frame(r,bg=BG);header.pack(fill='x',padx=28,pady=(22,16))
        tk.Label(header,text='STEREO REHBER',fg=TEXT,bg=BG,font=('DejaVu Sans',24,'bold')).pack(side='left')
        tk.Label(header,text='Mevcut kalibrasyon hazır  •  Hareket deneyi',fg=GREEN,bg=BG,font=('DejaVu Sans',12)).pack(side='right')
        body=tk.Frame(r,bg=BG);body.pack(fill='both',expand=True,padx=28)
        body.columnconfigure(0,weight=1,uniform='panels');body.columnconfigure(1,weight=1,uniform='panels');body.rowconfigure(0,weight=1)
        left=tk.Frame(body,bg=CARD,padx=18,pady=16);left.grid(row=0,column=0,sticky='nsew',padx=(0,18))
        tk.Label(left,text='TELEFONUN GÖRDÜĞÜ',fg=TEXT,bg=CARD,font=('DejaVu Sans',14,'bold')).pack(anchor='w')
        camera_hint=tk.Label(left,text='İki görüntüde de aynı sabit nesne görünsün.',fg=MUTED,bg=CARD,font=('DejaVu Sans',11),justify='left',wraplength=440)
        camera_hint.pack(anchor='w',pady=(5,14))
        left.bind('<Configure>',lambda event:camera_hint.config(wraplength=max(220,event.width-40)))
        self.previews=[]
        for label in ('Üst kamera · telefoto','Alt kamera · geniş açı'):
            tk.Label(left,text=label,fg=MUTED,bg=CARD,font=('DejaVu Sans',11)).pack(anchor='w',pady=(5,5))
            preview=tk.Label(left,text='Kameraları bağlayınca görüntü burada görünecek.',font=('DejaVu Sans',10),
                             fg=MUTED,bg='#0a111d',width=1,height=1,wraplength=420)
            preview.pack(fill='both',expand=True,pady=(0,8));self.previews.append(preview)
        self.preview_note=tk.Label(left,text='Canlı ham önizleme',fg=MUTED,bg=CARD,font=('DejaVu Sans',10))
        self.preview_note.pack(anchor='w')
        right=tk.Frame(body,bg=BG);right.grid(row=0,column=1,sticky='nsew')
        self.step_label=tk.Label(right,text='ÖNCE BAĞLANTI',fg=GREEN,bg=BG,font=('DejaVu Sans',12,'bold'));self.step_label.pack(anchor='w')
        self.heading=tk.Label(right,text='Telefonun gördüğünü kontrol edin',fg=TEXT,bg=BG,font=('DejaVu Sans',21,'bold'),wraplength=440,justify='left')
        self.heading.pack(anchor='w',pady=(10,10))
        self.instruction=tk.Label(right,text='Telefonu açıp USB ile bağlayın. Arka kameraları kitaplık veya eşya bulunan sabit bir alana çevirin.',
                                  fg=MUTED,bg=BG,font=('DejaVu Sans',13),wraplength=430,justify='left')
        self.instruction.pack(anchor='w')
        right.bind('<Configure>',lambda event:[w.config(wraplength=max(250,event.width-12)) for w in (self.heading,self.instruction,self.status)])
        self.diagram=tk.Canvas(right,height=105,bg=BG,highlightthickness=0);self.diagram.pack(fill='x',pady=(8,4))
        self.status=tk.Label(right,text='Yeni dama çekimi gerekmiyor.',fg=GREEN,bg=BG,font=('DejaVu Sans',12),wraplength=440,justify='left');self.status.pack(anchor='w',pady=(4,12))
        self.bar=tk.Canvas(right,height=8,bg='#26374a',highlightthickness=0);self.bar.pack(fill='x')
        self.progress_label=tk.Label(right,text='Dört kısa adım · her adımı siz başlatırsınız',fg=MUTED,bg=BG,font=('DejaVu Sans',11));self.progress_label.pack(anchor='w',pady=(8,12))
        self.button=tk.Button(right,text='Kameraları bağla',command=self.action,font=('DejaVu Sans',15,'bold'),bg=GREEN,fg='#082b25',activebackground='#8deccb',relief='flat',pady=13)
        self.button.pack(fill='x')
        self.cancel_button=tk.Button(right,text='Kapat',command=self.close,font=('DejaVu Sans',12),fg=MUTED,bg=BG,activebackground=CARD,relief='flat',pady=9)
        self.cancel_button.pack(fill='x',pady=(4,8))
        self.step_rows=[]
        for i,(_,title,_,_) in enumerate(STEPS):
            row=tk.Label(right,text=f'{i+1}   {title}',anchor='w',fg=MUTED,bg=BG,font=('DejaVu Sans',11),pady=4)
            row.pack(fill='x');self.step_rows.append(row)
        footer=tk.Label(r,text='Görüntüler yalnız bu bilgisayarda saklanır.  •  Boşluk: başlat / duraklat  •  F11: tam ekran',fg=MUTED,bg=BG,font=('DejaVu Sans',10))
        footer.pack(pady=16)
        self.last_preview=0.;self.view_state=None;self.photos=[];self.draw_phone('steady')

    def draw_phone(self,mode):
        c=self.diagram;c.delete('all');x=115
        if mode=='turn':points=[x-22,24,x+28,10,x+28,95,x-22,108]
        else:points=[x-26,12,x+26,12,x+26,108,x-26,108]
        c.create_polygon(points,fill='#234355',outline=GREEN,width=3)
        c.create_oval(x-17,24,x-8,33,fill=GREEN,outline='');c.create_oval(x-17,39,x-8,48,fill=GREEN,outline='')
        if mode in ('slide','return'):
            a,b=(165,245) if mode=='slide' else (245,165)
            c.create_line(a,58,b,58,fill=GREEN,width=4,arrow='last',arrowshape=(14,17,7))
            text='Yavaşça kaydırın' if mode=='slide' else 'İlk konuma dönün'
        elif mode=='turn':
            c.create_arc(165,28,245,88,start=25,extent=270,style='arc',outline=GREEN,width=3);text='Küçük bir dönüş'
        else:
            c.create_line(76,116,156,116,fill=MUTED,width=2);text='Rahatça sabit tutun'
        c.create_text(292,61,text=text,width=115,fill=MUTED,font=('DejaVu Sans',11))
        c.scale('all',0,0,.85,.85)

    def present(self,state):
        self.view_state=state;kind=state['kind'];guide=state.get('guide');index=guide['index'] if guide else 0
        self.status.config(text=state.get('message',''),fg='#ffc77b' if kind in ('error','waiting') else GREEN)
        if kind in ('error','done','cancelled'):
            self.heading.config(text={'error':'Kayıt durdu','done':'Dört adım tamamlandı','cancelled':'Kayıt durduruldu'}[kind])
            self.instruction.config(text='Kaydedilen görüntüler korundu. Sonuçlar henüz hareket doğruluğu olarak değerlendirilmedi.' if state.get('frames',0) else 'Bağlantıyı kontrol edip tekrar deneyebilirsiniz.')
            self.step_label.config(text='SONUÇ');self.button.config(text='Yeniden bağlan',state='normal');self.cancel_button.config(text='Kapat')
        elif kind=='connecting':
            self.heading.config(text='Kameralar hazırlanıyor');self.button.config(state='disabled');self.cancel_button.config(text='İptal et')
        elif kind=='closing':
            self.heading.config(text='Kameralar kapatılıyor');self.button.config(state='disabled');self.cancel_button.config(state='disabled')
        elif guide:
            step=STEPS[min(index,len(STEPS)-1)]
            self.step_label.config(text=f'ADIM {min(index+1,4)} / 4')
            self.heading.config(text=step[1]);self.instruction.config(text=step[2]);self.draw_phone(step[3])
            self.button.config(text='Duraklat' if guide['active'] else ('Devam et' if guide['elapsed']>0 else 'Hazırım · bu adımı başlat'),
                               state='normal' if kind=='ready' or guide['active'] else 'disabled')
            self.cancel_button.config(text='Kaydı bitir ve kapat',state='normal')
        if guide:
            total=sum(guide['counts']);fraction=guide['elapsed']/guide['duration']
            if guide['finished']:fraction=1.
            self.bar.delete('all');self.bar.create_rectangle(0,0,max(1,self.bar.winfo_width())*fraction,8,fill=GREEN,width=0)
            self.progress_label.config(text=f"{max(0.,guide['duration']-guide['elapsed']):.1f} sn geçerli kayıt kaldı · {total} çift" if guide['active'] else f'{total} çift kaydedildi · hazır olduğunuzda devam edin')
            if guide['finished']:self.progress_label.config(text=f'{total} stereo çift kaydedildi')
            for i,row in enumerate(self.step_rows):
                row.config(text=f"{'✓' if i<index else str(i+1)}   {STEPS[i][1]}"+(f"   · {guide['counts'][i]} çift" if i<index else ''),fg=GREEN if i<=index else MUTED)

    def render_preview(self,packet):
        images,received,delta,expires=packet
        if time.monotonic()>=expires:return
        photos=[]
        for view,im in zip(self.previews,images):
            photo=ImageTk.PhotoImage(ImageOps.contain(im,(max(100,view.winfo_width()-8),max(80,view.winfo_height()-8))))
            view.config(image=photo,text='');photos.append(photo)
        self.photos=photos;self.last_preview=expires
        self.preview_note.config(text=f'Canlı ham önizleme · çift zaman farkı {delta:.1f} ms')

    def poll(self):
        try:self.present(self.messages.get_nowait())
        except queue.Empty:pass
        try:self.render_preview(self.preview_queue.get_nowait())
        except queue.Empty:pass
        if self.last_preview and time.monotonic()>=self.last_preview:
            for view in self.previews:view.config(image='',text='Güncel görüntü bekleniyor…')
            self.photos=[];self.last_preview=0.;self.preview_note.config(text='Önizleme güncel değil')
        if not self.closing:self.root.after(60,self.poll)
