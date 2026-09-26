"""Bounded latest-cloud shared memory. No pickle/socket/file traffic for frames."""
import multiprocessing as mp
import time
import numpy as np


class CloudMailbox:
    def __init__(self,context,capacity=12000):
        if not 1<=capacity<=12000:raise ValueError('Invalid cloud capacity')
        self.capacity=capacity
        self.xyz=context.RawArray('f',capacity*3)
        self.info=context.RawArray('d',5)  # sequence, count, expires, kept, before
        self.lock=context.Lock()
        self.stop=context.Event()

    def publish(self,xyz,expires,kept,before):
        xyz=np.asarray(xyz,dtype=np.float32)
        if xyz.ndim!=2 or xyz.shape[1]!=3 or len(xyz)>self.capacity or not np.isfinite(xyz).all():
            raise ValueError('Invalid bounded cloud')
        if not np.isfinite(expires) or not 0<=len(xyz)<=kept<=before:
            raise ValueError('Invalid cloud metadata')
        if not self.lock.acquire(False):return False
        try:
            np.frombuffer(self.xyz,dtype=np.float32).reshape(-1,3)[:len(xyz)]=xyz
            self.info[0]+=1
            self.info[1:]=[len(xyz),expires,kept,before]
        finally:self.lock.release()
        return True

    def read(self):
        if not self.lock.acquire(False):return None
        try:
            seq,count,expires,kept,before=self.info[:]
            xyz=np.frombuffer(self.xyz,dtype=np.float32).reshape(-1,3)[:int(count)].copy()
        finally:self.lock.release()
        return dict(sequence=int(seq),xyz=xyz,expires=expires,kept=int(kept),before=int(before))

    def clear(self):
        return self.publish(np.empty((0,3),np.float32),0.,0,0)


def renderable(snapshot,now):
    return snapshot is not None and snapshot['sequence']>0 and snapshot['expires']>now


def create_window(mailbox,backend='TkAgg'):
    import matplotlib
    matplotlib.use(backend)
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    fig=plt.figure(figsize=(10,8))
    fig.canvas.manager.set_window_title('Stereo · canlı 3B bulut')
    ax=fig.add_subplot(111,projection='3d');fig.subplots_adjust(bottom=.15,top=.85)
    fig.text(.05,.95,'CANLI TEK KARE · Birikimli harita değil · Birim: dama karesi',fontsize=11)
    fig.text(.05,.91,'Fare: döndür / araç çubuğu: yakınlaştır · Uzak nokta elemesi: 8 özgün piksel',fontsize=9)
    status=fig.text(.05,.05,'Uygun güncel çift bekleniyor…',fontsize=10)
    ax.set_xlabel('X · sağ');ax.set_ylabel('Y · aşağı');ax.set_zlabel('Z · ileri')
    # Fixed viewing volume prevents the apparent scene from breathing every frame.
    ax.set_xlim(-180,180);ax.set_ylim(-180,180);ax.set_zlim(0,360);ax.set_box_aspect((1,1,1))
    cloud=ax.scatter([],[],[],s=2,c=[],cmap='viridis',norm=Normalize(0,360),depthshade=False)
    state={'sequence':-1,'visible':False,'updates':0}
    def tick():
        if mailbox.stop.is_set():plt.close(fig);return False
        packet=mailbox.read()
        # A busy lock cannot refresh age; use only the last accepted expiry.
        if packet is not None:state['packet']=packet
        packet=state.get('packet')
        if not renderable(packet,time.monotonic()):
            if state['visible']:
                cloud._offsets3d=([],[],[]);cloud.set_array(np.empty(0));state['visible']=False
            status.set_text('Güncel bulut yok; bağlantı / uygun çift bekleniyor.')
        elif packet['sequence']!=state['sequence']:
            xyz=packet['xyz'];cloud._offsets3d=(xyz[:,0],xyz[:,1],xyz[:,2]);cloud.set_array(xyz[:,2])
            state['sequence']=packet['sequence'];state['visible']=True;state['updates']+=1
            status.set_text(f"{packet['kept']:,}/{packet['before']:,} nokta korundu · {len(xyz):,} çiziliyor · Güncelleme {state['updates']}")
        fig.canvas.draw_idle()
        return True
    timer=fig.canvas.new_timer(interval=250);timer.add_callback(tick);timer.start()
    fig.canvas.mpl_connect('close_event',lambda event:mailbox.stop.set())
    fig._live_cloud_controls=(timer,state,tick)
    return fig


def run_window(mailbox):
    create_window(mailbox)
    import matplotlib.pyplot as plt
    plt.show()


class LiveCloudWindow:
    def __init__(self):
        context=mp.get_context('spawn')
        self.mailbox=CloudMailbox(context)
        self.process=context.Process(target=run_window,args=(self.mailbox,),daemon=True)
        self.process.start()

    def alive(self):
        return self.process.is_alive() and not self.mailbox.stop.is_set()

    def close(self):
        self.mailbox.stop.set()
