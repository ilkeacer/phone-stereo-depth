"""Interactive saved-cloud viewer in a separate process; never merges camera poses."""
import argparse
from pathlib import Path
import numpy as np
from host.pointcloud import read_cloud


def display_subset(xyz,rgb,limit=12000,max_z=None):
    if limit<1:raise ValueError('Positive point limit required')
    if max_z is not None and (not np.isfinite(max_z) or max_z<=0):raise ValueError('Positive Z limit required')
    ids=np.arange(len(xyz))
    if max_z is not None:ids=ids[xyz[:,2]<=max_z]
    eligible=len(ids)
    if eligible>limit:ids=ids[np.linspace(0,eligible-1,limit,dtype=int)]
    return xyz[ids],rgb[ids],eligible


def open_view(directory):
    # Import GUI only in this separate viewer process.
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button,TextBox
    xyz,rgb,meta=read_cloud(directory)
    unit='dama karesi' if meta['unit']=='checker_square' else 'm (doğruluk ölçülmedi)'
    fig=plt.figure(figsize=(11,8));fig.canvas.manager.set_window_title('Stereo · kaydedilmiş 3B bulut')
    ax=fig.add_subplot(111,projection='3d');fig.subplots_adjust(bottom=.20,top=.85)
    fig.text(.05,.94,'KAYDEDİLMİŞ TEK KARE · Canlı veya birikimli harita değildir',fontsize=12)
    fig.text(.05,.90,'Fareyle sürükle: döndür · Araç çubuğu: yakınlaştır/kaydır · Birim: '+unit,fontsize=9)
    status=fig.text(.05,.035,'',fontsize=9)
    box=TextBox(fig.add_axes([.25,.10,.19,.045]),'En uzak Z: ',initial='')
    button=Button(fig.add_axes([.49,.10,.15,.045]),'Tümünü göster')
    def redraw(value=''):
        try:
            max_z=None if not value.strip() else float(value.replace(',','.'))
            points,colors,eligible=display_subset(xyz,rgb,max_z=max_z)
        except ValueError:
            status.set_text('Z sınırına pozitif bir sayı yazın; tüm noktalar için boş bırakın.');fig.canvas.draw_idle();return
        elev,azim=ax.elev,ax.azim
        ax.clear();ax.view_init(elev=elev,azim=azim)
        if len(points):
            # Depth coloring is explicitly a display aid, not measured surface RGB.
            ax.scatter(points[:,0],points[:,1],points[:,2],c=points[:,2],cmap='viridis',s=2,depthshade=False)
            # Bounds from all eligible vertices, not the display sample.
            all_points=xyz if max_z is None else xyz[xyz[:,2]<=max_z]
            low=all_points.min(axis=0);high=all_points.max(axis=0)
            center=(low+high)/2;radius=max(float((high-low).max())/2,1e-3)*1.05
            ax.set_xlim(center[0]-radius,center[0]+radius)
            ax.set_ylim(center[1]-radius,center[1]+radius)
            ax.set_zlim(center[2]-radius,center[2]+radius)
        ax.set_box_aspect((1,1,1))
        ax.set_xlabel('X · kamera sağı');ax.set_ylabel('Y · kamera aşağısı');ax.set_zlabel('Z · kamera ilerisi')
        ax.set_title(f'{len(xyz):,} kayıtlı nokta · {len(points):,} nokta çiziliyor')
        status.set_text(f'Z sınırında {eligible:,}/{len(xyz):,} nokta. Renk: Z (mor yakın, sarı uzak). '
                        'Yalnız görünüm örneklenir; dosya değişmez.')
        fig.canvas.draw_idle()
    box.on_submit(redraw);button.on_clicked(lambda event:box.set_val(''))
    redraw()
    # Keep widget callbacks alive for the lifetime of the figure.
    fig._cloud_controls=(box,button,redraw)
    return fig


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('directory',type=Path)
    args=ap.parse_args();open_view(args.directory)
    import matplotlib.pyplot as plt
    plt.show()
