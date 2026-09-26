"""Inspect a local RTAB-Map PCL cloud and its exported camera-body poses."""
import argparse
from pathlib import Path
import numpy as np
from host.ros_scale import saved_map_scale,scale_label


def read_pcl(path):
    types={'float':'<f4','double':'<f8','uchar':'u1','uint':'<u4','int':'<i4'}
    with Path(path).open('rb') as stream:
        if stream.readline()!=b'ply\n':raise ValueError('PLY required')
        properties=[];count=None;element=None;size=4;binary=False
        while True:
            raw=stream.readline(1024);size+=len(raw)
            if not raw or size>16384:raise ValueError('Invalid PLY header')
            line=raw.decode('ascii').strip().split()
            if line==['end_header']:break
            if line[:1]==['format']:binary=line[1:]==['binary_little_endian','1.0']
            if line[:1]==['element']:
                if count is None and line[1]!='vertex':raise ValueError('Vertices must be first')
                element=line[1]
                if element=='vertex':count=int(line[2])
            if line[:1]==['property'] and element=='vertex':
                if len(line)!=3 or line[1] not in types:raise ValueError('Unsupported vertex property')
                properties.append((line[2],types[line[1]]))
        if not binary or count is None or not 0<count<=2_000_000:raise ValueError('Unsupported cloud')
        dtype=np.dtype(properties)
        if not {'x','y','z','red','green','blue'}<=set(dtype.names):raise ValueError('Missing XYZ/RGB')
        vertices=np.fromfile(stream,dtype=dtype,count=count)
        if len(vertices)!=count:raise ValueError('Truncated cloud')
    xyz=np.column_stack([vertices[k] for k in ('x','y','z')])
    rgb=np.column_stack([vertices[k] for k in ('red','green','blue')])/255.
    if not np.isfinite(xyz).all():raise ValueError('Nonfinite cloud')
    return xyz,rgb


def make_figure(cloud,poses,*,title='ROS · deneysel 3B harita',color_mode='camera'):
    import matplotlib.pyplot as plt
    xyz,rgb=read_pcl(cloud)
    scale=saved_map_scale(cloud)
    trajectory=np.loadtxt(poses,ndmin=2)
    if trajectory.shape[1]!=9 or not np.isfinite(trajectory).all():raise ValueError('Invalid pose export')
    fig=plt.figure(figsize=(12,8),facecolor='#f4f6f8')
    if fig.canvas.manager:fig.canvas.manager.set_window_title('Stereo · deneysel harita')
    ax=fig.add_subplot(111,projection='3d');fig.subplots_adjust(top=.84,bottom=.12)
    sample=np.arange(len(xyz))
    if len(sample)>40000:sample=np.linspace(0,len(xyz)-1,40000,dtype=int)
    ax.scatter(*xyz[sample].T,c=rgb[sample] if color_mode=='camera' else xyz[sample,2],cmap=None if color_mode=='camera' else 'viridis',s=3,depthshade=False)
    path=trajectory[:,1:4]
    ax.plot(*path.T,color='#ff7800',marker='o',markersize=4,linewidth=2,label='Tahmini kamera yolu')
    ax.scatter(*path[0],c='#25a55f',s=60,label='Başlangıç')
    ax.scatter(*path[-1],c='#db3e40',s=60,label='Bitiş')
    bounds=np.vstack([xyz,path]);low=bounds.min(0);high=bounds.max(0)
    center=(low+high)/2;radius=max(float(np.max(high-low))/2,.01)*1.05
    ax.set_xlim(center[0]-radius,center[0]+radius);ax.set_ylim(center[1]-radius,center[1]+radius)
    ax.set_zlim(center[2]-radius,center[2]+radius);ax.set_box_aspect((1,1,1))
    ax.view_init(elev=25,azim=-120)
    for setter,label in [(ax.set_xlabel,'X'),(ax.set_ylabel,'Y'),(ax.set_zlabel,'Z')]:setter(label+(' · m' if scale['scaleSource']=='measured' else ' · tahmini m'))
    ax.legend(loc='upper left')
    fig.text(.06,.94,title,fontsize=20,weight='bold')
    color_note='Renkler kamera görüntüsünden' if color_mode=='camera' else 'Renkler harita Z koordinatını gösterir'
    fig.text(.06,.895,f'{len(xyz):,} nokta · {len(path)} kayıtlı poz · '+color_note,fontsize=11)
    fig.text(.06,.04,scale_label(scale),fontsize=10)
    return fig


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cloud',type=Path);parser.add_argument('poses',type=Path)
    parser.add_argument('--preview',type=Path)
    parser.add_argument('--title',default='ROS · deneysel 3B harita')
    parser.add_argument('--color-mode',choices=['camera','height'],default='camera')
    args=parser.parse_args()
    import matplotlib
    matplotlib.use('Agg' if args.preview else 'TkAgg')
    fig=make_figure(args.cloud,args.poses,title=args.title,color_mode=args.color_mode)
    if args.preview:
        if args.preview.exists():raise FileExistsError(args.preview)
        fig.savefig(args.preview,dpi=150)
    else:
        import matplotlib.pyplot as plt
        plt.show()


if __name__=='__main__':main()
