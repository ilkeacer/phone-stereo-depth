from pathlib import Path
import json,shutil,argparse
import numpy as np,cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from host.depth import reproject
ap=argparse.ArgumentParser(description='Export the recorded screen experiment comparison. Run from project root.')
ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
out=args.output;out.mkdir(parents=True,exist_ok=True)
c=np.load('data/calibration/screen_20260910/calibration.npz')
cal=json.loads(Path('data/calibration/screen_20260910/calibration.json').read_text())
sgsource=json.loads(Path('data/depth/screen-sgbm/metrics.json').read_text())
source={'left':sgsource['inputLeft'],'right':sgsource['inputRight'],'calibration':sgsource['calibrationFile'],'criterion':'recorded inputs declared by the SGBM run'}
source['deltaMs']=abs(int(Path(source['left']).stem.split('_')[-1])-int(Path(source['right']).stem.split('_')[-1]))/1e6
rect=[cv2.imread('data/depth/screen-sgbm/rectified-'+name+'.png',0) for name in ('left','right')]
d1=np.load('data/depth/screen-sgbm/disparity_px.npy');m1=cv2.imread('data/depth/screen-sgbm/valid-mask.png',0)>0
z1=np.load('data/depth/screen-sgbm/depth_z_checker_square.npy')
d2=np.load('data/depth/screen-raft/disparity_px.npy');m2=cv2.imread('data/depth/screen-raft/valid-mask.png',0)>0
xyz,m2=reproject(d2,c['Q'],m2);z2=xyz[:,:,2]
np.save('data/depth/screen-raft/depth_z_checker_square.npy',z2)
cv2.imwrite('data/depth/screen-raft/valid-depth-mask.png',m2.astype(np.uint8)*255)
limit=np.percentile(np.concatenate([d1[m1],d2[m2]]),[2,98])
fig,axes=plt.subplots(2,3,figsize=(16,8),constrained_layout=True)
for ax,image,name in zip(axes[0,:2],rect,['Rectified upper sensor (20)','Rectified lower sensor (21)']):
 ax.imshow(image,cmap='gray');ax.set_title(name);ax.axis('off')
axes[0,2].axis('off');axes[0,2].text(.02,.96,'REAL PHONE · HELD-OUT POSE\n\n27 training / 10 held-out poses\nEpipolar error median: 0.49 px\nEpipolar error p95: 1.45 px\nPair timestamp separation: 1.15 ms\n\nMETRE SCALE NOT VERIFIED\nDark areas below = masked matches\nRepetitive checkerboard is challenging\nDense accuracy not measured',va='top',fontsize=12)
for ax,d,m,name in zip(axes[1,:2],[d1,d2],[m1,m2],['SGBM','RAFT-Stereo']):
 artist=ax.imshow(np.ma.masked_where(~m,d),cmap=plt.get_cmap('turbo').with_extremes(bad='#20232b'),vmin=limit[0],vmax=limit[1]);ax.set_facecolor('#20232b');ax.set_title(f'{name} · retained pixels {m.mean():.1%}');ax.axis('off')
fig.colorbar(artist,ax=list(axes[1,:2]),label='Disparity in original image pixels',shrink=.8)
axes[1,2].imshow(np.ma.masked_where(~m2,z2),cmap=plt.get_cmap('viridis').with_extremes(bad='#20232b'),vmin=np.nanpercentile(z2,2),vmax=np.nanpercentile(z2,98));axes[1,2].set_title('RAFT optical Z · checker-square units');axes[1,2].axis('off');axes[1,2].set_facecolor('#20232b')
fig.suptitle('Stereo camera prototype — geometric calibration passed; metric accuracy pending',fontsize=16)
fig.savefig(out/'comparison.png',dpi=130);plt.close(fig)
common=m1&m2
sgbm=json.loads(Path('data/depth/screen-sgbm/metrics.json').read_text());raft=json.loads(Path('data/depth/screen-raft/metrics.json').read_text())
report={'calibration':cal,'sampleSelection':source,'sgbm':sgbm,'raft':raft,'raftPositiveDepthValidRatio':float(m2.mean()),'commonValidPixelRatio':float(common.mean()),'commonPixelMedianDisparityDifferencePx':float(np.median(abs(d1[common]-d2[common]))),'disparityDifferenceIsNotGroundTruthError':True,'metricScaleVerified':False,'denseAccuracyVerified':False}
(out/'results.json').write_text(json.dumps(report,indent=2))
for name in ['calibration.json','calibration.npz']:shutil.copy2(Path('data/calibration/screen_20260910')/name,out/name)
for method in ['sgbm','raft']:
 target=out/method;target.mkdir(exist_ok=True)
 for name in ['disparity_px.npy','depth_z_checker_square.npy','valid-mask.png','metrics.json']:
  shutil.copy2(Path('data/depth/screen-'+method)/name,target/name)
for name in ['rectified-left.png','rectified-right.png']:shutil.copy2(Path('data/depth/screen-sgbm')/name,out/name)
print(json.dumps({k:v for k,v in report.items() if k not in ['calibration','sgbm','raft']},indent=2))
