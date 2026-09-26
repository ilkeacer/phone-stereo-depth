"""Private single-frame point clouds in rectified left optical coordinates."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
import cv2
from host.depth import scaled_rectification


def disparity_support_mask(z,valid,p2,output_scale,min_disparity=8.):
    """Reject small effective disparity in original calibrated pixel units.

    This limits depth sensitivity to disparity perturbations; it is not a
    calibrated confidence estimate or proof that the retained points are right.
    Effective disparity removes the principal-point offset: abs(f*B/Z).
    """
    z=np.asarray(z);valid=np.asarray(valid,dtype=bool);p2=np.asarray(p2,dtype=float)
    if z.ndim!=2 or valid.shape!=z.shape:raise ValueError('Depth/mask shape mismatch')
    if not np.isfinite(output_scale) or not 0<output_scale<=1:raise ValueError('Invalid output scale')
    if not np.isfinite(min_disparity) or min_disparity<0:raise ValueError('Invalid disparity threshold')
    if p2.shape!=(3,4) or not np.isfinite(p2).all() or p2[0,3]==0 or not np.allclose(p2[1:,3],0):
        raise ValueError('Horizontal nonzero stereo baseline required')
    base=valid&np.isfinite(z)&(z>0)
    effective=np.zeros(z.shape,dtype=np.float64)
    np.divide(abs(p2[0,3])/output_scale,z,out=effective,where=base)
    keep=base&(effective>=min_disparity)
    return keep,dict(type='minimum_effective_disparity',originalPixelThreshold=float(min_disparity),
                     outputScale=float(output_scale),before=int(base.sum()),kept=int(keep.sum()),
                     removed=int(base.sum()-keep.sum()),confidenceCalibrated=False,
                     maximumRetainedZ=(float(abs(p2[0,3])/output_scale/min_disparity) if min_disparity else None))


def points_from_z(z,valid,left,p1):
    """Z is axial depth, not ray length. No filling of rejected pixels."""
    z=np.asarray(z);valid=np.asarray(valid,dtype=bool);left=np.asarray(left)
    p1=np.asarray(p1,dtype=float)
    if z.ndim!=2 or valid.shape!=z.shape or left.shape!=z.shape or left.dtype!=np.uint8:
        raise ValueError('Require matching Z/mask/grayscale shapes')
    if p1.shape!=(3,4) or not np.isfinite(p1).all() or not np.allclose(p1[:,3],0):
        raise ValueError('P1 must describe the rectified left optical origin')
    good=valid&np.isfinite(z)&(z>0)
    yy,xx=np.nonzero(good)
    rays=np.column_stack((xx,yy,np.ones(len(xx))))@np.linalg.inv(p1[:,:3]).T
    if not np.isfinite(rays).all() or np.any(np.abs(rays[:,2])<1e-12):
        raise ValueError('Invalid projection rays')
    xyz=rays/rays[:,2,None]*z[yy,xx,None]
    with np.errstate(over='ignore'):xyz=xyz.astype(np.float32)
    keep=np.isfinite(xyz).all(axis=1)
    grey=left[yy[keep],xx[keep]]
    return xyz[keep],np.repeat(grey[:,None],3,axis=1),np.column_stack((xx[keep],yy[keep]))


def save_cloud(directory,z,valid,left,p1,provenance):
    """Write a new bundle; never replace an existing scene capture."""
    if provenance.get('unit') not in ('checker_square','m'):
        raise ValueError('Explicit calibration length unit required')
    xyz,rgb,pixels=points_from_z(z,valid,left,p1)
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
    data=np.empty(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1')])
    for i,name in enumerate(('x','y','z')):data[name]=xyz[:,i]
    for i,name in enumerate(('red','green','blue')):data[name]=rgb[:,i]
    header=('ply\nformat binary_little_endian 1.0\ncomment unit '+provenance['unit']+
            '\ncomment frame rectified_left_optical X_right Y_down Z_forward\n'+
            f'element vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\n'+
            'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode('ascii')
    path=directory/'cloud.ply'
    with path.open('xb') as f:f.write(header);f.write(data.tobytes())
    report=dict(provenance=provenance,points=len(xyz),inputPixels=int(z.size),
                maskPositiveFinitePixels=int((np.asarray(valid,dtype=bool)&np.isfinite(z)&(z>0)).sum()),
                reconstructionSize=[z.shape[1],z.shape[0]],projectionP1=np.asarray(p1).tolist(),
                coordinateFrame='rectified_left_optical',axes='X right, Y down, Z forward',
                colorSource='same-pair rectified left grayscale replicated to RGB',
                unit=provenance['unit'],metricAccuracyValidated=False,accumulatedMap=False,
                boundsXYZ=[xyz.min(axis=0).tolist(),xyz.max(axis=0).tolist()] if len(xyz) else None,
                plySha256=hashlib.sha256(path.read_bytes()).hexdigest())
    (directory/'metadata.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


def read_cloud(directory):
    """Read this project's bounded binary PLY + metadata bundle, not arbitrary PLY."""
    directory=Path(directory);path=directory/'cloud.ply'
    if path.stat().st_size>30_004_096:raise ValueError('Cloud exceeds viewer limit')
    raw=path.read_bytes();marker=b'end_header\n';end=raw.find(marker,0,4096)
    if end<0:raise ValueError('Missing bounded PLY header')
    header=raw[:end].decode('ascii').splitlines();payload=raw[end+len(marker):]
    expected=['property float x','property float y','property float z',
              'property uchar red','property uchar green','property uchar blue']
    if header[:2]!=['ply','format binary_little_endian 1.0'] or header[-6:]!=expected:
        raise ValueError('Unsupported PLY format')
    vertices=[line for line in header if line.startswith('element vertex ')]
    if len(vertices)!=1:raise ValueError('Missing vertex count')
    count=int(vertices[0].split()[-1])
    if not 0<=count<=2_000_000 or len(payload)!=count*15:raise ValueError('Invalid PLY payload length')
    metadata_path=directory/'metadata.json'
    if metadata_path.stat().st_size>1_000_000:raise ValueError('Metadata exceeds viewer limit')
    meta=json.loads(metadata_path.read_text())
    if meta.get('unit') not in ('checker_square','m') or 'comment unit '+meta['unit'] not in header:
        raise ValueError('Missing or inconsistent length unit')
    if meta.get('coordinateFrame')!='rectified_left_optical' or meta.get('points')!=count:
        raise ValueError('Inconsistent cloud metadata')
    if hashlib.sha256(raw).hexdigest()!=meta.get('plySha256'):raise ValueError('Cloud checksum mismatch')
    rows=np.frombuffer(payload,dtype=[('xyz','<f4',(3,)),('rgb','u1',(3,))])
    xyz=rows['xyz'].copy();rgb=rows['rgb'].copy()
    if not np.isfinite(xyz).all() or np.any(xyz[:,2]<=0):raise ValueError('Invalid cloud coordinates')
    return xyz,rgb,meta


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    for name in ('calibration','depth','left','output'):ap.add_argument('--'+name,type=Path,required=True)
    ap.add_argument('--mask',type=Path,help='Optional explicit uint8 mask PNG; NaN depth is always excluded')
    ap.add_argument('--scale',type=float,choices=(1.,.5,.25),default=1.)
    ap.add_argument('--min-disparity',type=float,default=0.,help='Optional effective disparity floor in original calibrated pixels; not a confidence score')
    args=ap.parse_args();c=np.load(args.calibration)
    if args.scale!=1:c=scaled_rectification(c,args.scale)
    z=np.load(args.depth);left=cv2.imread(str(args.left),0)
    if left is None:raise SystemExit('Missing left grayscale image')
    mask=cv2.imread(str(args.mask),0) if args.mask else np.isfinite(z)&(z>0)
    if mask is None:raise SystemExit('Missing mask')
    if z.shape[::-1]!=tuple(c['size']):raise SystemExit('Depth size differs from scaled calibration')
    meta=dict(unit=str(c['lengthUnit']),source='recorded depth files',depthFile=str(args.depth),
              leftFile=str(args.left),calibrationSha256=hashlib.sha256(args.calibration.read_bytes()).hexdigest(),outputScale=args.scale)
    if args.min_disparity:
        mask,meta['cloudFilter']=disparity_support_mask(z,mask,c['P2'],args.scale,args.min_disparity)
    print(json.dumps(save_cloud(args.output,z,mask,left,c['P1'],meta),indent=2))
