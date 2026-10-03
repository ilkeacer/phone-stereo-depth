"""Optional recorded-map GPU reduction preserving independent-frame voxel means.

No TSDF, surface interpolation, camera, network or model inference. Pixel/pose
unprojection and voxel addresses remain identical to the existing CPU path.
"""
import numpy as np
import time


def confirmed_voxels_gpu(points, frame_ids, *, resolution=.01, min_frames=3, cell_limit=2_000_000):
    prepare_started=time.monotonic()
    points=np.asarray(points)
    frame_ids=np.asarray(frame_ids)
    if (points.dtype != np.float32 or points.ndim != 2 or points.shape[1] != 3
            or not len(points) or not np.isfinite(points).all() or len(points)>5_000_000
            or frame_ids.shape != (len(points),) or not np.issubdtype(frame_ids.dtype,np.integer)
            or (frame_ids<0).any()):
        raise ValueError('Finite float32 XYZ and nonnegative integer frame IDs required; maximum5M points')
    if (not .005 <= resolution <= .1 or type(min_frames) is not int or min_frames<1
            or type(cell_limit) is not int or cell_limit<1):
        raise ValueError('Invalid confirmed map settings')
    # CPU division deliberately preserves existing float32 voxel boundary rounding.
    addresses=np.floor(points/resolution)
    if (addresses<np.iinfo(np.int32).min).any() or (addresses>np.iinfo(np.int32).max).any():
        raise ValueError('Voxel addresses outside int32 range')
    keys=addresses.astype(np.int64)
    lower=keys.min(0);span=keys.max(0)-lower+1
    unique_frames,compact_frames=np.unique(frame_ids,return_inverse=True)
    frame_count=len(unique_frames)
    if int(span[0])*int(span[1])*int(span[2])*frame_count > np.iinfo(np.int64).max:
        raise ValueError('Packed voxel/frame identity overflows int64')
    shifted=keys-lower
    codes=(shifted[:,0]*int(span[1])+shifted[:,1])*int(span[2])+shifted[:,2]
    combined=codes*frame_count+compact_frames
    cpu_prepare_ms=(time.monotonic()-prepare_started)*1000
    import torch
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA required for optional GPU map reduction')
    start=time.monotonic();torch.cuda.synchronize()
    xyz=torch.from_numpy(points).to(device='cuda',dtype=torch.float64)
    packed=torch.from_numpy(combined).cuda()
    frame_cells,inverse,pixels=torch.unique(packed,sorted=True,return_inverse=True,return_counts=True)
    sums=torch.zeros((len(frame_cells),3),device='cuda',dtype=torch.float64)
    sums.index_add_(0,inverse,xyz)
    means=sums/pixels[:,None]
    cell_codes=frame_cells//frame_count
    cells,cell_inverse,support=torch.unique_consecutive(cell_codes,return_inverse=True,return_counts=True)
    if len(cells)>cell_limit:
        raise ValueError('Cell limit exceeded; output refused rather than truncated')
    totals=torch.zeros((len(cells),3),device='cuda',dtype=torch.float64)
    totals.index_add_(0,cell_inverse,means)
    representatives=totals/support[:,None]
    first=torch.cumsum(support,0)-support
    earliest=frame_cells[first]%frame_count
    # Match CPU dictionary insertion order: first frame, then lexicographic voxel.
    order=torch.argsort(earliest,stable=True)
    order=order[support[order]>=min_frames]
    result=representatives[order].float().cpu().numpy()
    counts=support[order].cpu().numpy()
    torch.cuda.synchronize()
    reduction_ms=(time.monotonic()-start)*1000
    stats=dict(inputPoints=len(points),independentFrames=frame_count,perFrameCells=len(frame_cells),
        allCells=len(cells),confirmedCells=len(result),capacityRejections=0,
        cpuAddressAndPackingMs=cpu_prepare_ms,reductionWithTransferMs=reduction_ms,
        fusionWithAddressAndTransferMs=cpu_prepare_ms+reduction_ms,
        device=torch.cuda.get_device_name(0),torch=torch.__version__,
        voxelAddressCalculation='existing CPU float32 division; no altered voxel boundaries',
        fusion='mean of one point per independent frame per voxel; float64 reductions',
        surfaceInterpolation=False)
    return result,counts,stats


def build(source, model_map, output):
    """Re-fuse a validated prediction cache; write a new private map and path."""
    import json
    from pathlib import Path
    import shutil
    from host.arcore_nvblox import load_frames
    from host.arcore_detail import unproject
    from host.arcore_floor import write_cloud
    from host.arcore_scene_export import private_output
    from host.arcore_confirmed_map import file_sha256
    started=time.monotonic();output=private_output(output)
    frames,poses,manifest,hashes,end=load_frames(source,model_map,'cache')
    point_parts=[];id_parts=[]
    for frame in frames:
        points=unproject(frame['depth']*1000,frame['valid'],frame['intrinsics'],frame['translation'],frame['quaternion'])
        point_parts.append(points);id_parts.append(np.full(len(points),frame['index'],np.int64))
        if sum(len(p) for p in point_parts)>5_000_000:
            raise ValueError('GPU batch point limit exceeded')
    cloud,support,stats=confirmed_voxels_gpu(np.concatenate(point_parts),np.concatenate(id_parts),
        resolution=manifest['settings']['voxelM'],min_frames=manifest['settings']['minIndependentCameraFrames'],
        cell_limit=min(manifest['settings'].get('cellLimit',2_000_000),2_000_000))
    if not len(cloud):raise ValueError('No independently supported points')
    after={p:file_sha256(p) for p in hashes}
    if hashes!=after:raise RuntimeError('Recorded input changed during reduction')
    output.mkdir(parents=True,exist_ok=False)
    write_cloud(output/'map_cloud.ply',cloud)
    shutil.copyfile(Path(model_map)/'map_poses.txt',output/'map_poses.txt')
    result=dict(status='recorded_gpu_confirmed_reduction',source='cached_pretrained_depth',
        captureComplete=True,captureEndReason=end,metricAccuracyValidated=False,cameraStarted=False,
        points=len(cloud),poses=len(poses),frames=len(frames),mapTruncated=False,
        inputFiles=len(hashes),inputHashesBefore=hashes,inputHashesAfter=after,inputsUnchanged=True,
        minIndependentCameraFrames=int(support.min()),gpuBatchStats=stats,
        settings=dict(voxelM=manifest['settings']['voxelM'],minIndependentCameraFrames=manifest['settings']['minIndependentCameraFrames'],
                      cellLimit=min(manifest['settings'].get('cellLimit',2_000_000),2_000_000)),
        model=manifest['model'],elapsedS=time.monotonic()-started,
        note='Recorded GPU batch, not live FPS; same independent-frame means; no TSDF or interpolation')
    final={p:file_sha256(p) for p in hashes}
    if hashes!=final:
        (output/'result.json').write_text(json.dumps(dict(status='failed',reason='Input changed'))+'\n')
        raise RuntimeError('Recorded input changed during output writing')
    result['artifacts']={p.name:dict(bytes=p.stat().st_size,sha256=file_sha256(p)) for p in output.iterdir() if p.is_file()}
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    import argparse
    import json
    from pathlib import Path
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('input','model-map','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();result=build(args.input,args.model_map,args.output)
    print(json.dumps({k:v for k,v in result.items() if not k.startswith('inputHashes')},indent=2))


if __name__=='__main__':main()
