"""Optional GPU TSDF experiment using recorded RGB, depth and ARCore poses.

Requires a separate nvblox_torch environment. No camera, model inference, ROS
migration or upload occurs. TSDF weights are not independent-view validation.
"""
import argparse
import base64
import json
from pathlib import Path
import time

import numpy as np

from host.arcore_ai_map import decode_rgb, edge_safe_mask, texture_samples
from host.arcore_confirmed_map import file_sha256
from host.arcore_detail import COMPLETE_CAPTURE_REASONS, decode_packet
from host.arcore_scene_export import camera_frustum, glb_scene, private_output, write_mesh_ply


def optical_pose(translation, quaternion):
    """ARCore camera-to-world -> ROS world from OpenCV/nvblox optical camera."""
    q = np.asarray(quaternion, dtype=np.float64)
    t = np.asarray(translation, dtype=np.float64)
    if (q.shape != (4,) or t.shape != (3,) or not np.isfinite(q).all()
            or not np.isfinite(t).all() or not .99 <= np.linalg.norm(q) <= 1.01):
        raise ValueError('Invalid recorded pose')
    x, y, z, w = q/np.linalg.norm(q)
    r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    a = np.array([[1,0,0],[0,0,-1],[0,1,0]])
    pose = np.eye(4, dtype=np.float32)
    pose[:3,:3] = a @ r @ np.diag([1,-1,-1])
    pose[:3,3] = a @ t
    return pose


def cached_depth(path, decoded, metric, factor):
    with np.load(path, allow_pickle=False) as cache:
        depth, valid, k = cache['depthM'], cache['valid'], cache['intrinsics']
        if (int(cache['timestampNs']) != decoded['timestamp']
                or int(cache['depthTimestampNs']) != decoded['depth_timestamp']
                or metric['timestampNs'] != decoded['timestamp']
                or metric['depthTimestampNs'] != decoded['depth_timestamp']):
            raise ValueError('Cached depth frame identity mismatch')
    expected = decoded['intrinsics']*factor
    expected[2:] += .5*(factor-1)
    if (depth.shape != tuple(n*factor for n in decoded['raw'].shape)
            or valid.shape != depth.shape or valid.dtype != np.bool_
            or k.shape != (4,) or not np.allclose(k, expected, atol=1e-6, rtol=0)
            or not np.isfinite(depth[valid]).all()
            or np.any((depth[valid] < .5) | (depth[valid] > 5))):
        raise ValueError('Cached prediction geometry differs from recorded camera')
    # Preserve the existing discontinuity rejection even for a modified cache.
    return depth.astype(np.float32), edge_safe_mask(depth, valid), k


def padded_planes(depth, rgb, valid):
    """Meet native color raycaster alignment with masked, zero-depth padding."""
    h,w = depth.shape
    if rgb.shape != (h,w,3) or valid.shape != (h,w):
        raise ValueError('Aligned RGB/depth/mask required')
    bottom,right = (-h)%4,(-w)%4
    depth = np.pad(depth,((0,bottom),(0,right)),constant_values=0)
    rgb = np.pad(rgb,((0,bottom),(0,right),(0,0)),constant_values=0)
    valid = np.pad(valid,((0,bottom),(0,right)),constant_values=False)
    return depth,rgb,valid


def load_frames(source, model_map, depth_source):
    source, model_map = Path(source), Path(model_map)
    if depth_source not in ('cache', 'raw'):
        raise ValueError('Depth source must be cache or raw')
    inputs = [source, model_map/'result.json', model_map/'frame_metrics.json', model_map/'map_poses.txt']
    hashes = {str(p):file_sha256(p) for p in inputs}
    manifest = json.loads(inputs[1].read_text())
    if (manifest.get('status') != 'experimental' or not manifest.get('captureComplete')
            or manifest.get('mapTruncated') or not manifest.get('sourceUnchanged')
            or manifest.get('sourceSha256Before') != hashes[str(source)]
            or manifest.get('sourceSha256After') != hashes[str(source)]):
        raise ValueError('Complete nontruncated matching recorded cache provenance required')
    factor = manifest['settings']['outputFactor']
    if type(factor) is not int or factor not in (1,2,4):
        raise ValueError('Invalid cached output factor')
    metrics = json.loads(inputs[2].read_text())
    if len({r['depthFrame'] for r in metrics}) != len(metrics):
        raise ValueError('Duplicate prediction frame identity')
    predicted = {r['depthFrame']:r for r in metrics if r['status'] == 'predicted'}
    if len(predicted) != manifest['predictedFrames'] or not predicted:
        raise ValueError('Prediction frame count differs from manifest')
    frames, poses, seen = [], [], set()
    sequence = timestamp = depth_stamp = last_tracked = index = 0
    end = None
    with source.open() as stream:
        for line in stream:
            if not line.strip():
                continue
            if end is not None:
                raise ValueError('Packet after capture end')
            packet = json.loads(line); d = decode_packet(packet)
            if d['sequence'] <= sequence:
                raise ValueError('Nonincreasing packet sequence')
            sequence = d['sequence']
            if d['kind'] == 'end':
                end = d['reason']; continue
            if d['timestamp'] < timestamp:
                raise ValueError('Decreasing camera timestamp')
            timestamp = d['timestamp']
            if d['state'] == 'TRACKING' and timestamp > last_tracked:
                poses.append((len(poses), *d['position'], *d['rotation'], timestamp/1e9))
                last_tracked = timestamp
            if d['kind'] != 'depth':
                continue
            if d['depth_timestamp'] <= depth_stamp:
                raise ValueError('Nonincreasing depth timestamp')
            depth_stamp = d['depth_timestamp']; frame = index; index += 1
            if frame not in predicted:
                continue
            path = model_map/'predictions'/f'{frame:04d}.npz'
            hashes[str(path)] = file_sha256(path)
            depth, valid, k = cached_depth(path, d, predicted[frame], factor)
            if depth_source == 'raw':
                depth = d['raw'].astype(np.float32)/1000
                valid = edge_safe_mask(depth, (depth >= .5) & (depth <= 5) & (d['confidence'] >= 128))
                k = d['intrinsics']
            rgb = decode_rgb(packet)
            color = texture_samples(rgb, packet['textureToRgbCornersPx'], (depth.shape[1],depth.shape[0]))
            color = np.ascontiguousarray(color[...,::-1], dtype=np.uint8)
            pose_row = np.array((0, *d['position'], *d['rotation'], timestamp/1e9))
            frames.append(dict(index=frame, depth=np.where(valid,depth,0).astype(np.float32),
                valid=valid, intrinsics=k, rgb=color, pose=optical_pose(d['translation'],d['quaternion']),
                translation=d['translation'],quaternion=d['quaternion'],
                timestamp=timestamp, pose_row=pose_row))
            seen.add(frame)
    saved_poses = np.loadtxt(model_map/'map_poses.txt', ndmin=2)
    poses = np.asarray(poses, dtype=np.float64)
    if (end not in COMPLETE_CAPTURE_REASONS or seen != set(predicted)
            or index != manifest['depthFrames'] or len(poses) != manifest['poses']):
        raise ValueError('Incomplete recorded capture/cache')
    if poses.shape != saved_poses.shape or not np.allclose(poses,saved_poses,atol=1e-9,rtol=0):
        raise ValueError('Cached path differs from recorded source poses')
    if any(file_sha256(p) != sha for p,sha in hashes.items()):
        raise RuntimeError('Recorded input changed during loading')
    return frames, poses, manifest, hashes, end


def build(source, model_map, output, *, depth_source='cache', voxel_m=.01, truncation_vox=2., weighting='distance'):
    output = private_output(output)
    if not .005 <= voxel_m <= .02 or not 1 <= truncation_vox <= 4:
        raise ValueError('Voxel must be 0.5..2cm; truncation 1..4 voxels')
    started = time.monotonic()
    frames, poses, manifest, hashes, end = load_frames(source,model_map,depth_source)
    import torch
    import nvblox_torch
    from nvblox_torch.mapper import Mapper
    from nvblox_torch.mapper_params import MapperParams, ProjectiveIntegratorParams, MeshIntegratorParams
    from nvblox_torch.sensor import Sensor
    if not torch.cuda.is_available():
        raise RuntimeError('nvblox experiment requires a CUDA GPU')
    projective = ProjectiveIntegratorParams()
    projective.projective_integrator_max_integration_distance_m = 5.
    projective.projective_integrator_truncation_distance_vox = float(truncation_vox)
    if weighting not in ('distance','constant'):
        raise ValueError('Weighting must be distance or constant')
    if weighting == 'constant':
        projective.projective_integrator_weighting_mode = 'kConstantWeight'
        projective.projective_integrator_max_weight = 100.
    mesh_params = MeshIntegratorParams()
    mesh_params.mesh_integrator_min_weight = 3.
    params = MapperParams()
    params.set_projective_integrator_params(projective)
    params.set_mesh_integrator_params(mesh_params)
    mapper = Mapper(voxel_sizes_m=float(voxel_m),mapper_parameters=params)
    # Read actual parameters back from the constructed native mapper.
    actual = mapper.params()
    native_projective=actual.get_projective_integrator_params()
    native_mesh=actual.get_mesh_integrator_params()
    read_params=lambda p:{name[4:]:getattr(p._c_params,name)() for name in p._c_params._method_names() if name.startswith('get_')}
    settings = dict(fusion='nvblox_tsdf',voxelM=voxel_m,
        truncationVox=float(actual.get_projective_integrator_params().projective_integrator_truncation_distance_vox),
        integrationMaxM=float(actual.get_projective_integrator_params().projective_integrator_max_integration_distance_m),
        meshMinWeight=float(actual.get_mesh_integrator_params().mesh_integrator_min_weight),
        independentViewSupportValidated=False,depthSource=depth_source,
        confidenceMinimum=128 if depth_source == 'raw' else None,
        edgeJumpMinimumM=.05,edgeJumpRelative=.03,depthM=[.5,5.],
        nativeProjectiveParams=read_params(native_projective),nativeMeshParams=read_params(native_mesh))
    free_before, total = torch.cuda.mem_get_info()
    sampled_memory = []
    timings = []
    for frame in frames:
        begin = time.monotonic()
        fx,fy,cx,cy = map(float,frame['intrinsics'])
        aligned_depth,aligned_rgb,aligned_valid = padded_planes(frame['depth'],frame['rgb'],frame['valid'])
        h,w = aligned_depth.shape
        sensor = Sensor.from_camera(fu=fx,fv=fy,cu=cx,cv=cy,width=w,height=h)
        depth = torch.from_numpy(aligned_depth).cuda()
        rgb = torch.from_numpy(aligned_rgb).cuda()
        pose = torch.from_numpy(frame['pose'])
        mask = torch.from_numpy(aligned_valid.astype(np.uint8)).cuda()
        mapper.add_depth_frame(depth,pose,sensor,mask)
        mapper.add_color_frame(rgb,pose,sensor,mask)
        torch.cuda.synchronize()
        timings.append(dict(frame=frame['index'],timestampNs=frame['timestamp'],
            acceptedPixels=int(frame['valid'].sum()),nativeShape=list(frame['depth'].shape),
            paddedShape=list(aligned_depth.shape),fusionWithTransferMs=(time.monotonic()-begin)*1000))
        free,_ = torch.cuda.mem_get_info();sampled_memory.append(total-free)
    mesh_start = time.monotonic();mapper.update_color_mesh();torch.cuda.synchronize()
    mesh = mapper.get_color_mesh()
    xyz = mesh.vertices().cpu().numpy().astype(np.float32)
    faces = mesh.triangles().cpu().numpy().astype(np.uint32)
    rgb = mesh.vertex_colors().cpu().numpy().astype(np.float32)/255
    if not len(xyz) or not len(faces) or not np.isfinite(xyz).all():
        raise ValueError('No finite TSDF surface produced')
    frustum = camera_frustum(frames[-1]['pose_row'],frames[-1]['intrinsics'],
                             (frames[-1]['depth'].shape[1],frames[-1]['depth'].shape[0]))
    glb,_ = glb_scene(xyz,rgb,faces,poses,frustum,fusion='nvblox_tsdf')
    after = {p:file_sha256(p) for p in hashes}
    if after != hashes:
        raise RuntimeError('Recorded input changed during GPU fusion')
    output.mkdir(parents=True,exist_ok=False)
    write_mesh_ply(output/'observed_surfaces.ply',xyz,rgb,faces)
    write_mesh_ply(output/'map_cloud.ply',xyz,rgb,np.empty((0,3),np.uint32))
    np.savetxt(output/'map_poses.txt',poses,fmt='%.17g')
    (output/'observed_room.glb').write_bytes(glb)
    mapper.save_map(str(output/'map.nvblx'),0)
    result = dict(status='experimental_gpu_tsdf',metricAccuracyValidated=False,
        fullRoomCoverageValidated=False,relocalizationValidated=False,cameraStarted=False,
        captureEndReason=end,predictionFrames=len(frames),poses=len(poses),settings=settings,
        outputPoints=len(xyz),meshVertices=len(xyz),meshTriangles=len(faces),surfaceAvailable=True,
        cloudBoundsEstimatedM=dict(min=xyz.min(0).tolist(),max=xyz.max(0).tolist()),
        model=manifest['model'] if depth_source == 'cache' else dict(model='ARCore raw depth'),
        inputFiles=len(hashes),inputHashesBefore=hashes,inputHashesAfter=after,inputsUnchanged=True,
        runtime=dict(nvblox=nvblox_torch.__version__,upstreamCommit=nvblox_torch.__git_sha__,
                     torch=torch.__version__,cuda=torch.version.cuda,gpu=torch.cuda.get_device_name(0)),
        fusionWithTransferMedianMs=float(np.median([r['fusionWithTransferMs'] for r in timings])),
        fusionWithTransferP95Ms=float(np.percentile([r['fusionWithTransferMs'] for r in timings],95)),
        meshAndExportS=time.monotonic()-mesh_start,elapsedS=time.monotonic()-started,
        sampledGpuUsedMaxMiB=max(sampled_memory)/2**20,gpuUsedBeforeMiB=(total-free_before)/2**20,
        note='Recorded-data GPU fusion only; TSDF may interpolate surfaces; weights do not certify independent views; no physical accuracy or live FPS claim')
    template=Path(__file__).with_name('arcore_scene_viewer.html').read_text()
    payload=dict(glb=base64.b64encode(glb).decode('ascii'),stats={k:v for k,v in result.items() if not k.startswith('inputHashes')})
    encoded=json.dumps(payload,separators=(',',':')).replace('<',r'\u003c').replace('>',r'\u003e').replace('&',r'\u0026')
    (output/'viewer.html').write_text(template.replace('__SCENE_PAYLOAD__',encoded))
    (output/'frame_metrics.json').write_text(json.dumps(timings,indent=2)+'\n')
    final = {p:file_sha256(p) for p in hashes}
    if final != hashes:
        (output/'result.json').write_text(json.dumps(dict(status='failed',reason='Input changed'))+'\n')
        raise RuntimeError('Recorded input changed during output writing')
    result['artifacts']={p.name:dict(bytes=p.stat().st_size,sha256=file_sha256(p)) for p in output.iterdir() if p.is_file()}
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('input','model-map','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--depth-source',choices=('raw','cache'),default='cache')
    parser.add_argument('--voxel-m',type=float,default=.01)
    parser.add_argument('--truncation-vox',type=float,default=2.)
    parser.add_argument('--weighting',choices=('distance','constant'),default='distance')
    args=parser.parse_args()
    result=build(args.input,args.model_map,args.output,depth_source=args.depth_source,
                 voxel_m=args.voxel_m,truncation_vox=args.truncation_vox,weighting=args.weighting)
    print(json.dumps({k:v for k,v in result.items() if not k.startswith('inputHashes')},indent=2))


if __name__=='__main__':
    main()
