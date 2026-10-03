"""Export observed recorded ARCore/AI surfaces and path as a private 3D scene.

Faces connect adjacent supported pixels only. No hole filling, room completion,
new inference, phone connection or cross-session localization is performed.
"""
import argparse
import base64
import json
from pathlib import Path
import shutil
import struct
import time

import numpy as np

from host.arcore_confirmed_map import file_sha256
from host.arcore_detail import COMPLETE_CAPTURE_REASONS, decode_packet, unproject
from host.ros_arcore_saved import load_saved
from host.ros_map_view import read_pcl

ROOT = Path(__file__).resolve().parents[1]


def private_output(output):
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    if output.is_relative_to(ROOT) and output.relative_to(ROOT).parts[0] not in ('work', 'data', 'outputs'):
        raise ValueError('Scene output inside repository must be under ignored work/data/outputs')
    return output


def pixel_faces(indices, depth, absolute_m=.05, relative=.03):
    """Two triangles per pixel quad; reject unsupported/collapsed/jumping faces."""
    if indices.shape != depth.shape or indices.ndim != 2:
        raise ValueError('Matching depth and index grids required')
    a, b = indices[:-1, :-1], indices[1:, :-1]
    c, d = indices[:-1, 1:], indices[1:, 1:]
    za, zb = depth[:-1, :-1], depth[1:, :-1]
    zc, zd = depth[:-1, 1:], depth[1:, 1:]
    faces = np.concatenate([np.stack([a,b,c], -1).reshape(-1,3),
                            np.stack([c,b,d], -1).reshape(-1,3)])
    z = np.concatenate([np.stack([za,zb,zc], -1).reshape(-1,3),
                        np.stack([zc,zb,zd], -1).reshape(-1,3)])
    keep = ((faces >= 0).all(1) & (faces[:,0] != faces[:,1])
            & (faces[:,0] != faces[:,2]) & (faces[:,1] != faces[:,2])
            & np.isfinite(z).all(1) & (z > 0).all(1)
            & (np.ptp(z, axis=1) <= np.maximum(absolute_m, relative*z.min(1))))
    return faces[keep].astype(np.uint32)


def compact_surface(xyz, rgb, faces):
    if not len(faces):
        raise ValueError('No observed supported triangles; surface export unavailable')
    # Deduplicate faces regardless of camera orientation, preserving first winding.
    _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    faces = faces[np.sort(first)]
    triangle = xyz[faces]
    area2 = np.linalg.norm(np.cross(triangle[:,1]-triangle[:,0], triangle[:,2]-triangle[:,0]), axis=1)
    faces = faces[area2 > 1e-10]
    if not len(faces):
        raise ValueError('Only collapsed surface triangles')
    used, inverse = np.unique(faces.ravel(), return_inverse=True)
    return xyz[used], rgb[used], inverse.reshape(-1,3).astype(np.uint32), used


def supported_faces(faces, frame_ids, min_frames):
    """Count a triangle at most once per independent recorded depth frame."""
    if type(min_frames) is not int or min_frames < 1:
        raise ValueError('Positive independent triangle frame support required')
    if frame_ids.shape != (len(faces),) or not np.issubdtype(frame_ids.dtype,np.integer):
        raise ValueError('One integer depth frame ID per triangle required')
    canonical=np.sort(faces,axis=1)
    _,frame_first=np.unique(np.column_stack((canonical,frame_ids)),axis=0,return_index=True)
    frame_first=np.sort(frame_first)
    _,first,counts=np.unique(canonical[frame_first],axis=0,return_index=True,return_counts=True)
    keep=counts>=min_frames
    selected=np.sort(frame_first[first[keep]])
    levels,frequencies=np.unique(counts,return_counts=True)
    metrics=dict(uniqueCandidateTriangles=len(first),supportedCandidateTriangles=int(keep.sum()),
                 unsupportedTriangles=int((~keep).sum()),
                 independentFaceFrameHistogram={str(int(n)):int(c) for n,c in zip(levels,frequencies)})
    return faces[selected],metrics


def write_mesh_ply(path, xyz, rgb, faces):
    vertices = np.empty(len(xyz), dtype=[(n,'<f4') for n in 'xyz'] + [(n,'u1') for n in ('red','green','blue')])
    for n, values in zip('xyz', xyz.T):
        vertices[n] = values
    for n, values in zip(('red','green','blue'), np.rint(np.clip(rgb,0,1)*255).astype('u1').T):
        vertices[n] = values
    polygons = np.empty(len(faces), dtype=[('count','u1'),('vertices','<i4',(3,))])
    polygons['count'], polygons['vertices'] = 3, faces
    header = ('ply\nformat binary_little_endian 1.0\n'
              f'element vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\n'
              'property uchar red\nproperty uchar green\nproperty uchar blue\n'
              f'element face {len(faces)}\nproperty list uchar int vertex_indices\nend_header\n')
    with Path(path).open('xb') as stream:
        stream.write(header.encode('ascii')); stream.write(vertices.tobytes()); stream.write(polygons.tobytes())


def camera_frustum(pose, intrinsics, size, length=.2):
    fx, fy, cx, cy = intrinsics
    width, height = size
    camera = np.array([[0,0,0], *[[(x-cx)*length/fx, (cy-y)*length/fy, -length]
                                  for x,y in ((0,0),(width-1,0),(width-1,height-1),(0,height-1))]])
    q = pose[4:8]
    if not np.isfinite(q).all() or not .99 <= np.linalg.norm(q) <= 1.01:
        raise ValueError('Invalid saved camera quaternion')
    q = q/np.linalg.norm(q)
    cross = 2*np.cross(q[:3], camera)
    world = camera + q[3]*cross + np.cross(q[:3], cross) + pose[1:4]
    edges = np.array([[0,1],[0,2],[0,3],[0,4],[1,2],[2,3],[3,4],[4,1]])
    return world[edges].reshape(-1,3).astype(np.float32)


def srgb_to_linear(rgb):
    rgb = np.asarray(rgb, dtype=np.float32)
    return np.where(rgb <= .04045, rgb/12.92, ((rgb+.055)/1.055)**2.4).astype('<f4')


def glb_scene(xyz, rgb, faces, poses, frustum, *, fusion='observed'):
    """glTF 2.0 binary: observed surface, path and last mapped camera frustum."""
    surface_available=bool(len(faces))
    scene_name='Observed surfaces' if surface_available else 'Supported source points (no observed triangles)'
    if fusion=='nvblox_tsdf':scene_name='Experimental interpolated TSDF surface'
    document = dict(asset=dict(version='2.0', generator='phone-stereo-depth recorded observed surfaces'),
        scene=0, scenes=[dict(nodes=[0])], nodes=[
            dict(name='ARCore estimated map', rotation=[-float(np.sqrt(.5)),0,0,float(np.sqrt(.5))], children=[1,2,3]),
            dict(name=scene_name,mesh=0),dict(name='Recorded camera path',mesh=1),dict(name='Last mapped camera pose',mesh=2)],
        extensionsUsed=['KHR_materials_unlit'], materials=[
            dict(name='Recorded RGB',doubleSided=True,extensions={'KHR_materials_unlit':{}},
                 pbrMetallicRoughness=dict(baseColorFactor=[1,1,1,1],metallicFactor=0,roughnessFactor=1))],
        meshes=[], buffers=[], bufferViews=[], accessors=[],
        extras=dict(metricAccuracyValidated=False, fullRoomCoverageValidated=False,surfaceAvailable=surface_available,
                    relocalizationValidated=False, coordinates='ARCore session in ROS Z-up, root rotates into glTF Y-up',
                    note=('Experimental TSDF surface; independent view support not validated'
                          if fusion=='nvblox_tsdf' else 'Predicted observed surfaces only; gaps remain unknown')))
    binary = bytearray()

    def accessor(array, kind, component_type, *, indices=False, bounds=False):
        binary.extend(b'\0'*((-len(binary))%4))
        offset = len(binary); data = array.tobytes(); binary.extend(data)
        view = len(document['bufferViews'])
        document['bufferViews'].append(dict(buffer=0,byteOffset=offset,byteLength=len(data),target=34963 if indices else 34962))
        info = dict(bufferView=view,componentType=component_type,count=len(array),type=kind)
        if bounds:
            info.update(min=array.min(axis=0).tolist(),max=array.max(axis=0).tolist())
        document['accessors'].append(info)
        return len(document['accessors'])-1

    for name, positions, colors, polygons, mode in (
            (scene_name,xyz,srgb_to_linear(rgb),faces.ravel().astype('<u4') if surface_available else None,4 if surface_available else 0),
            ('Recorded camera path',poses[:,1:4],srgb_to_linear(np.tile([1,.46,.04],(len(poses),1))),None,3),
            ('Last mapped camera pose',frustum,srgb_to_linear(np.tile([.98,.88,.1],(len(frustum),1))),None,1)):
        primitive = dict(attributes=dict(POSITION=accessor(np.asarray(positions,dtype='<f4'),'VEC3',5126,bounds=True),
                                         COLOR_0=accessor(np.asarray(colors,dtype='<f4'),'VEC3',5126)),mode=mode,material=0)
        if polygons is not None:
            primitive['indices'] = accessor(polygons,'SCALAR',5125,indices=True)
        document['meshes'].append(dict(name=name,primitives=[primitive]))
    document['buffers'] = [dict(byteLength=len(binary))]
    encoded = json.dumps(document,separators=(',',':'),ensure_ascii=True).encode('utf-8')
    encoded += b' '*((-len(encoded))%4)
    binary.extend(b'\0'*((-len(binary))%4))
    length = 12+8+len(encoded)+8+len(binary)
    glb = (struct.pack('<4sII',b'glTF',2,length)+struct.pack('<II',len(encoded),0x4E4F534A)+encoded
           +struct.pack('<II',len(binary),0x004E4942)+binary)
    return glb, document


def export(source, model_map, colored_map, output, *, min_face_frames=1):
    source, model_map, colored_map = Path(source),Path(model_map),Path(colored_map)
    output = private_output(output)
    if type(min_face_frames) is not int or min_face_frames < 1:
        raise ValueError('Positive independent triangle frame support required')
    started = time.monotonic()
    inputs = [source, model_map/'result.json', model_map/'frame_metrics.json', model_map/'map_cloud.ply', model_map/'map_poses.txt',
              colored_map/'map_cloud.ply', colored_map/'map_poses.txt', colored_map/'result.json']
    hashes = {str(p):file_sha256(p) for p in inputs}
    manifest = json.loads((model_map/'result.json').read_text())
    colors = json.loads((colored_map/'result.json').read_text())
    if (manifest.get('status') != 'experimental' or not manifest.get('captureComplete')
            or manifest.get('mapTruncated') or manifest.get('sourceSha256Before') != hashes[str(source)]
            or manifest.get('sourceSha256After') != hashes[str(source)]
            or colors.get('sourceSha256Before') != hashes[str(source)]
            or colors.get('sourceSha256After') != hashes[str(source)]
            or colors.get('geometryFileSha256Before') != hashes[str(model_map/'map_cloud.ply')]
            or colors.get('geometryFileSha256After') != hashes[str(model_map/'map_cloud.ply')]):
        raise ValueError('Complete nontruncated matching recorded map/cache provenance required')
    xyz, rgb, poses = load_saved(colored_map)
    _, _, model_poses = load_saved(model_map)
    if not np.array_equal(poses, model_poses):
        raise ValueError('Colored camera path differs from supported model map')
    original, _ = read_pcl(model_map/'map_cloud.ply')
    if not np.array_equal(xyz, original):
        raise ValueError('Colored geometry differs from supported source cloud')
    resolution = manifest['settings']['voxelM']
    targets = {tuple(key):index for index,key in enumerate(np.floor(xyz/resolution).astype(np.int32))}
    if len(targets) != len(xyz):
        raise ValueError('Ambiguous confirmed voxel identity')
    predicted = {r['depthFrame']:r for r in json.loads((model_map/'frame_metrics.json').read_text()) if r['status']=='predicted'}
    faces = []; face_frames = []; cached = {}; index = sequence = last_pose = last_depth = 0
    end = None; last_geometry = None; seen = set(); matched_pixels = valid_pixels = 0
    recorded_poses = []; last_tracked_timestamp = 0
    with source.open() as archive:
        for line in archive:
            if not line.strip():
                continue
            if end is not None:
                raise ValueError('Packet after capture end')
            d = decode_packet(json.loads(line))
            if d['sequence'] <= sequence:
                raise ValueError('Nonincreasing packet sequence')
            sequence = d['sequence']
            if d['kind']=='end':
                end=d['reason'];continue
            if d['timestamp'] < last_pose:
                raise ValueError('Decreasing camera timestamp')
            last_pose=d['timestamp']
            if d['state']=='TRACKING' and d['timestamp']>last_tracked_timestamp:
                recorded_poses.append((len(recorded_poses), *d['position'], *d['rotation'], d['timestamp']/1e9))
                last_tracked_timestamp=d['timestamp']
            if d['kind'] != 'depth':
                continue
            if d['depth_timestamp'] <= last_depth:
                raise ValueError('Nonincreasing raw depth timestamp')
            last_depth = d['depth_timestamp']
            frame=index;index+=1
            if frame not in predicted:
                continue
            cache=model_map/'predictions'/f'{frame:04d}.npz'
            cached[str(cache)]=file_sha256(cache)
            with np.load(cache,allow_pickle=False) as stored:
                depth, valid, k = stored['depthM'],stored['valid'],stored['intrinsics']
                if (int(stored['timestampNs']) != d['timestamp'] or int(stored['depthTimestampNs']) != d['depth_timestamp']
                        or predicted[frame]['timestampNs'] != d['timestamp']):
                    raise ValueError('Cached depth frame identity mismatch')
            factor=manifest['settings']['outputFactor']
            expected_k=d['intrinsics']*factor;expected_k[2:] += .5*(factor-1)
            if (depth.shape != tuple(n*factor for n in d['raw'].shape) or valid.shape != depth.shape
                    or valid.dtype != np.bool_ or k.shape != (4,) or not np.allclose(k,expected_k,atol=1e-6,rtol=0)
                    or not np.isfinite(depth[valid]).all() or np.any((depth[valid]<.5)|(depth[valid]>5))):
                raise ValueError('Cached prediction geometry differs from recorded camera')
            points=unproject(depth*1000,valid,k,d['translation'],d['quaternion'])
            addresses=np.floor(points/resolution).astype(np.int32)
            ids=np.fromiter((targets.get(tuple(a),-1) for a in addresses),dtype=np.int32,count=len(points))
            grid=np.full(depth.shape,-1,np.int32);grid[valid]=ids
            frame_faces=pixel_faces(grid,depth)
            faces.append(frame_faces);face_frames.append(np.full(len(frame_faces),frame,np.int32))
            matched_pixels+=int((ids>=0).sum());valid_pixels+=len(ids);seen.add(frame)
            mapped_pose=np.array((0,*d['position'],*d['rotation'],d['timestamp']/1e9))
            last_geometry=(mapped_pose,k,(depth.shape[1],depth.shape[0]))
    if end not in COMPLETE_CAPTURE_REASONS or seen != set(predicted) or not faces:
        raise ValueError('Incomplete recorded capture or prediction cache')
    if poses.shape!=np.asarray(recorded_poses).shape or not np.allclose(poses,recorded_poses,atol=1e-9,rtol=0):
        raise ValueError('Saved camera path differs from recorded source poses')
    raw_faces=np.concatenate(faces)
    supported,face_support=supported_faces(raw_faces,np.concatenate(face_frames),min_face_frames)
    if len(supported):
        vertices, vertex_rgb, triangles, used=compact_surface(xyz,rgb,supported)
    else:
        vertices,vertex_rgb=xyz[:0],rgb[:0]
        triangles=np.empty((0,3),np.uint32);used=np.empty(0,np.int64)
    frustum=camera_frustum(*last_geometry)
    glb,document=glb_scene(vertices if len(triangles) else xyz,
                           vertex_rgb if len(triangles) else rgb,triangles,poses,frustum)
    all_hashes={**hashes,**cached}
    after={path:file_sha256(path) for path in all_hashes}
    if all_hashes != after:
        raise RuntimeError('Recorded input changed during export')
    output.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(colored_map/'map_cloud.ply',output/'map_cloud.ply')
    shutil.copyfile(colored_map/'map_poses.txt',output/'map_poses.txt')
    write_mesh_ply(output/'observed_surfaces.ply',vertices,vertex_rgb,triangles)
    (output/'observed_room.glb').write_bytes(glb)
    stats = dict(status='recorded_observed_scene',metricAccuracyValidated=False,fullRoomCoverageValidated=False,
        relocalizationValidated=False,captureEndReason=end,inputPoints=len(xyz),outputPoints=len(xyz),meshVertices=len(vertices),
        candidateTriangles=len(raw_faces),meshTriangles=len(triangles),poses=len(poses),predictionFrames=len(seen),
        surfaceAvailable=bool(len(triangles)),
        **face_support,
        pathMatchesRecordedSource=True,lastMappedPoseTimestampS=float(last_geometry[0][-1]),
        lastPathPoseTimestampS=float(poses[-1,-1]),
        supportedPixels=matched_pixels,predictionPixels=valid_pixels,inputFiles=len(all_hashes),
        inputHashesBefore=all_hashes,inputHashesAfter=after,inputsUnchanged=True,
        maxRetainedXYZDifferenceM=float(np.max(np.abs(vertices-xyz[used]))) if len(used) else 0.,
        settings=dict(voxelM=resolution,minIndependentFrames=manifest['settings']['minIndependentCameraFrames'],
                      minIndependentFaceFrames=min_face_frames,
                      edgeJumpMinimumM=.05,edgeJumpRelative=.03,fillUnknownRegions=False),
        cloudBoundsEstimatedM=dict(min=xyz.min(0).tolist(),max=xyz.max(0).tolist()),
        model=manifest['model'],elapsedS=time.monotonic()-started,
        note='Recorded visible scene only; bounding box is not measured room dimensions; no camera started')
    template=(Path(__file__).with_name('arcore_scene_viewer.html')).read_text()
    payload=dict(glb=base64.b64encode(glb).decode('ascii'),stats={k:v for k,v in stats.items() if not k.startswith('inputHashes')})
    encoded_payload=json.dumps(payload,separators=(',',':')).replace('<',r'\u003c').replace('>',r'\u003e').replace('&',r'\u0026')
    (output/'viewer.html').write_text(template.replace('__SCENE_PAYLOAD__',encoded_payload))
    final_hashes={path:file_sha256(path) for path in all_hashes}
    if final_hashes != all_hashes:
        (output/'result.json').write_text(json.dumps(dict(status='failed',reason='Recorded input changed during output writing'))+'\n')
        raise RuntimeError('Recorded input changed during output writing')
    stats['inputHashesAfter']=final_hashes
    stats['artifacts']={p.name:dict(bytes=p.stat().st_size,sha256=file_sha256(p)) for p in output.iterdir() if p.is_file()}
    (output/'result.json').write_text(json.dumps(stats,indent=2)+'\n')
    return stats


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('input','model-map','colored-map','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--min-face-frames',type=int,default=1,
                        help='Minimum independent depth frames observing each triangle (default: 1; optional 2+)')
    args=parser.parse_args()
    result=export(args.input,args.model_map,args.colored_map,args.output,min_face_frames=args.min_face_frames)
    print(json.dumps({k:v for k,v in result.items() if not k.startswith('inputHashes')},indent=2))


if __name__=='__main__':
    main()
