"""Show an exported map in ROS/RViz without opening the phone cameras."""
import argparse
from pathlib import Path
import signal
import tempfile
import yaml
import numpy as np
from host.ros_map_view import read_pcl
from host.ros_process import ProcessGroup,session_lock
from host.ros_scale import saved_map_scale,scale_label


def map_messages(cloud,poses):
    from sensor_msgs.msg import PointCloud2,PointField
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    xyz,rgb=read_pcl(cloud);trajectory=np.loadtxt(poses,ndmin=2)
    if trajectory.shape[1]!=9 or not np.isfinite(trajectory).all():raise ValueError('Geçersiz kamera yolu')
    norms=np.linalg.norm(trajectory[:,4:8],axis=1)
    if not np.allclose(norms,1,atol=1e-4):raise ValueError('Geçersiz poz dönüşü')
    points=np.empty(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
    for i,key in enumerate(('x','y','z')):points[key]=xyz[:,i]
    colors=np.rint(rgb*255).astype(np.uint32);points['rgb']=(colors[:,0]<<16)|(colors[:,1]<<8)|colors[:,2]
    msg=PointCloud2();msg.header.frame_id='saved_map';msg.height=1;msg.width=len(points)
    msg.fields=[PointField(name=key,offset=i*4,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1)
                for i,key in enumerate(('x','y','z','rgb'))]
    msg.point_step=16;msg.row_step=16*len(points);msg.is_dense=True;msg.is_bigendian=False;msg.data=points.tobytes()
    path=RosPath();path.header.frame_id='saved_map'
    for row in trajectory:
        pose=PoseStamped();pose.header.frame_id='saved_map'
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,row[1:4])
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=map(float,row[4:8])
        path.poses.append(pose)
    return msg,path


def saved_last_transform(path,stamp):
    """Mark the final recorded pose, never a live relocalization estimate."""
    from geometry_msgs.msg import TransformStamped
    if not path.poses:raise ValueError('Kayitli kamera yolu bos')
    last=path.poses[-1].pose
    marker=TransformStamped();marker.header.frame_id='saved_map';marker.child_frame_id='saved_phone_last'
    marker.header.stamp=stamp
    marker.transform.translation.x=last.position.x
    marker.transform.translation.y=last.position.y
    marker.transform.translation.z=last.position.z
    marker.transform.rotation=last.orientation
    return marker


def main():
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('cloud',type=Path);p.add_argument('poses',type=Path)
    p.add_argument('--rviz',action='store_true');args=p.parse_args()
    diagnostic=args.cloud.parent.name=='diagnostic-export'
    root=Path(__file__).resolve().parents[1];cloud,path=map_messages(args.cloud,args.poses)
    with session_lock(root/'work/ros-saved-map.lock'),tempfile.TemporaryDirectory(prefix='phone-map-view-') as temp:
        rclpy.init();node=rclpy.create_node('phone_saved_map');viewer=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pc=node.create_publisher(type(cloud),'/phone/saved_map',qos);pp=node.create_publisher(type(path),'/phone/saved_path',qos)
        try:
            stamp=node.get_clock().now().to_msg();cloud.header.stamp=stamp;path.header.stamp=stamp
            from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
            from geometry_msgs.msg import TransformStamped
            origin=TransformStamped();origin.header.frame_id='saved_map_origin';origin.child_frame_id='saved_map'
            origin.header.stamp=stamp;origin.transform.rotation.w=1.
            broadcaster=StaticTransformBroadcaster(node);broadcaster.sendTransform([origin,saved_last_transform(path,stamp)])
            pc.publish(cloud);pp.publish(path)
            print(('DENEYSEL takip kesintili parça: ' if diagnostic else 'ROS haritası: ')+
                  f'{cloud.width} nokta, {len(path.poses)} poz. '+scale_label(saved_map_scale(args.cloud)),flush=True)
            if args.rviz:
                config=yaml.safe_load((root/'configs/phone-map.rviz').read_text())
                manager=config['Visualization Manager'];manager['Global Options']['Fixed Frame']='saved_map'
                for display in manager['Displays']:
                    if display.get('Name')=='Canli 3B harita':display['Enabled']=False
                    if diagnostic and display.get('Name')=='Kayitli 3B harita':
                        display['Name']='DENEYSEL parca - robot icin gecersiz'
                xyz,_=read_pcl(args.cloud);low=xyz.min(0);high=xyz.max(0);center=(low+high)/2
                manager['Views']['Current']['Distance']=max(1.,float(np.linalg.norm(high-low)*1.2))
                manager['Views']['Current']['Focal Point']=dict(zip(('X','Y','Z'),map(float,center)))
                config_path=Path(temp)/('DENEYSEL-harita-parcasi.rviz' if diagnostic else 'saved-map.rviz')
                config_path.write_text(yaml.safe_dump(config,sort_keys=False))
                viewer=ProcessGroup(['rviz2','-d',str(config_path)])
            while rclpy.ok() and (viewer is None or viewer.process.poll() is None):rclpy.spin_once(node,timeout_sec=.2)
        except KeyboardInterrupt:pass
        finally:
            if viewer:viewer.stop()
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
