"""Stereo odometry + precomputed RGB-D surfaces."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    run=os.environ['PHONE_MAP_DIR']
    use_sim=os.environ.get('PHONE_USE_SIM_TIME','true').lower()=='true'
    odometry=Node(package='rtabmap_odom',executable='stereo_odometry',name='stereo_odometry',namespace='rtabmap',output='screen',
        parameters=[dict(use_sim_time=use_sim,frame_id='phone_link',odom_frame_id='odom',publish_tf=True,
                         approx_sync=True,approx_sync_max_interval=.02,topic_queue_size=100,sync_queue_size=100,qos=1,qos_camera_info=1)],
        arguments=['--Odom/ResetCountdown','30'],
        remappings=[('left/image_rect','/phone/left/image_rect'),('right/image_rect','/phone/right/image_rect'),
                    ('left/camera_info','/phone/left/camera_info'),('right/camera_info','/phone/right/camera_info')])
    mapping=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rtabmap_launch'),'launch','rtabmap.launch.py')),
        launch_arguments=dict(stereo='false',depth='true',visual_odometry='false',frame_id='phone_link',
            # Stereo odometry stamps the later exposure, while RGB/depth use
            # left time. Keep exact image synchronization and query stereo TF
            # at image time instead of requiring an equal-stamp odom message.
            use_sim_time=str(use_sim).lower(),approx_sync='false',odom_frame_id='odom',
            # TF carries no covariance; retain explicit RTAB-Map defaults.
            odom_tf_linear_variance='0.001',odom_tf_angular_variance='0.01',
            topic_queue_size='100',sync_queue_size='100',qos='1',
            rgb_topic='/phone/left/image_rect',depth_topic='/phone/depth/image_rect',camera_info_topic='/phone/left/camera_info',
            odom_topic='/rtabmap/odom',database_path=run+'/map.db',rtabmap_args='--Rtabmap/WorkingDirectory '+run+
                (' --Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true' if os.environ.get('PHONE_LOCALIZATION_ONLY')=='1' else ''),
            rtabmap_viz='false',rviz='false').items())
    return LaunchDescription([odometry,mapping])
