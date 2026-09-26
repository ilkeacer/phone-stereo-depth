"""Preserve Android inertial timestamps and device axes in ROS without inventing camera TF."""


def nearest_acceleration(gyro, candidates, max_delta_ns=20_000_000):
    """Return the closest raw accelerometer sample only within a bounded time window."""
    if not candidates:return None
    nearest=min(candidates,key=lambda row:abs(row['timestampNs']-gyro['timestampNs']))
    return nearest if abs(nearest['timestampNs']-gyro['timestampNs'])<=max_delta_ns else None


def gyro_message(gyro,offset_ns,acceleration=None):
    from sensor_msgs.msg import Imu
    stamp=int(gyro['timestampNs'])+int(offset_ns)
    if stamp<0:raise ValueError('Negative ROS inertial timestamp')
    message=Imu()
    message.header.stamp.sec=stamp//1_000_000_000
    message.header.stamp.nanosec=stamp%1_000_000_000
    message.header.frame_id='phone_imu_android'
    message.orientation.w=1.0
    message.orientation_covariance[0]=-1.0
    message.angular_velocity.x=float(gyro['x'])
    message.angular_velocity.y=float(gyro['y'])
    message.angular_velocity.z=float(gyro['z'])
    # No sensor covariance or rigid camera-to-IMU transform has been measured.
    if acceleration is None:
        message.linear_acceleration_covariance[0]=-1.0
    else:
        message.linear_acceleration.x=float(acceleration['x'])
        message.linear_acceleration.y=float(acceleration['y'])
        message.linear_acceleration.z=float(acceleration['z'])
    return message
