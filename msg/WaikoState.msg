# State estimated by WAIKO after one input sample.

# Stamp of the input sample; frame_id is the world frame.
std_msgs/Header header

# Number of contacts already set before this sample, which were used for the correction.
uint32 maintained_contacts_count

# Kinematic measurement of the local linear velocity of the IMU, yv = -w x p_anchor - v_anchor (IMU frame).
geometry_msgs/Vector3 computed_yv

# x1: linear velocity of the IMU in the world, expressed in the IMU frame.
geometry_msgs/Vector3 x1
# x2: intermediate estimate of the tilt, R^T e_z.
geometry_msgs/Vector3 x2
# Orientation and position of the IMU in the world.
geometry_msgs/Quaternion orientation
geometry_msgs/Point position
# Origin of the frame in which the local position is estimated: position = position_anchor + R p_local.
geometry_msgs/Point position_anchor
# Gyrometer measurement used for this sample (IMU frame).
geometry_msgs/Vector3 angular_velocity

# Correction terms of this sample, as a local angular velocity and a local linear velocity.
geometry_msgs/Vector3 orientation_correction_from_meas
geometry_msgs/Vector3 position_correction_from_meas

waiko_ros2_control/WaikoContactState[] contacts
