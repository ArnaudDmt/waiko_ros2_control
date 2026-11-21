# One input sample of WAIKO.
std_msgs/Header header
# Accelerometer and gyrometer measurements; the orientation field is not used.
sensor_msgs/Imu imu
# Resets the estimated local linear velocity x1 to the kinematic measurement yv at this sample.
bool reset_local_linear_velocity
waiko_ros2_control/WaikoContact[] contacts
