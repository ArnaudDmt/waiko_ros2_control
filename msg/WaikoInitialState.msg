# Initial state of WAIKO; the stamp is the time of the first input sample.
std_msgs/Header header
# Linear velocity of the IMU in the world, expressed in the IMU frame.
geometry_msgs/Vector3 x1
# Intermediate estimate of the tilt, usually R^T e_z.
geometry_msgs/Vector3 x2
# Pose of the IMU in the world.
geometry_msgs/Quaternion orientation
geometry_msgs/Point position
