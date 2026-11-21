# Contact currently tracked by the leg odometry.
string name
float64 contact_lambda
# Reference pose of the contact in the world.
geometry_msgs/Point world_ref_position
geometry_msgs/Quaternion world_ref_orientation
# Pose of the contact in the IMU frame.
geometry_msgs/Point imu_contact_position
geometry_msgs/Quaternion imu_contact_orientation
