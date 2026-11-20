# Kinematics of one contact in the IMU frame, measured by the robot (forward kinematics) and its contact detection.
string name
# True when the contact is set with the environment.
bool active
# Weight of the contact in the leg odometry (for example the measured normal force).
float64 contact_lambda
geometry_msgs/Point imu_contact_position
geometry_msgs/Vector3 imu_contact_linear_velocity
# Not used by the estimator.
geometry_msgs/Vector3 imu_contact_angular_velocity
geometry_msgs/Quaternion imu_contact_orientation
# Load of the contact along its surface normal (N); not used by the estimator.
float64 normal_force
