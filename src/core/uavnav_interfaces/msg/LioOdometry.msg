# LIO odometry at scan timestamp
# frame: lio_odom
builtin_interfaces/Time stamp
uint32 epoch
uint32 reset_counter
geometry_msgs/Pose pose
geometry_msgs/Vector3 velocity
float64[36] pose_covariance
float64[9] velocity_covariance
