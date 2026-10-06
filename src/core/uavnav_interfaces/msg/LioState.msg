# LiDAR-inertial state at sensor time
# frame: lio_odom (pose), base_link (velocity, angular_velocity)
builtin_interfaces/Time stamp
uint32 epoch
uint32 reset_counter
uint8 lio_state  # one of the LioHealth state constants (INITIALIZING..RESTARTING)
geometry_msgs/Pose pose
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 angular_velocity
float64[36] pose_covariance
float64[9] velocity_covariance
geometry_msgs/Vector3 reset_delta_position
geometry_msgs/Vector3 reset_delta_velocity
float64 reset_delta_yaw
