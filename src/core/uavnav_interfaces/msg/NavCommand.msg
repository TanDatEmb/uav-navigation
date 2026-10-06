# Navigation command: position, velocity, acceleration + yaw
# frame: lio_odom
uint8 MAIN=0
uint8 BACKUP=1
uint8 BRAKE=2
uint8 HOLD=3

builtin_interfaces/Time stamp  # RosTime
int64 sample_time_ns  # RosTime; the time the command sample refers to (ns)
uint32 lio_epoch
uint64 bundle_id
uint8 phase  # MAIN/BACKUP/BRAKE/HOLD
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float64 yaw_rad
float64 yaw_rate_rad_s
int64 lease_ns  # validity duration of this command (ns)
