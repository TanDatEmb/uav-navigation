std_msgs/Header header

uint64 localization_epoch
uint64 goal_epoch
string mission_id
uint32 waypoint_index
uint64 request_id

uint64 world_generation
uint64 world_revision
builtin_interfaces/Time world_observation_stamp

uint64 bundle_generation
uint64 sample_id
builtin_interfaces/Time state_source_stamp
builtin_interfaces/Time valid_until

uint8 ROLE_MAIN=0
uint8 ROLE_BACKUP=1
uint8 ROLE_EMERGENCY=2
uint8 role

uint8 STATUS_EMPTY=0
uint8 STATUS_READY=1
uint8 STATUS_BRAKING=2
uint8 STATUS_COMPLETED=3
uint8 STATUS_REJECTED=4
uint8 STATUS_HANDOVER=5
uint8 status

uint16 reason_code

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk

float64 yaw
float64 yaw_rate
float64 trajectory_time_s
