std_msgs/Header header

string mission_id
uint32 waypoint_index
uint64 request_id

uint8 ACTIVE=0
uint8 BRAKING=1
uint8 PAUSED=2
uint8 COMPLETE=3
uint8 FAILED=4
uint8 state

uint8 NONE=0
uint8 SAFETY_STOP=1
uint8 OPERATOR_TAKEOVER=2
uint8 ODOMETRY_STALE=3
uint8 TRAJECTORY_INVALID=4
uint8 reason
