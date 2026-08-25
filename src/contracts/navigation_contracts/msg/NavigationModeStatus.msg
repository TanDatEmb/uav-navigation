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

# Explicit waypoint acceptance evidence.  A rolling horizon completion or a
# goal publication is not itself waypoint acceptance; these fields are set
# only when MissionController has measured the vehicle inside the acceptance
# radius with a valid velocity sample.
bool waypoint_accepted
uint32 accepted_waypoint_index
float64 acceptance_position_error_m
float64 acceptance_speed_mps
