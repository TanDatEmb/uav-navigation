std_msgs/Header header

string mission_id
uint32 waypoint_index
uint64 request_id

# PX4-owned lifecycle observation. activation_id is monotonic within the
# adapter process; airborne is sampled from armed state and measured height.
# Neither field grants mission acceptance.
uint64 activation_id
bool airborne

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

# Diagnostic-only External Mode state classification. This is a published
# state label; it does not alter mode transitions or PX4 handover behavior.
uint8 TRACK_TRAJECTORY=0
uint8 WAIT_AIRBORNE=1
uint8 WAIT_HEALTH=2
uint8 WAIT_FIRST_COMMAND=3
uint8 MISSION_HOLD=4
uint8 COMPLETED_HOLD=5
uint8 RECOVERY_HOLD=6
uint8 FAILSAFE_HOLD=7
uint8 HANDOVER_HOLD=8
uint8 external_mode_state
string external_mode_reason

# Read-only copy of Core's terminal acceptance receipt on COMPLETE. A rolling
# horizon completion or goal publication is never acceptance evidence.
bool waypoint_accepted
uint32 accepted_waypoint_index
float64 acceptance_position_error_m
float64 acceptance_speed_mps
