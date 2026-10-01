std_msgs/Header header

# Immutable Core-owned mission transition receipt. The PX4 adapter may use a
# matching COMPLETE receipt for mode completion, but cannot advance the gate.
string mission_id
uint64 route_revision
uint64 localization_epoch
uint64 mode_activation_id
uint32 waypoint_index
uint64 request_id

uint8 GOAL=0
uint8 COMPLETE=1
uint8 event

bool waypoint_accepted
uint32 accepted_waypoint_index
float64 acceptance_position_error_m
float64 acceptance_speed_mps
