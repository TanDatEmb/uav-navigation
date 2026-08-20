# Atomic branchable trajectory contract.
std_msgs/Header header
builtin_interfaces/Time valid_from
builtin_interfaces/Time branch_time
builtin_interfaces/Time decision_deadline

uint64 bundle_id
uint64 parent_bundle_id
string mission_id
uint32 waypoint_index
uint64 request_id
uint64 world_generation
uint64 world_revision
uint64 route_id

TrajectorySegment common_prefix
bool nominal_valid
TrajectorySegment nominal_suffix
TrajectorySegment safety_suffix

uint8 SAFETY_ROUTE=0
uint8 SAFETY_STOP=1
uint8 safety_kind

uint8 BRANCH_NOMINAL=0
uint8 BRANCH_SAFETY=1
uint8 selected_branch
uint8 requested_branch
