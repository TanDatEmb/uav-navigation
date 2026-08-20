# Atomic nominal/safety handover contract. The runtime publishes both
# candidates produced from the same world snapshot; the PX4 adapter promotes
# the selected candidate at valid_from.
std_msgs/Header header
builtin_interfaces/Time valid_from
uint64 bundle_id
uint64 parent_bundle_id

string mission_id
uint32 waypoint_index
uint64 request_id
uint8 SELECTED_NOMINAL=0
uint8 SELECTED_SAFETY=1
uint8 selected_candidate
uint64 world_generation
uint64 world_revision
bool nominal_available
bool safety_available
navigation_interfaces/TrajectoryCandidate nominal
navigation_interfaces/TrajectoryCandidate safety
