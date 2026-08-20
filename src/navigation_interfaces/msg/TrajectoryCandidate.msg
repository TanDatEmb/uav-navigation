# A complete P/V/A candidate within a planned trajectory bundle.
std_msgs/Header header
builtin_interfaces/Time valid_from
uint64 trajectory_id
uint64 parent_trajectory_id
float64 commitment_horizon_s

string mission_id
uint32 waypoint_index
uint64 request_id

bool success
uint8 failure_code
uint8 ROLE_NOMINAL=0
uint8 ROLE_SAFETY=1
uint8 ROLE_COMMITTED=2
uint8 trajectory_role
uint8 SAFETY_KIND_NONE=0
uint8 SAFETY_KIND_ROUTE=1
uint8 SAFETY_KIND_BRAKING_STOP=2
uint8 safety_plan_kind
uint64 world_generation
uint64 world_revision
bool known_free_only
bool current_pose_unknown_overlay

float64 duration_s
float64[] time_from_start
geometry_msgs/Point[] position
geometry_msgs/Vector3[] velocity
geometry_msgs/Vector3[] acceleration
