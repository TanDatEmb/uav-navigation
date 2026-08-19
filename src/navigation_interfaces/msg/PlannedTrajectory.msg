# Product-owned trajectory output. The provenance fields identify the exact
# WorldModel snapshot used by the synchronous Planner call.
std_msgs/Header header

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

float64 duration_s
float64[] time_from_start
geometry_msgs/Point[] position
geometry_msgs/Vector3[] velocity
geometry_msgs/Vector3[] acceleration
