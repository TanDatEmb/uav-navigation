# Product-owned trajectory output. The provenance fields identify the exact
# WorldModel snapshot used by the synchronous Planner call.
std_msgs/Header header

bool success
uint8 failure_code
uint64 world_generation
uint64 world_revision

float64 duration_s
float64[] time_from_start
geometry_msgs/Point[] position
geometry_msgs/Vector3[] velocity
geometry_msgs/Vector3[] acceleration
