std_msgs/Header header

string mission_id
uint32 waypoint_index
uint64 request_id
geometry_msgs/Point target
float64 acceptance_radius_m

# Intermediate mission points normally use PASS_THROUGH. STOP is reserved for
# the final waypoint or an explicit hold request.
uint8 BEHAVIOR_PASS_THROUGH=0
uint8 BEHAVIOR_STOP=1
uint8 behavior
bool has_next_target
geometry_msgs/Point next_target

# Immutable mission route and measured monotonic progress used by mission,
# planner, yaw, and diagnostics. current/next fields remain transport
# compatibility mirrors and must agree with this snapshot.
navigation_contracts/RouteSnapshot route
