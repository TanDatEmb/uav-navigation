string mission_id
string frame_id
uint64 route_revision
uint64 request_id
uint32 active_waypoint_index

uint8 BEHAVIOR_PASS_THROUGH=0
uint8 BEHAVIOR_STOP=1
geometry_msgs/Point[] waypoint_positions
string[] waypoint_ids
float64[] waypoint_acceptance_radii_m
uint8[] waypoint_behaviors

bool measured_progress_valid
uint32 measured_segment_index
float64 measured_progress_arc_m
float64 measured_projection_arc_m
float64 measured_lateral_error_m
