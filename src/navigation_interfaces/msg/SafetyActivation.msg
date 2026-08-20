# Request to activate a preloaded safety branch of a trajectory bundle.
std_msgs/Header header
builtin_interfaces/Time activate_at
uint64 bundle_id
uint64 world_generation
uint64 world_revision

uint8 REASON_MAP_INVALIDATION=0
uint8 REASON_PLANNER_TIMEOUT=1
uint8 REASON_STALE_BUNDLE=2
uint8 REASON_OPERATOR_REQUEST=3
uint8 reason
