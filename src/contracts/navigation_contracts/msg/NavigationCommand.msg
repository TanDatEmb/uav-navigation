std_msgs/Header header

uint64 localization_epoch
uint64 goal_epoch
# PX4 External Mode activation that this Core command is authorized for.
# A prior activation's still-live command cannot be replayed after reentry.
uint64 mode_activation_id
string mission_id
uint32 waypoint_index
uint64 request_id

uint64 world_generation
uint64 world_revision
builtin_interfaces/Time world_observation_stamp

uint64 bundle_generation
# Optional producer-certified MAIN continuation for the current pass-through
# waypoint. Existing mission/request/goal/bundle fields bind this witness.
bool certified_main_continuation
uint64 continuation_boundary_stamp_ns
uint64 sample_id
# Exact producer execution-state source time used by the final authorization
# decision. It may be newer than the state used by an earlier diagnostic trace;
# the PVA reference itself remains evaluated at header.stamp.
builtin_interfaces/Time state_source_stamp
builtin_interfaces/Time valid_until

uint8 ROLE_MAIN=0
uint8 ROLE_BACKUP=1
uint8 ROLE_EMERGENCY=2
uint8 role

uint8 STATUS_EMPTY=0
uint8 STATUS_READY=1
uint8 STATUS_BRAKING=2
uint8 STATUS_COMPLETED=3
uint8 STATUS_REJECTED=4
uint8 STATUS_HANDOVER=5
uint8 status

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk

float64 yaw
float64 yaw_rate
float64 trajectory_time_s

# These two fields are current velocity-only emergency authorization inputs.
# They remain on the control wire even though the other execution trace fields
# moved to NavigationExecutionDiagnostics.
uint8 EMERGENCY_AUTHORIZATION_NONE=0
uint8 EMERGENCY_AUTHORIZATION_ACTUAL_ANCHOR_CERTIFICATE_EXCEEDED=1
uint8 EMERGENCY_AUTHORIZATION_PROJECTED_MAIN_ONLY_CERTIFICATE_EXCEEDED=2
uint8 EMERGENCY_AUTHORIZATION_OTHER_INVALID=3
# Independent brake attempt, not an actual/projected tracking violation witness.
uint8 EMERGENCY_AUTHORIZATION_INDETERMINATE_PRE_START_TRACKING=4
uint8 emergency_authorization_reason
uint8 emergency_candidate_commit_result
