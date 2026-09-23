# PX4-local observation that one exact Core command passed adapter admission.
# It does not decide mission progress or certify future command execution.
std_msgs/Header header
uint64 mode_activation_id
string mission_id
uint64 localization_epoch
uint64 goal_epoch
uint32 waypoint_index
uint64 request_id
uint64 bundle_generation
uint64 sample_id
