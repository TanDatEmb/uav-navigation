std_msgs/Header header

uint64 localization_epoch

uint8 WAITING_FOR_SENSORS=0
uint8 INITIALIZING=1
uint8 TRACKING=2
uint8 DEGRADED=3
uint8 LOST=4
uint8 RESETTING=5
uint8 state

bool navigation_valid
bool covariance_valid
bool observability_valid
bool correction_fresh
bool propagation_valid

builtin_interfaces/Time last_correction_stamp
# Conservative source-time barrier for untagged propagated odometry after a
# localization reset. Samples at or before this time cannot be re-admitted.
# Diagnostic-only source freshness observation. PropagatedOdometry carries the
# authoritative localization_epoch and sequence used for admission.
builtin_interfaces/Time last_propagated_state_stamp
uint16 reason_code
