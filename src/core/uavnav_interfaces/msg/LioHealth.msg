# Health status of LIO system
uint8 INITIALIZING=0
uint8 TRACKING=1
uint8 DEGRADED=2
uint8 LOST=3
uint8 RESTARTING=4

builtin_interfaces/Time stamp  # SensorTime of the newest IMU sample evaluated
uint8 state
uint32 epoch
uint16 reason
string reason_name
float64 translation_min_eigenvalue
float64 rotation_min_eigenvalue
float64 correction_age_s
float64 output_error_attitude_rad
float64 output_error_velocity_mps
float64 output_error_position_m
