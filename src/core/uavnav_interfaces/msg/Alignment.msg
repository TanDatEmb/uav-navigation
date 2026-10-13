# Alignment transformation from lio_odom to PX4 local frame
# T maps lio_odom (FRD) -> PX4 local (NED): p_px4 = Rz(yaw_T) * p_lio_frd + (x_T, y_T, z_T)
uint8 INIT=0
uint8 VALID=1
uint8 FROZEN=2
uint8 INVALID=3

builtin_interfaces/Time stamp  # SensorTime of the LIO sample in the newest pair
uint8 state
# The filtered T (VALID, FROZEN); in INIT the running mean of the accumulation, not usable; in INVALID the last T, for logging only
float64 x_m
float64 y_m
float64 z_m
float64 yaw_rad
# The instant T of the pair formed by the call that produced this message; NaN when no pair was formed.
# raw - filtered is NOT a residual: it includes the lever arm (Rz(yaw_raw) - Rz(yaw_filtered)) * p_lio, which grows with |p_lio|
float64 raw_x_m
float64 raw_y_m
float64 raw_z_m
float64 raw_yaw_rad
float64 age_s  # seconds from the last accepted pair to the newest time seen; 0 before the first accepted pair
# Measured at the vehicle (the LIO position of the pair mapped through the reference T), not at the lio_odom origin
float64 residual_position_m  # |p_px4 - T(p_lio)|, 3-D
float64 residual_yaw_rad     # wrap(yaw_instant - yaw_T)
