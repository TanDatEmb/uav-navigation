# Alignment transformation from lio_odom to PX4 local frame
# T maps lio_odom -> PX4 local
uint8 INIT=0
uint8 VALID=1
uint8 FROZEN=2
uint8 INVALID=3

builtin_interfaces/Time stamp  # SensorTime of the LIO sample in the newest pair
uint8 state
float64 x_m
float64 y_m
float64 z_m
float64 yaw_rad
float64 raw_x_m
float64 raw_y_m
float64 raw_z_m
float64 raw_yaw_rad
float64 age_s
float64 residual_position_m
float64 residual_yaw_rad
