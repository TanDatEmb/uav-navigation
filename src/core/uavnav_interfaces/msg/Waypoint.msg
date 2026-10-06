# Mission waypoint
# x,y,z = lat_deg,lon_deg,alt_m when frame GPS; NaN yaw_rad = unspecified
uint8 STOP=0
uint8 PASS_THROUGH=1

float64 x
float64 y
float64 z
uint8 behavior
float64 acceptance_radius_m
float64 yaw_rad
