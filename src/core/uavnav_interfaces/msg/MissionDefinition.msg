# Mission definition with waypoints and parameters
uint8 FRAME_GPS=0
uint8 FRAME_LOCAL=1
uint8 YAW_LOCK=0
uint8 YAW_SEGMENT=1

uint64 mission_id
uint8 frame  # FRAME_GPS: Waypoint x,y,z = lat_deg,lon_deg,alt_m, z = altitude above PX4 home [m]; FRAME_LOCAL: metres
uint8 yaw_mode
float64 yaw_rate_rad_s
float64 yaw_rad  # Y1 lock heading; NaN = yaw at mission start
Waypoint[] waypoints
