# Mission definition with waypoints and parameters
uint8 FRAME_GPS=0
uint8 FRAME_LOCAL=1
uint8 YAW_LOCK=0
uint8 YAW_SEGMENT=1

uint64 mission_id
uint8 frame
uint8 yaw_mode
float64 yaw_rate_rad_s
Waypoint[] waypoints
