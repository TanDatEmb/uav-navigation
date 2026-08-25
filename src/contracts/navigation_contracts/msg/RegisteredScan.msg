std_msgs/Header header

# Identity of the public FAST-LIO frame.  A reset starts a new epoch and
# invalidates every observation and command from the previous epoch.
uint64 localization_epoch
# Monotonic within one localization_epoch; it may restart after an epoch reset.
uint64 scan_sequence

# The corrected pose maps base_link into header.frame_id (normally lio_odom).
string body_frame_id
geometry_msgs/PoseWithCovariance corrected_pose

# points.header.stamp and points.header.frame_id must equal header.stamp and
# header.frame_id respectively.  The cloud is registered in the outer frame.
sensor_msgs/PointCloud2 points
