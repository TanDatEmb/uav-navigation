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

# Optional explicit no-return endpoints in the same outer frame.  A ray to one
# of these endpoints authorizes miss-only evidence along the ray; the endpoint
# itself is never inserted as occupied.  An entirely empty message means that
# this sensor/update has no explicit visibility evidence and must not be
# interpreted as free space.
# When populated, free_space_endpoints.header.stamp and
# free_space_endpoints.header.frame_id must equal header.stamp and
# header.frame_id respectively.
sensor_msgs/PointCloud2 free_space_endpoints
