# Atomic geometric LiDAR observation for the navigation world model.
#
# This message is a geometric observation, not a ROG-Map command. It carries
# exactly the deskewed, common-filtered LiDAR scan and the corrected sensor
# pose associated with the same estimator output epoch. It intentionally
# excludes any world-model, occupancy, or planner concept.
#
# Contract (see docs/architecture/navigation_layers.md):
#   header.stamp        == the deskew reference epoch of the scan
#   header.frame_id      == "lio_odom"
#   sensor_pose           == ^lio_odom T_livox at exactly header.stamp,
#                            derived from the same corrected estimator state
#   points.header.stamp   == header.stamp
#   points.header.frame_id == "livox_frame"
#   public_frame_generation == the LioPublicFrameGeneration value active when
#                            this observation was produced
#   observation_stream_id == publisher process identity for
#                           observation_sequence
#   observation_sequence == monotonically increasing publisher identity for
#                           published observations; skipped publications do
#                           not consume a sequence value

std_msgs/Header header

geometry_msgs/Pose sensor_pose

sensor_msgs/PointCloud2 points

uint64 public_frame_generation

uint64 observation_stream_id

uint64 observation_sequence
