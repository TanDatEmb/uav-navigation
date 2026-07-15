// lio_px4_alignment.hpp
// Bridge FAST-LIO2 odometry to PX4 external odometry
// Transforms from LIO world (ENU-like) to PX4 world (NED)

#ifndef PX4_MAPPING_LIO_PX4_ALIGNMENT_HPP_
#define PX4_MAPPING_LIO_PX4_ALIGNMENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>

namespace px4_mapping {

/**
 * @brief Bridge node for LIO-to-PX4 odometry alignment
 *
 * Transform conventions:
 *   - LIO: lio_world (ENU-like, gravity-aligned, Z-up)
 *   - PX4: map (NED, X-north, Y-east, Z-down)
 *
 * Transformation:
 *   - Position: [x, -y, -z] (ENU → NED)
 *   - Rotation: 180° roll then negate pitch/yaw
 *   - Velocity: [x, -y, -z] (ENU → NED)
 *
 * Covariance is transformed accordingly.
 */
class LioPx4Alignment : public rclcpp::Node {
 public:
  LioPx4Alignment();

 private:
  void loadParameters();
  void lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Transform LIO pose to PX4 NED frame
  px4_msgs::msg::VehicleOdometry transformToPX4(
      const nav_msgs::msg::Odometry& lio_msg);

  // Static transform: lio_world → map (NED)
  // Computed once from parameters or TF
  void computeStaticTransform();

  // Members
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string lio_topic_;
  std::string px4_topic_;
  std::string lio_frame_id_;
  std::string px4_frame_id_;
  bool use_tf_lookup_;

  // Static transform (if not using TF)
  Eigen::Matrix3d R_lio_px4_;  // Rotation from LIO to PX4
  Eigen::Vector3d t_lio_px4_;  // Translation (typically zero)

  // Covariance transform matrix
  Eigen::Matrix<double, 6, 6> J_pose_transform_;
};

}  // namespace px4_mapping

#endif  // PX4_MAPPING_LIO_PX4_ALIGNMENT_HPP_
