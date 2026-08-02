#include "px4_odometry_bridge/reference_point_converter.hpp"

namespace px4_odometry_bridge {

ConvertedOdometry ReferencePointConverter::convert(const ConvertedOdometry &source) const {
  ConvertedOdometry output = source;
  const Eigen::Matrix3d world_from_base = source.orientation.toRotationMatrix();
  const Eigen::Matrix3d base_from_source = config_.base_from_source.linear();
  const Eigen::Vector3d r_base_source = config_.base_from_source.translation();
  const Eigen::Vector3d source_velocity_world = world_from_base * source.velocity_body;
  const Eigen::Vector3d omega_base = source.angular_velocity_body;
  const Eigen::Vector3d output_origin_velocity_world =
      source_velocity_world - world_from_base * omega_base.cross(r_base_source);
  output.position = source.position - world_from_base * r_base_source;
  output.velocity_body = base_from_source *
                        (world_from_base.transpose() * output_origin_velocity_world);
  output.orientation = Eigen::Quaterniond(world_from_base * base_from_source.transpose()).normalized();
  output.angular_velocity_body = base_from_source * omega_base;
  return output;
}

}  // namespace px4_odometry_bridge
