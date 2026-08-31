#include "uav_simulation/visibility_cloud.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace uav::simulation {

std::optional<VisibilityCloud> makeVisibilityCloud(
    const gz::msgs::LaserScan& scan, const std::string_view expected_frame,
    const std::size_t maximum_endpoints) {
  constexpr std::size_t kMaximumRayCount = 262144U;
  if (expected_frame.empty() || scan.frame() != expected_frame ||
      maximum_endpoints == 0U ||
      !scan.has_header() || !scan.header().has_stamp() ||
      scan.count() == 0U || scan.vertical_count() == 0U ||
      !std::isfinite(scan.angle_min()) ||
      !std::isfinite(scan.angle_max()) ||
      !std::isfinite(scan.angle_step()) || scan.angle_step() == 0.0 ||
      !std::isfinite(scan.range_min()) || scan.range_min() <= 0.0 ||
      !std::isfinite(scan.range_max()) || scan.range_max() <= 0.0 ||
      scan.range_min() > scan.range_max() ||
      !std::isfinite(scan.vertical_angle_min()) ||
      !std::isfinite(scan.vertical_angle_max()) ||
      !std::isfinite(scan.vertical_angle_step()) ||
      (scan.vertical_count() > 1U && scan.vertical_angle_step() == 0.0)) {
    return std::nullopt;
  }
  const std::size_t horizontal_count = scan.count();
  const std::size_t vertical_count = scan.vertical_count();
  if (vertical_count > kMaximumRayCount / horizontal_count ||
      horizontal_count * vertical_count > kMaximumRayCount ||
      static_cast<std::size_t>(scan.ranges_size()) !=
          horizontal_count * vertical_count) {
    return std::nullopt;
  }

  const auto& stamp = scan.header().stamp();
  if (stamp.sec() < 0 || stamp.nsec() < 0 || stamp.nsec() >= 1'000'000'000 ||
      stamp.sec() > std::numeric_limits<std::int32_t>::max()) {
    return std::nullopt;
  }

  VisibilityCloud result;
  result.frame_id = scan.frame();
  result.stamp_sec = static_cast<std::int32_t>(stamp.sec());
  result.stamp_nanosec = static_cast<std::uint32_t>(stamp.nsec());
  result.source_ray_count = static_cast<std::uint32_t>(
      horizontal_count * vertical_count);
  const double range_max = scan.range_max();
  const double range_epsilon = std::max(1.0e-3, 1.0e-6 * range_max);
  result.endpoints.reserve(static_cast<std::size_t>(scan.ranges_size()));
  for (std::size_t vertical = 0U; vertical < vertical_count; ++vertical) {
    const double elevation = scan.vertical_angle_min() +
        static_cast<double>(vertical) * scan.vertical_angle_step();
    if (!std::isfinite(elevation)) return std::nullopt;
    const double horizontal_cos = std::cos(elevation);
    const double vertical_sin = std::sin(elevation);
    for (std::size_t horizontal = 0U; horizontal < horizontal_count;
         ++horizontal) {
      const double range = scan.ranges(static_cast<int>(
          vertical * horizontal_count + horizontal));
      const bool explicit_no_return =
          (std::isinf(range) && range > 0.0) ||
          (std::isfinite(range) && range >= range_max - range_epsilon);
      if (!explicit_no_return) continue;
      const double azimuth = scan.angle_min() +
          static_cast<double>(horizontal) * scan.angle_step();
      if (!std::isfinite(azimuth)) return std::nullopt;
      const double horizontal_range = range_max * horizontal_cos;
      const VisibilityEndpoint endpoint{
          static_cast<float>(horizontal_range * std::cos(azimuth)),
          static_cast<float>(horizontal_range * std::sin(azimuth)),
          static_cast<float>(range_max * vertical_sin)};
      if (std::isfinite(endpoint.x) && std::isfinite(endpoint.y) &&
          std::isfinite(endpoint.z)) {
        result.endpoints.push_back(endpoint);
      }
    }
  }
  if (result.endpoints.size() > maximum_endpoints) {
    std::vector<VisibilityEndpoint> downsampled;
    downsampled.reserve(maximum_endpoints);
    for (std::size_t index = 0U; index < maximum_endpoints; ++index) {
      const std::size_t source_index =
          index * result.endpoints.size() / maximum_endpoints;
      downsampled.push_back(result.endpoints[source_index]);
    }
    result.endpoints = std::move(downsampled);
  }
  return result;
}

std::optional<VisibilityCloud> makeVisibilityCloud(
    const sensor_msgs::msg::PointCloud2& cloud,
    const OrganizedVisibilityConfig& config,
    const std::string_view expected_frame,
    const std::size_t maximum_endpoints) {
  constexpr std::size_t kMaximumRayCount = 262144U;
  if (expected_frame.empty() || cloud.header.frame_id != expected_frame ||
      maximum_endpoints == 0U || cloud.is_bigendian ||
      cloud.header.stamp.sec < 0 || cloud.header.stamp.nanosec >= 1'000'000'000U ||
      config.horizontal_count == 0U || config.vertical_count == 0U ||
      cloud.width != config.horizontal_count ||
      cloud.height != config.vertical_count ||
      config.vertical_count > kMaximumRayCount / config.horizontal_count ||
      !std::isfinite(config.horizontal_angle_min_rad) ||
      !std::isfinite(config.horizontal_angle_max_rad) ||
      !std::isfinite(config.vertical_angle_min_rad) ||
      !std::isfinite(config.vertical_angle_max_rad) ||
      !std::isfinite(config.range_max_m) || config.range_max_m <= 0.0 ||
      cloud.point_step == 0U ||
      cloud.row_step < cloud.point_step * cloud.width ||
      static_cast<std::uint64_t>(cloud.row_step) * cloud.height > cloud.data.size()) {
    return std::nullopt;
  }
  const auto field_offset = [&cloud](const std::string_view name)
      -> std::optional<std::uint32_t> {
    for (const auto& field : cloud.fields) {
      if (field.name == name &&
          field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
          field.count == 1U && field.offset <= cloud.point_step &&
          sizeof(float) <= cloud.point_step - field.offset) {
        return field.offset;
      }
    }
    return std::nullopt;
  };
  const auto x_offset = field_offset("x");
  const auto y_offset = field_offset("y");
  const auto z_offset = field_offset("z");
  if (!x_offset || !y_offset || !z_offset) return std::nullopt;

  VisibilityCloud result;
  result.frame_id = cloud.header.frame_id;
  result.stamp_sec = cloud.header.stamp.sec;
  result.stamp_nanosec = cloud.header.stamp.nanosec;
  result.source_ray_count = config.horizontal_count * config.vertical_count;
  result.endpoints.reserve(std::min<std::size_t>(result.source_ray_count,
                                                  maximum_endpoints));
  const double horizontal_step = config.horizontal_count > 1U
      ? (config.horizontal_angle_max_rad - config.horizontal_angle_min_rad) /
            static_cast<double>(config.horizontal_count - 1U)
      : 0.0;
  const double vertical_step = config.vertical_count > 1U
      ? (config.vertical_angle_max_rad - config.vertical_angle_min_rad) /
            static_cast<double>(config.vertical_count - 1U)
      : 0.0;
  std::vector<VisibilityEndpoint> all_endpoints;
  all_endpoints.reserve(result.source_ray_count);
  for (std::uint32_t row = 0U; row < cloud.height; ++row) {
    const double elevation = config.vertical_angle_min_rad +
        static_cast<double>(row) * vertical_step;
    const double horizontal_range = config.range_max_m * std::cos(elevation);
    for (std::uint32_t column = 0U; column < cloud.width; ++column) {
      const std::size_t offset = static_cast<std::size_t>(row) * cloud.row_step +
          static_cast<std::size_t>(column) * cloud.point_step;
      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      std::memcpy(&x, cloud.data.data() + offset + *x_offset, sizeof(float));
      std::memcpy(&y, cloud.data.data() + offset + *y_offset, sizeof(float));
      std::memcpy(&z, cloud.data.data() + offset + *z_offset, sizeof(float));
      const bool explicit_no_return =
          !std::isnan(x) && !std::isnan(y) && !std::isnan(z) &&
          (std::isinf(x) || std::isinf(y) || std::isinf(z));
      if (!explicit_no_return) continue;
      const double azimuth = config.horizontal_angle_min_rad +
          static_cast<double>(column) * horizontal_step;
      const VisibilityEndpoint endpoint{
          static_cast<float>(horizontal_range * std::cos(azimuth)),
          static_cast<float>(horizontal_range * std::sin(azimuth)),
          static_cast<float>(config.range_max_m * std::sin(elevation))};
      if (std::isfinite(endpoint.x) && std::isfinite(endpoint.y) &&
          std::isfinite(endpoint.z)) {
        all_endpoints.push_back(endpoint);
      }
    }
  }
  if (all_endpoints.size() <= maximum_endpoints) {
    result.endpoints = std::move(all_endpoints);
  } else {
    for (std::size_t index = 0U; index < maximum_endpoints; ++index) {
      result.endpoints.push_back(
          all_endpoints[index * all_endpoints.size() / maximum_endpoints]);
    }
  }
  return result;
}

}  // namespace uav::simulation
