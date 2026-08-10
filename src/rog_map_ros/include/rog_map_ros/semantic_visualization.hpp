#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rog_map_core/voxel_visualization.hpp"

namespace uav::nav::rog {

inline constexpr const char* kSemanticMarkerNamespace = "rog_map/semantic";
inline constexpr std::int32_t kSemanticOccupiedMarkerId = 0;
inline constexpr std::int32_t kSemanticClearanceMarkerId = 1;
inline constexpr std::int32_t kSemanticBoundsMarkerId = 2;

struct SemanticPalette {
  static constexpr float occupied_r = 166.0F / 255.0F;
  static constexpr float occupied_g = 181.0F / 255.0F;
  static constexpr float occupied_b = 194.0F / 255.0F;
  static constexpr float clearance_r = 217.0F / 255.0F;
  static constexpr float clearance_g = 139.0F / 255.0F;
  static constexpr float clearance_b = 55.0F / 255.0F;
  static constexpr float bounds_r = 78.0F / 255.0F;
  static constexpr float bounds_g = 181.0F / 255.0F;
  static constexpr float bounds_b = 171.0F / 255.0F;
};

inline void validateCubeScaleRatio(const double ratio) {
  if (!std::isfinite(ratio) || ratio <= 0.0 || ratio > 1.0) {
    throw std::invalid_argument(
        "mapping.visualization.cube_scale_ratio must be finite, > 0 and <= 1");
  }
}

inline builtin_interfaces::msg::Time semanticStamp(const std::int64_t stamp_ns) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  return stamp;
}

inline visualization_msgs::msg::Marker makeSemanticMarker(
    const std::string& frame_id, const std::int64_t stamp_ns,
    const std::int32_t id, const std::int32_t type, const float r,
    const float g, const float b, const float alpha) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = semanticStamp(stamp_ns);
  marker.ns = kSemanticMarkerNamespace;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = alpha;
  return marker;
}

inline visualization_msgs::msg::MarkerArray makeSemanticMarkerArray(
    const std::string& frame_id, const std::int64_t stamp_ns,
    const VisualizationVoxelSet& occupied,
    const VisualizationVoxelSet& full_surface, const MapBounds& bounds,
    const double resolution, const double cube_scale_ratio) {
  validateCubeScaleRatio(cube_scale_ratio);
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    throw std::invalid_argument("semantic visualization resolution must be finite and positive");
  }
  const auto clearance = deriveClearanceSurface(full_surface, occupied);
  const auto occupied_centers = sortedCentersFromVisualizationSet(occupied, resolution);
  const auto clearance_centers = sortedCentersFromVisualizationSet(clearance, resolution);

  visualization_msgs::msg::MarkerArray result;
  auto occupied_marker = makeSemanticMarker(
      frame_id, stamp_ns, kSemanticOccupiedMarkerId,
      visualization_msgs::msg::Marker::CUBE_LIST,
      SemanticPalette::occupied_r, SemanticPalette::occupied_g,
      SemanticPalette::occupied_b, 1.0F);
  auto clearance_marker = makeSemanticMarker(
      frame_id, stamp_ns, kSemanticClearanceMarkerId,
      visualization_msgs::msg::Marker::CUBE_LIST,
      SemanticPalette::clearance_r, SemanticPalette::clearance_g,
      SemanticPalette::clearance_b, 1.0F);
  const float cube_scale = static_cast<float>(resolution * cube_scale_ratio);
  occupied_marker.scale.x = cube_scale;
  occupied_marker.scale.y = cube_scale;
  occupied_marker.scale.z = cube_scale;
  clearance_marker.scale = occupied_marker.scale;
  for (const auto& center : occupied_centers) {
    geometry_msgs::msg::Point point;
    point.x = center.x(); point.y = center.y(); point.z = center.z();
    occupied_marker.points.push_back(point);
  }
  for (const auto& center : clearance_centers) {
    geometry_msgs::msg::Point point;
    point.x = center.x(); point.y = center.y(); point.z = center.z();
    clearance_marker.points.push_back(point);
  }

  auto bounds_marker = makeSemanticMarker(
      frame_id, stamp_ns, kSemanticBoundsMarkerId,
      visualization_msgs::msg::Marker::LINE_LIST,
      SemanticPalette::bounds_r, SemanticPalette::bounds_g,
      SemanticPalette::bounds_b, 0.90F);
  bounds_marker.scale.x = 0.03;
  const auto& min = bounds.min;
  const auto& max = bounds.max;
  const auto point = [](const double x, const double y, const double z) {
    geometry_msgs::msg::Point result;
    result.x = x; result.y = y; result.z = z;
    return result;
  };
  const std::vector<geometry_msgs::msg::Point> corners = {
      point(min.x(), min.y(), min.z()), point(max.x(), min.y(), min.z()),
      point(max.x(), max.y(), min.z()), point(min.x(), max.y(), min.z()),
      point(min.x(), min.y(), max.z()), point(max.x(), min.y(), max.z()),
      point(max.x(), max.y(), max.z()), point(min.x(), max.y(), max.z())};
  constexpr int edges[][2] = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
      {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (const auto& edge : edges) {
    bounds_marker.points.push_back(corners[edge[0]]);
    bounds_marker.points.push_back(corners[edge[1]]);
  }
  result.markers = {std::move(occupied_marker), std::move(clearance_marker),
                    std::move(bounds_marker)};
  return result;
}

inline visualization_msgs::msg::MarkerArray makeSemanticDeleteArray(
    const std::string& frame_id, const std::int64_t stamp_ns) {
  visualization_msgs::msg::MarkerArray result;
  for (const auto id : {kSemanticOccupiedMarkerId, kSemanticClearanceMarkerId,
                        kSemanticBoundsMarkerId}) {
    auto marker = makeSemanticMarker(
        frame_id, stamp_ns, id, visualization_msgs::msg::Marker::CUBE_LIST,
        0.0F, 0.0F, 0.0F, 0.0F);
    marker.action = visualization_msgs::msg::Marker::DELETE;
    result.markers.push_back(std::move(marker));
  }
  return result;
}

}  // namespace uav::nav::rog
