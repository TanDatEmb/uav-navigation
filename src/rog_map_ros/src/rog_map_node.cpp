#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rog_map_core/voxel_occupancy_map.hpp"
#include "rog_map_ros/exact_pairing.hpp"
#include "rog_map_ros/frame_transform.hpp"

namespace uav::nav::rog {
namespace {

using Cloud = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
using Odom = nav_msgs::msg::Odometry::ConstSharedPtr;
using PairCache = ExactPairingCache<Cloud, Odom>;

std::int64_t stampNs(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

const char* stateName(MapValidity state) {
  switch (state) {
    case MapValidity::kWaitingForLio: return "WAITING_FOR_LIO";
    case MapValidity::kActive: return "ACTIVE";
    case MapValidity::kStale: return "STALE";
    case MapValidity::kInvalid: return "INVALID";
  }
  return "UNKNOWN";
}

struct RawPair {
  Cloud cloud;
  Odom odom;
  std::chrono::steady_clock::time_point paired_at;
};

}  // namespace

class RogMapNode final : public rclcpp::Node {
 public:
  explicit RogMapNode(const rclcpp::NodeOptions& options)
      : Node("rog_map", options),
        odom_frame_(declare_parameter("frames.odom", "lio_odom")),
        corrected_child_frame_(declare_parameter("frames.corrected_child", "base_link")),
        lidar_frame_(declare_parameter("frames.lidar", "livox_frame")),
        cloud_topic_(declare_parameter("input.deskewed_points_topic", "/lio/deskewed_points")),
        odom_topic_(declare_parameter("input.corrected_odometry_topic", "/lio/odometry_corrected")),
        diagnostics_topic_(declare_parameter("input.diagnostics_topic", "/lio/diagnostics")),
        unmatched_timeout_ms_(declare_parameter<std::int64_t>(
            "mapping.observation.unmatched_timeout_ms", 500)),
        cache_capacity_(declare_parameter<std::int64_t>(
            "mapping.observation.cache_depth", 16)),
        queue_depth_(declare_parameter<std::int64_t>(
            "mapping.observation.update_queue_depth", 1)),
        stale_timeout_ms_(declare_parameter<std::int64_t>(
            "mapping.lifecycle.stale_timeout_ms", 500)),
        visualization_enabled_(declare_parameter(
            "mapping.visualization.enabled", false)),
        occupied_visualization_topic_(declare_parameter(
            "mapping.visualization.occupied_topic", "/rog_map/occupied_voxels")),
        inflated_visualization_topic_(declare_parameter(
            "mapping.visualization.inflated_topic", "/rog_map/inflated_voxels")),
        map_(loadMapConfig()) {
    if (queue_depth_ != 1 || cache_capacity_ <= 0 || unmatched_timeout_ms_ <= 0 ||
        stale_timeout_ms_ <= 0) {
      throw std::invalid_argument("ROG mapping queues and timeouts must be positive; queue depth is 1");
    }
    cache_ = std::make_unique<PairCache>(static_cast<std::size_t>(cache_capacity_));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
        [this](Cloud message) { onCloud(std::move(message)); });
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS{rclcpp::KeepLast{10}}.reliable().durability_volatile(),
        [this](Odom message) { onOdometry(std::move(message)); });
    diagnostics_subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        diagnostics_topic_, rclcpp::QoS{rclcpp::KeepLast{10}}.reliable(),
        [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
          onDiagnostics(*message);
        });
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/rog_map/diagnostics", rclcpp::QoS{rclcpp::KeepLast{10}}.reliable());
    if (visualization_enabled_) {
      occupied_visualization_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          occupied_visualization_topic_, rclcpp::SensorDataQoS().keep_last(1));
      inflated_visualization_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          inflated_visualization_topic_, rclcpp::SensorDataQoS().keep_last(1));
      visualization_timer_ = create_wall_timer(
          std::chrono::milliseconds(500), [this] { publishVisualization(); });
    }
    cache_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { expireCaches(); });
    worker_ = std::thread([this] { updateLoop(); });
    RCLCPP_INFO(get_logger(), "ROG navigation map input: %s + %s, frame=%s, resolution=%.3f m",
                cloud_topic_.c_str(), odom_topic_.c_str(), odom_frame_.c_str(), map_.resolution());
  }

  ~RogMapNode() override {
    {
      std::lock_guard lock(mutex_);
      stopping_ = true;
    }
    ready_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  VoxelMapConfig loadMapConfig() {
    VoxelMapConfig config;
    config.resolution_m = declare_parameter("mapping.local_map.resolution_m", 0.10);
    const auto size = declare_parameter<std::vector<double>>(
        "mapping.local_map.size_m", {30.0, 30.0, 12.0});
    if (size.size() != 3U) throw std::invalid_argument("mapping.local_map.size_m must have 3 values");
    config.size_m = {size[0], size[1], size[2]};
    config.raycast_min_range_m = declare_parameter("mapping.observation.min_range_m", 0.30);
    config.raycast_max_range_m = declare_parameter("mapping.observation.max_range_m", 15.0);
    config.inflation_radius_m = declare_parameter("mapping.inflation.radius_m", 0.50);
    config.shift_threshold_m = declare_parameter("mapping.local_map.shift_threshold_m", 1.0);
    config.hit_log_odds = declare_parameter("mapping.occupancy.hit_log_odds", 0.85);
    config.miss_log_odds = declare_parameter("mapping.occupancy.miss_log_odds", -0.40);
    config.occupied_threshold = declare_parameter("mapping.occupancy.occupied_threshold", 0.0);
    config.clamp_min = declare_parameter("mapping.occupancy.clamp_min", -2.0);
    config.clamp_max = declare_parameter("mapping.occupancy.clamp_max", 3.5);
    return config;
  }

  void onCloud(Cloud message) {
    const auto stamp = stampNs(message->header.stamp);
    if (stamp <= 0 || message->header.frame_id != lidar_frame_) {
      std::lock_guard lock(mutex_); ++invalid_frame_count_;
      return;
    }
    std::lock_guard lock(mutex_);
    (void)cache_->insertCloud(stamp, std::move(message));
    tryPairLocked(stamp);
  }

  void onOdometry(Odom message) {
    const auto stamp = stampNs(message->header.stamp);
    std::lock_guard lock(mutex_);
    ++corrected_odom_received_;
    if (stamp <= 0 || message->header.frame_id != odom_frame_ ||
        message->child_frame_id != corrected_child_frame_ || !validQuaternion(message->pose.pose.orientation) ||
        !finitePose(message->pose.pose.position)) {
      ++invalid_pose_count_; return;
    }
    last_odom_wall_ = std::chrono::steady_clock::now();
    (void)cache_->insertOdom(stamp, std::move(message));
    tryPairLocked(stamp);
  }

  void tryPairLocked(std::int64_t stamp) {
    auto pair = cache_->takePair(stamp);
    if (!pair.has_value()) return;
    ++paired_observations_;
    if (pending_pair_) ++dropped_after_pair_;
    pending_pair_ = std::make_shared<RawPair>(RawPair{
        std::move(pair->first), std::move(pair->second), std::chrono::steady_clock::now()});
    ready_.notify_one();
  }

  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray& array) {
    for (const auto& status : array.status) {
      if (status.name != "fast_lio/estimator") continue;
      std::unordered_map<std::string, std::string> values;
      for (const auto& value : status.values) values[value.key] = value.value;
      const auto epoch_value = values.find("lio_generation");
      const auto status_value = values.find("status");
      std::lock_guard lock(mutex_);
      if (epoch_value == values.end() || status_value == values.end()) return;
      const auto epoch = std::stoull(epoch_value->second);
      if (epoch_ != 0 && epoch != epoch_) {
        resetForEpochLocked(epoch);
      } else if (epoch_ == 0) {
        epoch_ = epoch;
      }
      lio_tracking_ = status_value->second == "TRACKING";
      {
        std::lock_guard map_lock(map_mutex_);
        if (!lio_tracking_ && map_.validity() == MapValidity::kActive) {
          map_.setValidity(MapValidity::kStale);
        }
        if (status_value->second == "LOST" || status_value->second == "RESETTING") {
          map_.setValidity(MapValidity::kInvalid);
        }
      }
      publishDiagnosticsLocked();
      return;
    }
  }

  static bool validQuaternion(const geometry_msgs::msg::Quaternion& q) {
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return std::isfinite(norm) && norm > 1e-9 && std::abs(norm - 1.0) < 1e-3;
  }

  static bool finitePose(const geometry_msgs::msg::Point& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }

  void resetForEpochLocked(std::uint64_t epoch) {
    epoch_ = epoch;
    {
      std::lock_guard map_lock(map_mutex_);
      map_.reset(); map_.setValidity(MapValidity::kInvalid);
    }
    cache_->clear(); pending_pair_.reset(); ++map_reset_count_;
    last_integrated_stamp_ns_ = -1;
    lio_tracking_ = false;
    RCLCPP_WARN(get_logger(), "LIO continuity epoch changed; navigation map reset (%lu)", epoch);
  }

  bool transformAndRead(const RawPair& pair, Eigen::Vector3d& origin,
                        std::vector<Eigen::Vector3d>& points) {
    const auto& odom = *pair.odom;
    tf2::Transform tf_child_lidar;
    try {
      const auto transform = tf_buffer_->lookupTransform(
          corrected_child_frame_, lidar_frame_, pair.cloud->header.stamp,
          tf2::durationFromSec(0.0));
      tf2::fromMsg(transform.transform, tf_child_lidar);
    } catch (const tf2::TransformException& error) {
      (void)error; return false;
    }
    const Eigen::Quaterniond q_odom_child(odom.pose.pose.orientation.w,
                                          odom.pose.pose.orientation.x,
                                          odom.pose.pose.orientation.y,
                                          odom.pose.pose.orientation.z);
    if (!q_odom_child.coeffs().allFinite()) return false;
    const Eigen::Vector3d p_odom_child(odom.pose.pose.position.x,
                                       odom.pose.pose.position.y,
                                       odom.pose.pose.position.z);
    const Eigen::Quaterniond q_child_lidar(tf_child_lidar.getRotation().w(),
                                           tf_child_lidar.getRotation().x(),
                                           tf_child_lidar.getRotation().y(),
                                           tf_child_lidar.getRotation().z());
    const Eigen::Vector3d p_child_lidar(tf_child_lidar.getOrigin().x(),
                                        tf_child_lidar.getOrigin().y(),
                                        tf_child_lidar.getOrigin().z());
    const auto sensor_pose = composeSensorPose(
        q_odom_child, p_odom_child, q_child_lidar, p_child_lidar);
    origin = sensor_pose.translation;
    sensor_msgs::PointCloud2ConstIterator<float> x(*pair.cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*pair.cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*pair.cloud, "z");
    points.clear(); points.reserve(pair.cloud->width * pair.cloud->height);
    for (; x != x.end(); ++x, ++y, ++z) {
      const Eigen::Vector3d point_lidar(*x, *y, *z);
      if (!point_lidar.allFinite()) continue;
      points.push_back(transformLidarPointToLioOdom(sensor_pose, point_lidar));
    }
    return origin.allFinite();
  }

  void updateLoop() {
    while (true) {
      std::shared_ptr<RawPair> pair;
      {
        std::unique_lock lock(mutex_);
        ready_.wait(lock, [this] { return stopping_ || pending_pair_ != nullptr; });
        if (stopping_ && !pending_pair_) return;
        pair = std::move(pending_pair_);
      }
      {
        std::lock_guard lock(mutex_);
        if (!pair || !lio_tracking_ || epoch_ == 0) {
          ++rejected_invalid_state_; continue;
        }
        const auto stamp = stampNs(pair->cloud->header.stamp);
        if (stamp <= last_integrated_stamp_ns_) {
          ++out_of_order_count_; ++rejected_invalid_state_; continue;
        }
      }
      Eigen::Vector3d origin; std::vector<Eigen::Vector3d> points;
      if (!transformAndRead(*pair, origin, points)) {
        std::lock_guard lock(mutex_);
        ++invalid_frame_count_; ++rejected_invalid_state_; continue;
      }
      const auto update_started = std::chrono::steady_clock::now();
      MapUpdateStats stats;
      {
        std::lock_guard map_lock(map_mutex_);
        stats = map_.update(origin, points);
        map_.setValidity(MapValidity::kActive);
      }
      const auto map_update_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - update_started).count();
      const auto observation_age_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - pair->paired_at).count();
      {
        std::lock_guard lock(mutex_);
        const auto stamp = stampNs(pair->cloud->header.stamp);
        last_map_update_us_ = map_update_us;
        maximum_map_update_us_ = std::max(maximum_map_update_us_, map_update_us);
        last_observation_age_us_ = observation_age_us;
        maximum_observation_age_us_ = std::max(maximum_observation_age_us_, observation_age_us);
        ++integrated_observations_; ++map_update_count_;
        last_integrated_stamp_ns_ = stamp;
        map_shift_count_ = stats.shift_count;
        occupied_voxel_count_ = stats.occupied_voxel_count;
        inflated_voxel_count_ = stats.inflated_voxel_count;
        allocated_voxel_count_ = stats.allocated_voxel_count;
        last_update_wall_ = std::chrono::steady_clock::now();
      }
    }
  }

  void expireCaches() {
    std::lock_guard lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    cache_->expire(now, std::chrono::milliseconds(unmatched_timeout_ms_));
    if (last_odom_wall_.has_value() &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - *last_odom_wall_).count() > stale_timeout_ms_) {
      std::lock_guard map_lock(map_mutex_);
      if (map_.validity() == MapValidity::kActive) map_.setValidity(MapValidity::kStale);
    }
    publishDiagnosticsLocked();
  }

  void publishDiagnosticsLocked() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "rog_map/navigation"; status.hardware_id = "local_voxel_map";
    MapValidity validity;
    {
      std::lock_guard map_lock(map_mutex_);
      validity = map_.validity();
    }
    status.level = validity == MapValidity::kActive
                       ? diagnostic_msgs::msg::DiagnosticStatus::OK
                       : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = stateName(validity);
    const auto counters = cache_->counters();
    auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = value;
      status.values.push_back(std::move(item));
    };
    add("mapping_state", stateName(validity)); add("continuity_epoch", std::to_string(epoch_));
    add("cloud_received", std::to_string(counters.cloud_received));
    add("corrected_odom_received", std::to_string(corrected_odom_received_));
    add("paired_observations", std::to_string(paired_observations_));
    add("integrated_observations", std::to_string(integrated_observations_));
    add("expired_cloud", std::to_string(counters.expired_cloud));
    add("expired_odom", std::to_string(counters.expired_odom));
    add("duplicate_cloud", std::to_string(counters.duplicate_cloud));
    add("duplicate_odom", std::to_string(counters.duplicate_odom));
    add("timestamp_mismatch", std::to_string(counters.timestamp_mismatch));
    add("out_of_order", std::to_string(out_of_order_count_));
    add("rejected_invalid_state", std::to_string(rejected_invalid_state_));
    add("dropped_after_pair", std::to_string(dropped_after_pair_));
    add("invalid_frame", std::to_string(invalid_frame_count_));
    add("invalid_pose", std::to_string(invalid_pose_count_));
    add("map_update_count", std::to_string(map_update_count_));
    add("map_shift_count", std::to_string(map_shift_count_)); add("map_reset_count", std::to_string(map_reset_count_));
    add("occupied_voxel_count", std::to_string(occupied_voxel_count_));
    add("inflated_voxel_count", std::to_string(inflated_voxel_count_));
    add("allocated_voxel_count", std::to_string(allocated_voxel_count_));
    add("map_update_us_latest", std::to_string(last_map_update_us_));
    add("map_update_us_max", std::to_string(maximum_map_update_us_));
    add("observation_age_us_latest", std::to_string(last_observation_age_us_));
    add("observation_age_us_max", std::to_string(maximum_observation_age_us_));
    add("unmatched_cloud_cache_depth", std::to_string(cache_->cloudDepth()));
    add("unmatched_odom_cache_depth", std::to_string(cache_->odomDepth()));
    add("update_queue_depth", pending_pair_ ? "1" : "0"); add("update_queue_bound", "1");
    add("visualization_enabled", visualization_enabled_ ? "true" : "false");
    array.status.push_back(std::move(status)); diagnostics_publisher_->publish(std::move(array));
  }

  static sensor_msgs::msg::PointCloud2 makeVisualizationCloud(
      const std::string& frame_id, const rclcpp::Time& stamp,
      const std::vector<Eigen::Vector3d>& points) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id;
    const auto nanoseconds = stamp.nanoseconds();
    cloud.header.stamp.sec = static_cast<std::int32_t>(nanoseconds / 1'000'000'000LL);
    cloud.header.stamp.nanosec = static_cast<std::uint32_t>(
        nanoseconds % 1'000'000'000LL);
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto& point : points) {
      *x = static_cast<float>(point.x());
      *y = static_cast<float>(point.y());
      *z = static_cast<float>(point.z());
      ++x; ++y; ++z;
    }
    cloud.is_dense = true;
    return cloud;
  }

  void publishVisualization() {
    if (!visualization_enabled_) return;
    std::vector<Eigen::Vector3d> occupied;
    std::vector<Eigen::Vector3d> inflated;
    {
      std::lock_guard map_lock(map_mutex_);
      if (map_.validity() != MapValidity::kActive) return;
      occupied = map_.occupiedVoxelCenters();
      inflated = map_.inflatedVoxelCenters();
    }
    const auto stamp = now();
    if (occupied_visualization_publisher_) {
      occupied_visualization_publisher_->publish(
          makeVisualizationCloud(odom_frame_, stamp, occupied));
    }
    if (inflated_visualization_publisher_) {
      inflated_visualization_publisher_->publish(
          makeVisualizationCloud(odom_frame_, stamp, inflated));
    }
  }

  std::string odom_frame_, corrected_child_frame_, lidar_frame_;
  std::string cloud_topic_, odom_topic_, diagnostics_topic_;
  bool visualization_enabled_{false};
  std::string occupied_visualization_topic_, inflated_visualization_topic_;
  std::int64_t unmatched_timeout_ms_, cache_capacity_, queue_depth_, stale_timeout_ms_;
  VoxelOccupancyMap map_;
  std::unique_ptr<PairCache> cache_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_visualization_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_visualization_publisher_;
  rclcpp::TimerBase::SharedPtr cache_timer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  std::mutex mutex_; std::mutex map_mutex_;
  std::condition_variable ready_; std::thread worker_; bool stopping_{false};
  std::shared_ptr<RawPair> pending_pair_;
  std::uint64_t epoch_{0}; bool lio_tracking_{false};
  std::optional<std::chrono::steady_clock::time_point> last_odom_wall_;
  std::optional<std::chrono::steady_clock::time_point> last_update_wall_;
  std::size_t corrected_odom_received_{0}, paired_observations_{0}, integrated_observations_{0};
  std::size_t rejected_invalid_state_{0}, dropped_after_pair_{0}, invalid_frame_count_{0}, invalid_pose_count_{0};
  std::size_t map_update_count_{0}, map_shift_count_{0}, map_reset_count_{0};
  std::size_t occupied_voxel_count_{0}, inflated_voxel_count_{0}, allocated_voxel_count_{0};
  std::int64_t last_integrated_stamp_ns_{-1};
  std::int64_t last_map_update_us_{0}, maximum_map_update_us_{0};
  std::int64_t last_observation_age_us_{0}, maximum_observation_age_us_{0};
  std::size_t out_of_order_count_{0};
};

}  // namespace uav::nav::rog

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav::nav::rog::RogMapNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
