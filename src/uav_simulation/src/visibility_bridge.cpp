#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "uav_simulation/visibility_cloud.hpp"

namespace uav::simulation {

class VisibilityBridge final : public rclcpp::Node {
 public:
  VisibilityBridge()
      : Node("gz_visibility_bridge"),
        input_topic_(declare_parameter("input_topic", "/lidar/points")),
        ros_topic_(declare_parameter("ros_topic", "/lidar/free_space_endpoints")),
        expected_frame_(declare_parameter("expected_frame", "livox_frame")),
        maximum_endpoints_(declare_parameter("maximum_endpoints", 4096)),
        visibility_config_{
            static_cast<std::uint32_t>(declare_parameter("horizontal_count", 720)),
            static_cast<std::uint32_t>(declare_parameter("vertical_count", 28)),
            declare_parameter("horizontal_angle_min_rad", -3.141592653589793),
            declare_parameter("horizontal_angle_max_rad", 3.141592653589793),
            declare_parameter("vertical_angle_min_rad", -0.122173047639603),
            declare_parameter("vertical_angle_max_rad", 0.907571211037051),
            declare_parameter("range_max_m", 40.0)} {
    if (input_topic_.empty() || expected_frame_.empty() ||
        maximum_endpoints_ <= 0 || maximum_endpoints_ > 262144 ||
        visibility_config_.horizontal_count == 0U ||
        visibility_config_.vertical_count == 0U ||
        visibility_config_.horizontal_count > 262144U ||
        visibility_config_.vertical_count > 262144U) {
      throw std::invalid_argument("invalid organized visibility bridge contract");
    }
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ros_topic_, rclcpp::QoS{rclcpp::KeepLast{10}}.reliable().durability_volatile());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS{}.keep_last(1),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
          onCloud(std::move(cloud));
        });
    worker_ = std::thread([this]() { workerLoop(); });
  }

  ~VisibilityBridge() override {
    subscription_.reset();
    stopping_.store(true, std::memory_order_release);
    inbox_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    RCLCPP_INFO(
        get_logger(),
        "visibility_bridge_metrics received=%lu replaced=%lu published=%lu malformed=%lu",
        received_count_.load(), replaced_count_.load(), published_count_.load(),
        malformed_count_.load());
  }

 private:
  void onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
    if (stopping_.load(std::memory_order_acquire)) return;
    {
      std::lock_guard<std::mutex> lock(inbox_mutex_);
      if (latest_cloud_) replaced_count_.fetch_add(1U, std::memory_order_relaxed);
      latest_cloud_ = std::move(cloud);
      received_count_.fetch_add(1U, std::memory_order_relaxed);
    }
    inbox_cv_.notify_one();
  }

  void workerLoop() {
    while (true) {
      sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
      {
        std::unique_lock<std::mutex> lock(inbox_mutex_);
        inbox_cv_.wait(lock, [this]() {
          return stopping_.load(std::memory_order_acquire) || latest_cloud_;
        });
        if (stopping_.load(std::memory_order_acquire)) return;
        cloud = std::move(latest_cloud_);
      }
      publishCloud(*cloud);
    }
  }

  void publishCloud(const sensor_msgs::msg::PointCloud2& input) {
    const auto converted = makeVisibilityCloud(
        input, visibility_config_, expected_frame_,
        static_cast<std::size_t>(maximum_endpoints_));
    if (!converted.has_value()) {
      malformed_count_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "discarding malformed organized visibility cloud");
      return;
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = converted->frame_id;
    cloud.header.stamp.sec = converted->stamp_sec;
    cloud.header.stamp.nanosec = converted->stamp_nanosec;
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(converted->endpoints.size());
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(converted->endpoints.size());
    sensor_msgs::msg::PointField source_count;
    source_count.name = "visibility_source_ray_count";
    source_count.offset = 0U;
    source_count.datatype = sensor_msgs::msg::PointField::UINT8;
    source_count.count = converted->source_ray_count;
    cloud.fields.push_back(source_count);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto& endpoint : converted->endpoints) {
      *x = endpoint.x;
      *y = endpoint.y;
      *z = endpoint.z;
      ++x;
      ++y;
      ++z;
    }
    publisher_->publish(std::move(cloud));
    published_count_.fetch_add(1U, std::memory_order_relaxed);
  }

  std::string input_topic_;
  std::string ros_topic_;
  std::string expected_frame_;
  std::int64_t maximum_endpoints_;
  OrganizedVisibilityConfig visibility_config_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::mutex inbox_mutex_;
  std::condition_variable inbox_cv_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
  std::atomic<bool> stopping_{false};
  std::thread worker_;
  std::atomic<std::uint64_t> received_count_{0U};
  std::atomic<std::uint64_t> replaced_count_{0U};
  std::atomic<std::uint64_t> published_count_{0U};
  std::atomic<std::uint64_t> malformed_count_{0U};
};

}  // namespace uav::simulation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav::simulation::VisibilityBridge>());
  rclcpp::shutdown();
  return 0;
}
