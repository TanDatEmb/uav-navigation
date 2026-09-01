#include <gz/msgs/pointcloud_packed.pb.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <gz/transport/Node.hh>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/convert/sensor_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace uav::simulation {

namespace {

using GzPointCloud = gz::msgs::PointCloudPacked;

struct BridgeState {
  std::mutex inbox_mutex;
  std::condition_variable inbox_cv;
  std::unique_ptr<GzPointCloud> latest_cloud;
  std::atomic<bool> stopping{false};
  std::atomic<std::uint64_t> ingress_count{0U};
  std::atomic<std::uint64_t> dropped_count{0U};
  std::atomic<std::uint64_t> published_count{0U};
  std::atomic<std::uint64_t> copy_failure_count{0U};
  std::atomic<std::uint64_t> convert_failure_count{0U};
  std::atomic<std::uint64_t> publish_failure_count{0U};
};

}  // namespace

class GzLidarBridge final : public rclcpp::Node {
 public:
  GzLidarBridge()
      : Node("gz_lidar_bridge"),
        state_(std::make_shared<BridgeState>()),
        input_topic_(declare_parameter<std::string>("input_topic", "/sim/mid360/scan/points")),
        ros_topic_(declare_parameter<std::string>("ros_topic", "/lidar/points")) {
    if (input_topic_.empty() || ros_topic_.empty()) {
      throw std::invalid_argument("LiDAR bridge topics must not be empty");
    }

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ros_topic_, rclcpp::SensorDataQoS{}.keep_last(1));

    std::function<void(const GzPointCloud&)> callback =
        [state = state_](const GzPointCloud& cloud) noexcept {
          if (state->stopping.load(std::memory_order_acquire)) return;

          state->ingress_count.fetch_add(1U, std::memory_order_relaxed);
          std::unique_ptr<GzPointCloud> copy;
          try {
            copy = std::make_unique<GzPointCloud>(cloud);
          } catch (...) {
            state->copy_failure_count.fetch_add(1U, std::memory_order_relaxed);
            return;
          }

          {
            std::lock_guard<std::mutex> lock(state->inbox_mutex);
            if (state->stopping.load(std::memory_order_acquire)) return;
            if (state->latest_cloud) {
              state->dropped_count.fetch_add(1U, std::memory_order_relaxed);
            }
            state->latest_cloud = std::move(copy);
          }
          state->inbox_cv.notify_one();
        };

    gz_subscriber_ = gz_node_.CreateSubscriber(input_topic_, callback);
    if (!gz_subscriber_) {
      throw std::runtime_error("failed to subscribe to Gazebo LiDAR topic " + input_topic_);
    }

    worker_ = std::thread([this]() { workerLoop(); });
    diagnostic_timer_ =
        create_wall_timer(std::chrono::seconds(10), [this]() { logMetrics("periodic"); });

    RCLCPP_INFO(get_logger(),
                "bridging Gazebo PointCloudPacked %s -> ROS PointCloud2 %s "
                "with latest-only inbox depth=1 and SensorDataQoS depth=1",
                input_topic_.c_str(), ros_topic_.c_str());
  }

  ~GzLidarBridge() override {
    if (diagnostic_timer_) diagnostic_timer_->cancel();
    state_->stopping.store(true, std::memory_order_release);
    gz_subscriber_.Unsubscribe();
    state_->inbox_cv.notify_all();
    if (worker_.joinable()) worker_.join();
    logMetrics("shutdown");
  }

 private:
  void workerLoop() {
    while (true) {
      std::unique_ptr<GzPointCloud> cloud;
      {
        std::unique_lock<std::mutex> lock(state_->inbox_mutex);
        state_->inbox_cv.wait(lock, [this]() {
          return state_->stopping.load(std::memory_order_acquire) ||
                 state_->latest_cloud != nullptr;
        });
        if (state_->stopping.load(std::memory_order_acquire)) return;
        cloud = std::move(state_->latest_cloud);
      }

      sensor_msgs::msg::PointCloud2 converted;
      try {
        ros_gz_bridge::convert_gz_to_ros(*cloud, converted);
      } catch (const std::exception& error) {
        state_->convert_failure_count.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "PointCloud conversion failed: %s",
                             error.what());
        continue;
      } catch (...) {
        state_->convert_failure_count.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "PointCloud conversion failed with unknown error");
        continue;
      }

      try {
        publisher_->publish(std::move(converted));
        state_->published_count.fetch_add(1U, std::memory_order_relaxed);
      } catch (const std::exception& error) {
        state_->publish_failure_count.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "PointCloud publish failed: %s",
                             error.what());
      } catch (...) {
        state_->publish_failure_count.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "PointCloud publish failed with unknown error");
      }
    }
  }

  void logMetrics(const char* phase) const {
    RCLCPP_INFO(
        get_logger(),
        "gz_lidar_bridge_metrics phase=%s ingress=%llu published=%llu "
        "dropped=%llu copy_failures=%llu convert_failures=%llu "
        "publish_failures=%llu",
        phase,
        static_cast<unsigned long long>(state_->ingress_count.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(state_->published_count.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(state_->dropped_count.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(state_->copy_failure_count.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            state_->convert_failure_count.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            state_->publish_failure_count.load(std::memory_order_relaxed)));
  }

  std::shared_ptr<BridgeState> state_;
  std::string input_topic_;
  std::string ros_topic_;
  gz::transport::Node gz_node_;
  gz::transport::Node::Subscriber gz_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::thread worker_;
};

}  // namespace uav::simulation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<uav::simulation::GzLidarBridge>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("gz_lidar_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
