#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <functional>
#include <stdexcept>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>

#include "uav_simulation/visibility_cloud.hpp"

namespace uav::simulation {

class VisibilityBridge final : public rclcpp::Node {
 public:
  VisibilityBridge()
      : Node("gz_visibility_bridge"),
        gz_topic_(declare_parameter("gz_topic", "/sim/mid360/scan")),
        ros_topic_(declare_parameter("ros_topic", "/lidar/free_space_endpoints")),
        expected_frame_(declare_parameter("expected_frame", "livox_frame")),
        maximum_endpoints_(declare_parameter("maximum_endpoints", 4096)) {
    if (maximum_endpoints_ <= 0 || maximum_endpoints_ > 262144) {
      throw std::invalid_argument("maximum_endpoints must be in [1, 262144]");
    }
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ros_topic_, rclcpp::QoS{rclcpp::KeepLast{10}}.reliable().durability_volatile());
    const std::function<void(const gz::msgs::LaserScan&)> callback =
        [this](const gz::msgs::LaserScan& scan) { onScan(scan); };
    if (!gz_node_.Subscribe(gz_topic_, callback)) {
      throw std::runtime_error("failed to subscribe to Gazebo visibility scan: " +
                               gz_topic_);
    }
  }

 private:
  void onScan(const gz::msgs::LaserScan& scan) {
    const auto converted = makeVisibilityCloud(
        scan, expected_frame_, static_cast<std::size_t>(maximum_endpoints_));
    if (!converted.has_value()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "discarding malformed Gazebo visibility scan");
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
  }

  std::string gz_topic_;
  std::string ros_topic_;
  std::string expected_frame_;
  std::int64_t maximum_endpoints_;
  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace uav::simulation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav::simulation::VisibilityBridge>());
  rclcpp::shutdown();
  return 0;
}
