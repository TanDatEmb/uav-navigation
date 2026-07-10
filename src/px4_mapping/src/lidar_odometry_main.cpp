#include <px4_mapping/lidar_odometry.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<px4_mapping::LidarOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
