#include <px4_mapping/fast_lio2_node.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<px4_mapping::FastLio2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
