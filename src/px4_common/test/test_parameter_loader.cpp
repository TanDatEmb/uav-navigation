#include <px4_common/utils/parameter_loader.hpp>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

namespace {

using px4_common::utils::LoadParam;

class ParameterLoaderTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node_ = std::make_shared<rclcpp::Node>("test_parameter_loader_node");
    }

    void TearDown() override {
        node_.reset();
    }

    std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(ParameterLoaderTest, LoadsDeclaredDefaultValue) {
    const int value = LoadParam(*node_, "test_int", 42, "Test integer parameter");
    EXPECT_EQ(value, 42);
    EXPECT_TRUE(node_->has_parameter("test_int"));
}

TEST_F(ParameterLoaderTest, LoadsExternallySetValue) {
    node_.reset();
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("test_double", 3.14)});
    node_ = std::make_shared<rclcpp::Node>("test_parameter_loader_node", options);

    const double value = LoadParam(*node_, "test_double", 1.0);
    EXPECT_DOUBLE_EQ(value, 3.14);
}

TEST_F(ParameterLoaderTest, LoadsStringParameter) {
    const std::string value =
        LoadParam(*node_, "test_string", std::string("default"), "Test string parameter");
    EXPECT_EQ(value, "default");
}

}  // namespace
