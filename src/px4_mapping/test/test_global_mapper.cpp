#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <px4_mapping/global_mapper.hpp>
#include <px4_navigation_common/mapping/voxel_map_interface.hpp>

using px4_mapping::get_global_mapper_node;
using px4_navigation_common::mapping::IVoxMapManager;

class GlobalMapperTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // Each test gets a fresh node with default parameters.
        node_options_ = rclcpp::NodeOptions();
        node_options_.append_parameter_override("ready_min_frames", 3);
        node_options_.append_parameter_override("ready_min_occupied", 50);
        node_options_.append_parameter_override("timeout_seconds", 0.2);
        node_options_.append_parameter_override("use_sim_time", false);

        node_ = get_global_mapper_node(node_options_, iface_);
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);

        pub_cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/world/cloud", 20);
        pub_status_ =
            node_->create_publisher<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", 5);
        pub_local_pos_ = node_->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", 5);
        pub_px4_odom_ = node_->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", 20);
        pub_timesync_status_ =
            node_->create_publisher<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", 5);
        pub_lio_odom_ =
            node_->create_publisher<nav_msgs::msg::Odometry>("/localization/odometry", 5);
    }

    void TearDown() override {
        executor_->remove_node(node_);
        executor_.reset();
        node_.reset();
        iface_.reset();
    }

    sensor_msgs::msg::PointCloud2 MakeDenseCloud(uint32_t point_count, float radius) {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.frame_id = "sensor_flu";
        msg.header.stamp = node_->now();
        msg.height = 1;
        msg.width = point_count;
        msg.is_dense = true;
        msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                      sensor_msgs::msg::PointField::FLOAT32);

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(msg, "intensity");

        for (uint32_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            const float angle =
                static_cast<float>(i) * 2.0f * 3.14159265f / static_cast<float>(point_count);
            *iter_x = radius * std::cos(angle);
            *iter_y = radius * std::sin(angle);
            *iter_z = 0.5f;
            *iter_intensity = 100.0f;
        }
        return msg;
    }

    void SpinMs(int ms) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
        while (std::chrono::steady_clock::now() < deadline) {
            executor_->spin_some(std::chrono::milliseconds(10));
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void PublishAlignment(bool armed, bool ekf_valid, double speed, double lio_cov_trace) {
        px4_msgs::msg::VehicleStatus status;
        status.arming_state = armed ? px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED : 0;
        pub_status_->publish(status);

        px4_msgs::msg::VehicleLocalPosition local_pos;
        local_pos.xy_valid = ekf_valid;
        local_pos.z_valid = ekf_valid;
        local_pos.vx = static_cast<float>(speed);
        local_pos.vy = 0.0f;
        local_pos.vz = 0.0f;
        pub_local_pos_->publish(local_pos);

        // Reflect LIO covariance through the LIO odometry topic so the gate sees it.
        nav_msgs::msg::Odometry lio_odom;
        lio_odom.header.stamp = node_->now();
        lio_odom.pose.covariance[0] = lio_cov_trace / 3.0;
        lio_odom.pose.covariance[7] = lio_cov_trace / 3.0;
        lio_odom.pose.covariance[14] = lio_cov_trace / 3.0;
        pub_lio_odom_->publish(lio_odom);

        PublishPx4Odom();
    }

    void PublishPx4Odom(double x = 0.0, double y = 0.0, double z = 0.0) {
        px4_msgs::msg::TimesyncStatus timesync_status;
        timesync_status.timestamp = static_cast<uint64_t>(node_->now().nanoseconds() / 1000LL);
        timesync_status.estimated_offset = 0;
        pub_timesync_status_->publish(timesync_status);

        px4_msgs::msg::VehicleOdometry px4_odom;
        const uint64_t now_us = static_cast<uint64_t>(node_->now().nanoseconds() / 1000LL);
        px4_odom.timestamp = now_us;
        px4_odom.timestamp_sample = now_us;
        px4_odom.position[0] = static_cast<float>(x);
        px4_odom.position[1] = static_cast<float>(y);
        px4_odom.position[2] = static_cast<float>(z);
        px4_odom.q[0] = 1.0f;
        px4_odom.q[1] = 0.0f;
        px4_odom.q[2] = 0.0f;
        px4_odom.q[3] = 0.0f;
        pub_px4_odom_->publish(px4_odom);
    }

    rclcpp::NodeOptions node_options_;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<IVoxMapManager> iface_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr pub_status_;
    rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pub_local_pos_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_px4_odom_;
    rclcpp::Publisher<px4_msgs::msg::TimesyncStatus>::SharedPtr pub_timesync_status_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_lio_odom_;
};

TEST_F(GlobalMapperTest, InitiallyNotReady) {
    EXPECT_FALSE(iface_->IsReady());
    EXPECT_EQ(iface_->FramesDropped(), 0U);
}

TEST_F(GlobalMapperTest, BecomesReadyAfterSufficientOccupiedFrames) {
    // Publish enough points over enough consecutive frames to satisfy both
    // the occupancy and consecutive-frame requirements.
    for (int i = 0; i < 5; ++i) {
        PublishPx4Odom();
        pub_cloud_->publish(MakeDenseCloud(200, 3.0f));
        SpinMs(100);
    }

    EXPECT_TRUE(iface_->IsReady()) << "Node should be ready after 5 frames of dense data";
}

TEST_F(GlobalMapperTest, DropsReadyWhenDataStops) {
    // Make the node ready first.
    for (int i = 0; i < 5; ++i) {
        PublishPx4Odom();
        pub_cloud_->publish(MakeDenseCloud(200, 3.0f));
        SpinMs(100);
    }
    ASSERT_TRUE(iface_->IsReady());

    // Stop publishing. The timeout_seconds_ is set to 0.2s in SetUp(), so spin
    // long enough for the 1 Hz timeout timer to fire and mark data stale.
    SpinMs(500);
    EXPECT_FALSE(iface_->IsReady()) << "Node should drop ready after data timeout";
}

// Fixture that enables the alignment gate with short thresholds so the gate
// can be exercised quickly in unit tests.
class GlobalMapperAlignmentTest : public GlobalMapperTest {
   protected:
    void SetUp() override {
        node_options_ = rclcpp::NodeOptions();
        node_options_.append_parameter_override("ready_min_frames", 3);
        node_options_.append_parameter_override("ready_min_occupied", 50);
        node_options_.append_parameter_override("timeout_seconds", 2.0);
        node_options_.append_parameter_override("use_sim_time", false);
        node_options_.append_parameter_override("require_alignment_gate", true);
        node_options_.append_parameter_override("aligned_min_seconds", 0.3);
        node_options_.append_parameter_override("aligned_max_velocity", 0.1);
        node_options_.append_parameter_override("aligned_lio_covariance_max", 1.0);

        node_ = get_global_mapper_node(node_options_, iface_);
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);

        pub_cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/world/cloud", 20);
        pub_status_ =
            node_->create_publisher<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", 5);
        pub_local_pos_ = node_->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", 5);
        pub_px4_odom_ = node_->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", 20);
        pub_timesync_status_ =
            node_->create_publisher<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", 5);
        pub_lio_odom_ =
            node_->create_publisher<nav_msgs::msg::Odometry>("/localization/odometry", 5);
    }
};

TEST_F(GlobalMapperAlignmentTest, GateBlocksReadyUntilConditionsMet) {
    // Without alignment inputs the gate stays open for occupancy but alignment
    // is not captured, so IsReady() must remain false even with dense clouds.
    for (int i = 0; i < 5; ++i) {
        PublishPx4Odom();
        pub_cloud_->publish(MakeDenseCloud(200, 3.0f));
        SpinMs(100);
    }
    EXPECT_FALSE(iface_->IsReady()) << "Ready should be blocked before alignment is captured";

    // Publish aligned conditions and wait for the streak timer.
    // Keep publishing clouds so the data-fresh timeout does not expire
    // while the alignment gate is accumulating its 0.3 s streak.
    bool became_ready = false;
    for (int i = 0; i < 30; ++i) {
        PublishAlignment(true, true, 0.0, 0.001);
        PublishPx4Odom();
        pub_cloud_->publish(MakeDenseCloud(200, 3.0f));
        SpinMs(100);
        if (iface_->IsReady()) {
            became_ready = true;
            break;
        }
    }

    EXPECT_TRUE(became_ready) << "Ready should become true after alignment conditions hold";
}

TEST_F(GlobalMapperAlignmentTest, GateBlockedWhenMoving) {
    // Arm + valid EKF but moving too fast -> gate should not capture.
    // Publish alignment continuously so the speed constraint is seen by the
    // 10 Hz alignment tick (a single publish can race against gate evaluation).
    for (int i = 0; i < 5; ++i) {
        PublishAlignment(true, true, 1.0, 0.001);
        PublishPx4Odom();
        pub_cloud_->publish(MakeDenseCloud(200, 3.0f));
        SpinMs(100);
    }
    SpinMs(400);
    EXPECT_FALSE(iface_->IsReady()) << "Alignment should not capture when drone speed is high";
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}
