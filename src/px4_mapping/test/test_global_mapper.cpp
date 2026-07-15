#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2_utils/px4/topic.hpp>
#include <px4_ros2_utils/time/timesync.hpp>

#include <px4_mapping/global_mapper.hpp>
#include <px4_nav_common/mapping/voxel_map_interface.hpp>

using px4_mapping::get_global_mapper_node;
using px4_nav_common::mapping::IVoxMapManager;

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
        pub_status_ = node_->create_publisher<px4_msgs::msg::VehicleStatus>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::VehicleStatus>(
                "/fmu/out/vehicle_status"),
            5);
        pub_local_pos_ = node_->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position"),
            5);
        pub_px4_odom_ = node_->create_publisher<px4_msgs::msg::VehicleOdometry>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry"),
            20);
        pub_timesync_status_ = node_->create_publisher<px4_msgs::msg::TimesyncStatus>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::TimesyncStatus>(
                "/fmu/out/timesync_status"),
            5);
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
        // Unit-test clock model: an explicit zero-offset Timesync keeps ROS and
        // PX4 test clocks aligned without casting between timestamp domains.
        px4_ros2_utils::time::Timesync zero_offset_timesync(node_->get_clock());
        zero_offset_timesync.update_manual(1U, 0);
        const auto now_us = zero_offset_timesync.toPX4(node_->now());
        ASSERT_TRUE(now_us.has_value());
        ASSERT_NE(*now_us, 0U);

        px4_msgs::msg::TimesyncStatus timesync_status;
        timesync_status.timestamp = *now_us;
        timesync_status.estimated_offset = 0;
        pub_timesync_status_->publish(timesync_status);

        px4_msgs::msg::VehicleOdometry px4_odom;
        px4_odom.timestamp = *now_us;
        px4_odom.timestamp_sample = *now_us;
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

TEST_F(GlobalMapperTest, RejectsNonPositiveGlobalMapPublishInterval) {
    auto invalid_options = rclcpp::NodeOptions();
    invalid_options.append_parameter_override("global_map_publish_interval", 0);
    std::shared_ptr<IVoxMapManager> invalid_iface;
    EXPECT_THROW(get_global_mapper_node(invalid_options, invalid_iface), std::runtime_error);
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

class GlobalMapperLioWorldTest : public GlobalMapperTest {
   protected:
    void SetUp() override {
        node_options_ = rclcpp::NodeOptions();
        node_options_.append_parameter_override("ready_min_frames", 1);
        node_options_.append_parameter_override("ready_min_occupied", 50);
        node_options_.append_parameter_override("timeout_seconds", 2.0);
        node_options_.append_parameter_override("use_sim_time", false);
        node_options_.append_parameter_override("input_source", "lio_world");
        node_options_.append_parameter_override("cloud_topic", "/lio/cloud_registered");

        node_ = get_global_mapper_node(node_options_, iface_);
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        pub_cloud_ =
            node_->create_publisher<sensor_msgs::msg::PointCloud2>("/lio/cloud_registered", 20);
        pub_lio_odom_ =
            node_->create_publisher<nav_msgs::msg::Odometry>("/localization/odometry", 5);
    }
};

TEST_F(GlobalMapperLioWorldTest, RequiresLioPoseBeforeRaycastingRegisteredCloud) {
    auto cloud_without_pose = MakeDenseCloud(200, 3.0f);
    cloud_without_pose.header.frame_id = "lio_world";
    pub_cloud_->publish(cloud_without_pose);
    SpinMs(100);
    EXPECT_FALSE(iface_->IsReady());
    EXPECT_GT(iface_->FramesDropped(), 0U);

    const auto synchronized_stamp = node_->now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = synchronized_stamp;
    odom.header.frame_id = "lio_world";
    odom.pose.pose.position.x = 10.0;
    odom.pose.pose.orientation.w = std::sqrt(0.5);
    odom.pose.pose.orientation.z = std::sqrt(0.5);
    pub_lio_odom_->publish(odom);
    SpinMs(50);

    auto synchronized_cloud = MakeDenseCloud(200, 3.0f);
    synchronized_cloud.header.frame_id = "lio_world";
    synchronized_cloud.header.stamp = synchronized_stamp;
    pub_cloud_->publish(synchronized_cloud);
    SpinMs(150);

    EXPECT_TRUE(iface_->IsReady());
}

TEST_F(GlobalMapperLioWorldTest, PublishesAccumulatedOccupiedMap) {
    bool latest_has_first_region = false;
    bool latest_has_second_region = false;
    std::size_t latest_width = 0U;
    std::size_t map_messages = 0U;

    auto sub_map = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mapping/global", 10, [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            latest_has_first_region = false;
            latest_has_second_region = false;
            latest_width = msg->width;
            ++map_messages;

            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            for (; iter_x != iter_x.end(); ++iter_x) {
                latest_has_first_region = latest_has_first_region || (*iter_x < 4.0f);
                latest_has_second_region = latest_has_second_region || (*iter_x > 8.0f);
            }
        });
    ASSERT_NE(sub_map, nullptr);
    SpinMs(50);

    const auto publish_scan = [&](double sensor_x, float cloud_x_offset) {
        const auto stamp = node_->now();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "lio_world";
        odom.child_frame_id = "mid360_imu";
        odom.pose.pose.position.x = sensor_x;
        odom.pose.pose.orientation.w = 1.0;
        pub_lio_odom_->publish(odom);
        SpinMs(50);

        auto cloud = MakeDenseCloud(200, 3.0f);
        cloud.header.stamp = stamp;
        cloud.header.frame_id = "lio_world";
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        for (; iter_x != iter_x.end(); ++iter_x) {
            *iter_x += cloud_x_offset;
        }
        pub_cloud_->publish(cloud);
        SpinMs(200);
    };

    publish_scan(0.0, 0.0f);
    ASSERT_GT(map_messages, 0U);
    ASSERT_TRUE(latest_has_first_region);
    const std::size_t first_width = latest_width;

    publish_scan(12.0, 12.0f);
    EXPECT_TRUE(latest_has_first_region)
        << "Previously occupied voxels must remain in each full-map publication";
    EXPECT_TRUE(latest_has_second_region);
    EXPECT_GT(latest_width, first_width);
}

TEST_F(GlobalMapperLioWorldTest, PublishesRadiusBoundedLocalMapFromGlobalOccupancy) {
    bool global_has_first_region = false;
    bool global_has_second_region = false;
    bool local_has_first_region = false;
    bool local_has_second_region = false;
    std::size_t global_messages = 0U;
    std::size_t local_messages = 0U;

    auto inspect_regions = [](const sensor_msgs::msg::PointCloud2& msg, bool& has_first,
                              bool& has_second) {
        has_first = false;
        has_second = false;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
        for (; iter_x != iter_x.end(); ++iter_x) {
            has_first = has_first || (*iter_x < 5.0f);
            has_second = has_second || (*iter_x > 15.0f);
        }
    };

    auto sub_global = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mapping/global", 10, [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            inspect_regions(*msg, global_has_first_region, global_has_second_region);
            ++global_messages;
        });
    auto sub_local = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mapping/local", 10, [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            inspect_regions(*msg, local_has_first_region, local_has_second_region);
            ++local_messages;
        });
    ASSERT_NE(sub_global, nullptr);
    ASSERT_NE(sub_local, nullptr);
    SpinMs(50);

    const auto publish_scan = [&](double sensor_x, float cloud_x_offset) {
        const auto stamp = node_->now();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "lio_world";
        odom.pose.pose.position.x = sensor_x;
        odom.pose.pose.orientation.w = 1.0;
        pub_lio_odom_->publish(odom);
        SpinMs(50);

        auto cloud = MakeDenseCloud(200, 3.0f);
        cloud.header.stamp = stamp;
        cloud.header.frame_id = "lio_world";
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        for (; iter_x != iter_x.end(); ++iter_x) {
            *iter_x += cloud_x_offset;
        }
        pub_cloud_->publish(cloud);
        SpinMs(200);
    };

    publish_scan(0.0, 0.0f);
    ASSERT_GT(global_messages, 0U);
    ASSERT_GT(local_messages, 0U);
    ASSERT_TRUE(global_has_first_region);
    ASSERT_TRUE(local_has_first_region);

    publish_scan(20.0, 20.0f);
    EXPECT_TRUE(global_has_first_region);
    EXPECT_TRUE(global_has_second_region);
    EXPECT_FALSE(local_has_first_region);
    EXPECT_TRUE(local_has_second_region);
}

class GlobalMapperThrottledPublishTest : public GlobalMapperTest {
   protected:
    void SetUp() override {
        node_options_ = rclcpp::NodeOptions();
        node_options_.append_parameter_override("ready_min_frames", 1);
        node_options_.append_parameter_override("ready_min_occupied", 1);
        node_options_.append_parameter_override("timeout_seconds", 2.0);
        node_options_.append_parameter_override("use_sim_time", false);
        node_options_.append_parameter_override("input_source", "lio_world");
        node_options_.append_parameter_override("cloud_topic", "/lio/cloud_registered");
        node_options_.append_parameter_override("global_map_publish_interval", 3);

        node_ = get_global_mapper_node(node_options_, iface_);
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        pub_cloud_ =
            node_->create_publisher<sensor_msgs::msg::PointCloud2>("/lio/cloud_registered", 20);
        pub_lio_odom_ =
            node_->create_publisher<nav_msgs::msg::Odometry>("/localization/odometry", 5);
    }
};

TEST_F(GlobalMapperThrottledPublishTest, ThrottlesGlobalButNotLocalMap) {
    std::size_t global_messages = 0U;
    std::size_t local_messages = 0U;
    auto sub_global = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mapping/global", 10, [&](const sensor_msgs::msg::PointCloud2::SharedPtr) {
            ++global_messages;
        });
    auto sub_local = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mapping/local", 10, [&](const sensor_msgs::msg::PointCloud2::SharedPtr) {
            ++local_messages;
        });
    ASSERT_NE(sub_global, nullptr);
    ASSERT_NE(sub_local, nullptr);
    SpinMs(100);

    for (int i = 0; i < 4; ++i) {
        const auto stamp = node_->now();
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "lio_world";
        odom.pose.pose.orientation.w = 1.0;
        pub_lio_odom_->publish(odom);
        SpinMs(30);

        auto cloud = MakeDenseCloud(100, 3.0f);
        cloud.header.stamp = stamp;
        cloud.header.frame_id = "lio_world";
        pub_cloud_->publish(cloud);
        SpinMs(100);
    }

    EXPECT_EQ(global_messages, 2U) << "Global map should publish on frames 1 and 4";
    EXPECT_EQ(local_messages, 4U) << "Local map should publish on every accepted frame";
}

class GlobalMapperPx4RetentionTest : public GlobalMapperTest {
   protected:
    void SetUp() override {
        SetUpWithDistanceEviction(false);
    }

    void SetUpWithDistanceEviction(bool enable_distance_eviction) {
        node_options_ = rclcpp::NodeOptions();
        node_options_.append_parameter_override("ready_min_frames", 1);
        node_options_.append_parameter_override("ready_min_occupied", 1);
        node_options_.append_parameter_override("timeout_seconds", 5.0);
        node_options_.append_parameter_override("use_sim_time", false);
        node_options_.append_parameter_override("input_source", "px4_full");
        node_options_.append_parameter_override("use_lio_buffer", false);
        node_options_.append_parameter_override("enable_distance_eviction",
                                                enable_distance_eviction);

        node_ = get_global_mapper_node(node_options_, iface_);
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);

        pub_cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/world/cloud", 20);
        pub_px4_odom_ = node_->create_publisher<px4_msgs::msg::VehicleOdometry>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry"),
            20);
        pub_timesync_status_ = node_->create_publisher<px4_msgs::msg::TimesyncStatus>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::TimesyncStatus>(
                "/fmu/out/timesync_status"),
            5);
    }

    void PrimePx4Pose(double x, double y, double z) {
        // The first pair establishes Timesync; the second odometry sample is
        // deterministic even when DDS delivers the initial callbacks out of order.
        PublishPx4Odom(x, y, z);
        SpinMs(50);
        PublishPx4Odom(x, y, z);
        SpinMs(50);
    }
};

TEST_F(GlobalMapperPx4RetentionTest, GlobalHistoryIsIndependentOfInputSource) {
    bool global_has_origin_region = false;
    bool global_has_far_region = false;
    bool local_has_origin_region = false;
    bool local_has_far_region = false;

    auto inspect_regions = [](const sensor_msgs::msg::PointCloud2& msg, bool& has_origin,
                              bool& has_far) {
        has_origin = false;
        has_far = false;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
        for (; iter_x != iter_x.end(); ++iter_x) {
            has_origin = has_origin || (*iter_x < 10.0f);
            has_far = has_far || (*iter_x > 90.0f);
        }
    };

    auto sub_global = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mapping/global", 10, [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            inspect_regions(*msg, global_has_origin_region, global_has_far_region);
        });
    auto sub_local = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mapping/local", 10, [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            inspect_regions(*msg, local_has_origin_region, local_has_far_region);
        });
    ASSERT_NE(sub_global, nullptr);
    ASSERT_NE(sub_local, nullptr);
    SpinMs(50);

    PrimePx4Pose(0.0, 0.0, 0.0);
    pub_cloud_->publish(MakeDenseCloud(200, 3.0f));
    SpinMs(200);
    ASSERT_TRUE(global_has_origin_region);
    ASSERT_TRUE(local_has_origin_region);

    PrimePx4Pose(100.0, 0.0, 0.0);
    SpinMs(700);  // Exceeds the legacy 500 ms distance-eviction interval.
    pub_cloud_->publish(MakeDenseCloud(200, 3.0f));
    SpinMs(200);

    EXPECT_TRUE(global_has_origin_region);
    EXPECT_TRUE(global_has_far_region);
    EXPECT_FALSE(local_has_origin_region);
    EXPECT_TRUE(local_has_far_region);
}

class GlobalMapperPx4DistanceEvictionTest : public GlobalMapperPx4RetentionTest {
   protected:
    void SetUp() override {
        SetUpWithDistanceEviction(true);
    }
};

TEST_F(GlobalMapperPx4DistanceEvictionTest, ExplicitOptInEvictsLegacyDistantHistory) {
    bool has_origin_region = false;
    bool has_far_region = false;
    auto sub_global = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mapping/global", 10, [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            has_origin_region = false;
            has_far_region = false;
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            for (; iter_x != iter_x.end(); ++iter_x) {
                has_origin_region = has_origin_region || (*iter_x < 10.0f);
                has_far_region = has_far_region || (*iter_x > 90.0f);
            }
        });
    ASSERT_NE(sub_global, nullptr);
    SpinMs(50);

    PrimePx4Pose(0.0, 0.0, 0.0);
    pub_cloud_->publish(MakeDenseCloud(200, 3.0f));
    SpinMs(200);
    ASSERT_TRUE(has_origin_region);

    PrimePx4Pose(100.0, 0.0, 0.0);
    SpinMs(700);
    pub_cloud_->publish(MakeDenseCloud(200, 3.0f));
    SpinMs(200);

    EXPECT_FALSE(has_origin_region);
    EXPECT_TRUE(has_far_region);
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
        pub_status_ = node_->create_publisher<px4_msgs::msg::VehicleStatus>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::VehicleStatus>(
                "/fmu/out/vehicle_status"),
            5);
        pub_local_pos_ = node_->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position"),
            5);
        pub_px4_odom_ = node_->create_publisher<px4_msgs::msg::VehicleOdometry>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry"),
            20);
        pub_timesync_status_ = node_->create_publisher<px4_msgs::msg::TimesyncStatus>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::TimesyncStatus>(
                "/fmu/out/timesync_status"),
            5);
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
