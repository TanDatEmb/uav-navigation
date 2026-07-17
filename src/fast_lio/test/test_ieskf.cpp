#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include "fast_lio/ieskf.hpp"
#include "fast_lio/imu_processor.hpp"
#include "fast_lio/lidar_processor.hpp"
#include "fast_lio/map_builder.hpp"
#include "fast_lio/spatial_index.hpp"
#include "fast_lio/utils.hpp"

namespace fast_lio {
namespace {

TEST(IESKFTest, LargeMeasurementUpdateUsesFixedStateSolveAndCorrectResidualSign) {
    constexpr int kMeasurements = 20000;
    auto filter = std::make_shared<IESKF>();
    filter->setMeasurementCallback([](const State15&, SharedState15& shared) {
        shared.reset(kMeasurements);
        shared.H.col(3).setOnes();
        shared.b.setOnes();
        shared.num_measurements = kMeasurements;
        shared.valid = true;
        return true;
    });

    const auto start = std::chrono::steady_clock::now();
    filter->update(1);
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);

    EXPECT_LT(elapsed.count(), 2.0)
        << "A 15-state information solve should not scale cubically with measurement count";
    EXPECT_LT(filter->getState().p_w.x(), 0.0)
        << "Positive h(x)-z residual must produce a negative correction";
    EXPECT_TRUE(filter->getCovariance().allFinite());
    Eigen::LLT<Eigen::Matrix<double, 15, 15>> covariance_llt(filter->getCovariance());
    EXPECT_EQ(covariance_llt.info(), Eigen::Success);
}

TEST(IESKFTest, RejectsStateUpdateWhenFinalCovarianceLinearizationFails) {
    IESKF filter;
    const State15 state_before = filter.getState();
    const Eigen::Matrix<double, 15, 15> covariance_before = filter.getCovariance();
    int loss_calls = 0;

    filter.setMeasurementCallback([&loss_calls](const State15&, SharedState15& shared) {
        ++loss_calls;
        if (loss_calls > 1) {
            return false;
        }

        shared.reset(1);
        shared.H(0, 3) = 1.0;
        shared.b(0) = 1.0;
        shared.num_measurements = 1;
        shared.valid = true;
        return true;
    });

    filter.update(1);

    ASSERT_EQ(loss_calls, 2);
    EXPECT_TRUE(filter.getState().p_w.isApprox(state_before.p_w, 1e-12));
    EXPECT_TRUE(filter.getState().R_wb.matrix().isApprox(state_before.R_wb.matrix(), 1e-12));
    EXPECT_TRUE(filter.getCovariance().isApprox(covariance_before, 1e-12));
}

TEST(IESKFTest, ConfigurationControlsMeasurementWeightAndProcessNoise) {
    const auto run_measurement_update = [](double lidar_information) {
        IESKF filter;
        Config config;
        config.lidar_cov_inv = lidar_information;
        filter.configure(config);
        filter.setMeasurementCallback([](const State15&, SharedState15& shared) {
            shared.reset(1);
            shared.H(0, 3) = 1.0;
            shared.b(0) = 1.0;
            shared.num_measurements = 1;
            shared.valid = true;
            return true;
        });
        filter.update(1);
        return filter.getState().p_w.x();
    };

    const double weak_correction = run_measurement_update(1.0);
    const double strong_correction = run_measurement_update(10000.0);
    EXPECT_LT(strong_correction, weak_correction);
    EXPECT_GT(std::abs(strong_correction), 50.0 * std::abs(weak_correction));

    IESKF quiet_filter;
    Config quiet_config;
    quiet_config.na = 0.0;
    quiet_config.ng = 0.0;
    quiet_config.nba = 0.0;
    quiet_config.nbg = 0.0;
    quiet_filter.configure(quiet_config);

    IESKF noisy_filter;
    Config noisy_config = quiet_config;
    noisy_config.na = 1.0;
    noisy_config.ng = 1.0;
    noisy_config.nba = 1.0;
    noisy_config.nbg = 1.0;
    noisy_filter.configure(noisy_config);

    const IMUData stationary(Eigen::Vector3d(0.0, 0.0, 9.81), Eigen::Vector3d::Zero(), 0.01);
    quiet_filter.predict(stationary, 0.01);
    noisy_filter.predict(stationary, 0.01);
    EXPECT_GT(noisy_filter.getCovariance().trace(), quiet_filter.getCovariance().trace() + 0.03);
}

TEST(IESKFTest, GravityAlignmentHandlesAntiParallelAndDegenerateInputs) {
    IESKF filter;
    filter.initWithGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    const Eigen::Vector3d aligned = filter.getState().R_wb * Eigen::Vector3d(0.0, 0.0, -1.0);
    EXPECT_TRUE(aligned.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));

    const Eigen::Matrix3d attitude_before_invalid_input = filter.getState().R_wb.matrix();
    filter.initWithGravity(Eigen::Vector3d::Zero());
    EXPECT_TRUE(filter.getState().R_wb.matrix().allFinite());
    EXPECT_TRUE(filter.getState().R_wb.matrix().isApprox(attitude_before_invalid_input, 1e-12));
}

TEST(EigenSE3Test, ExponentialMapPreservesTinyRotation) {
    constexpr double kTinyAngle = 1e-8;
    const SO3d rotation = SO3d::exp(Eigen::Vector3d(0.0, 0.0, kTinyAngle));
    const Eigen::Vector3d rotated_x = rotation * Eigen::Vector3d::UnitX();

    EXPECT_NEAR(rotated_x.x(), 1.0, 1e-15);
    EXPECT_NEAR(rotated_x.y(), kTinyAngle, 1e-15);
    EXPECT_NEAR(rotated_x.z(), 0.0, 1e-15);
}

TEST(IESKFTest, StateUpdateAppliesRotationCorrection) {
    State15 state;
    V15D delta = V15D::Zero();
    delta.z() = 0.1;

    state.update(delta);

    const Eigen::AngleAxisd rotation(state.R_wb.matrix());
    EXPECT_NEAR(rotation.angle(), 0.1, 1e-9);
    EXPECT_NEAR(rotation.axis().z(), 1.0, 1e-9);
}

TEST(IMUProcessorTest, PropagatesAcrossConsecutiveZeroDurationSimScans) {
    Config config;
    config.imu_init_num = 1;
    config.imu_init_gyro_rms_max = 2.0;
    IMUProcessor processor(config);
    auto filter = std::make_shared<IESKF>();

    const Eigen::Vector3d acceleration(0.0, 0.0, 9.81);
    const Eigen::Vector3d angular_rate(0.0, 0.0, 1.0);
    const std::deque<IMUData> initialization{{acceleration, angular_rate, 0.90}};
    ASSERT_TRUE(processor.initialize(initialization));

    // The first scan defines the filter epoch; start and end are both 1.0 s in
    // simulation when per-point times are unavailable.
    processor.propagate(filter, initialization, 0.0, 1.0);

    const std::deque<IMUData> next_scan{{acceleration, angular_rate, 1.02},
                                        {acceleration, angular_rate, 1.08}};
    processor.propagate(filter, next_scan, 1.0, 1.10);

    const Eigen::AngleAxisd rotation(filter->getState().R_wb.matrix());
    EXPECT_NEAR(rotation.angle(), 0.10, 1e-3);
    EXPECT_NEAR(rotation.axis().z(), 1.0, 1e-6);
}

TEST(IMUProcessorTest, PropagatesAtMaximumSupportedInterval) {
    Config config;
    config.imu_init_num = 1;
    config.imu_init_gyro_rms_max = 2.0;
    IMUProcessor processor(config);
    auto filter = std::make_shared<IESKF>();

    const std::deque<IMUData> initialization{
        {Eigen::Vector3d(0.0, 0.0, 9.81), Eigen::Vector3d(0.0, 0.0, 1.0), 0.0}};
    ASSERT_TRUE(processor.initialize(initialization));
    processor.propagate(filter, initialization, 0.0, 0.0);
    processor.propagate(filter, {}, 0.0, 0.1);

    const Eigen::AngleAxisd rotation(filter->getState().R_wb.matrix());
    EXPECT_NEAR(rotation.angle(), 0.1, 1e-9);
    EXPECT_NEAR(rotation.axis().z(), 1.0, 1e-9);
}

TEST(IMUProcessorTest, WaitsForStationaryWindowAndNormalizesAccelerationMagnitude) {
    Config config;
    config.imu_init_num = 4;
    config.imu_init_accel_std_max = 0.2;
    config.imu_init_gyro_rms_max = 0.1;
    config.imu_init_gravity_tolerance = 3.0;
    IMUProcessor processor(config);

    const std::deque<IMUData> moving_window{
        {Eigen::Vector3d(0.0, 0.0, 8.0), Eigen::Vector3d(0.0, 0.0, 0.3), 0.01},
        {Eigen::Vector3d(0.0, 0.0, 12.0), Eigen::Vector3d(0.0, 0.0, 0.3), 0.02},
        {Eigen::Vector3d(0.0, 0.0, 8.0), Eigen::Vector3d(0.0, 0.0, 0.3), 0.03},
        {Eigen::Vector3d(0.0, 0.0, 12.0), Eigen::Vector3d(0.0, 0.0, 0.3), 0.04}};
    EXPECT_FALSE(processor.initialize(moving_window));

    constexpr double kMeasuredGravity = 11.3659;
    const std::deque<IMUData> stationary_window{
        {Eigen::Vector3d(0.0, 0.0, kMeasuredGravity), Eigen::Vector3d::Zero(), 0.05},
        {Eigen::Vector3d(0.0, 0.0, kMeasuredGravity), Eigen::Vector3d::Zero(), 0.06},
        {Eigen::Vector3d(0.0, 0.0, kMeasuredGravity), Eigen::Vector3d::Zero(), 0.07},
        {Eigen::Vector3d(0.0, 0.0, kMeasuredGravity), Eigen::Vector3d::Zero(), 0.08}};
    ASSERT_TRUE(processor.initialize(stationary_window));
    EXPECT_NEAR(processor.getAccelScale(), 9.80665 / kMeasuredGravity, 1e-12);

    auto filter = std::make_shared<IESKF>();
    filter->initWithGravity(processor.getMeanAcc());
    processor.propagate(filter, stationary_window, 0.0, 0.10);
    const std::deque<IMUData> next_scan{
        {Eigen::Vector3d(0.0, 0.0, kMeasuredGravity), Eigen::Vector3d::Zero(), 0.12},
        {Eigen::Vector3d(0.0, 0.0, kMeasuredGravity), Eigen::Vector3d::Zero(), 0.18}};
    processor.propagate(filter, next_scan, 0.10, 0.20);
    EXPECT_NEAR(filter->getState().v_w.norm(), 0.0, 1e-9);
}

TEST(IMUProcessorTest, IgnoresNonFiniteInitializationSamples) {
    Config config;
    config.imu_init_num = 2;
    IMUProcessor processor(config);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const std::deque<IMUData> invalid{
        {Eigen::Vector3d(nan, 0.0, 9.81), Eigen::Vector3d::Zero(), 0.01},
        {Eigen::Vector3d(0.0, 0.0, 9.81), Eigen::Vector3d::Zero(), nan}};
    EXPECT_FALSE(processor.initialize(invalid));

    const std::deque<IMUData> valid{
        {Eigen::Vector3d(0.0, 0.0, 9.81), Eigen::Vector3d::Zero(), 0.02},
        {Eigen::Vector3d(0.0, 0.0, 9.81), Eigen::Vector3d::Zero(), 0.03}};
    EXPECT_TRUE(processor.initialize(valid));
}

TEST(UtilsTest, MapsFilterCovarianceIntoRosPoseOrder) {
    Eigen::Matrix<double, 15, 15> state_covariance = Eigen::Matrix<double, 15, 15>::Zero();
    state_covariance(3, 3) = 1.0;
    state_covariance(4, 4) = 2.0;
    state_covariance(5, 5) = 3.0;
    state_covariance(0, 0) = 4.0;
    state_covariance(1, 1) = 5.0;
    state_covariance(2, 2) = 6.0;
    state_covariance(3, 0) = 0.7;
    state_covariance(0, 3) = 0.7;

    nav_msgs::msg::Odometry odom;
    utils::setOdometryPoseCovariance(state_covariance, odom);

    EXPECT_DOUBLE_EQ(odom.pose.covariance[0], 1.0);
    EXPECT_DOUBLE_EQ(odom.pose.covariance[7], 2.0);
    EXPECT_DOUBLE_EQ(odom.pose.covariance[14], 3.0);
    EXPECT_DOUBLE_EQ(odom.pose.covariance[21], 4.0);
    EXPECT_DOUBLE_EQ(odom.pose.covariance[28], 5.0);
    EXPECT_DOUBLE_EQ(odom.pose.covariance[35], 6.0);
    EXPECT_DOUBLE_EQ(odom.pose.covariance[3], 0.7);
    EXPECT_DOUBLE_EQ(odom.pose.covariance[18], 0.7);
}

TEST(UtilsTest, TrimDequeFrontRetainsNewestEntries) {
    std::deque<int> buffer{0, 1, 2, 3, 4};

    EXPECT_EQ(utils::trimDequeFront(buffer, 3), 2U);
    EXPECT_EQ(buffer, (std::deque<int>{2, 3, 4}));
    EXPECT_EQ(utils::trimDequeFront(buffer, 3), 0U);
    EXPECT_EQ(utils::trimDequeFront(buffer, 0), 3U);
    EXPECT_TRUE(buffer.empty());
}

TEST(UtilsTest, BoundedPathRetainsNewestPoses) {
    nav_msgs::msg::Path path;
    for (int i = 0; i < 5; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp.sec = i;
        pose.pose.position.x = static_cast<double>(i);
        utils::appendBoundedPose(path, pose, 3);
    }

    ASSERT_EQ(path.poses.size(), 3U);
    EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 2.0);
    EXPECT_DOUBLE_EQ(path.poses[1].pose.position.x, 3.0);
    EXPECT_DOUBLE_EQ(path.poses[2].pose.position.x, 4.0);
    EXPECT_EQ(path.header.stamp.sec, 4);
}

TEST(IKDTreeBackendTest, SizeTracksIncrementalUpdatesAndDeletion) {
    auto tree = MapTreeInterface::createIKDTree();

    CloudType::Ptr initial(new CloudType);
    for (const float x : {0.0f, 2.0f}) {
        PointType point;
        point.x = x;
        point.y = 0.0f;
        point.z = 0.0f;
        initial->push_back(point);
    }
    tree->build(initial);

    PointType added_point;
    added_point.x = 4.0f;
    added_point.y = 0.0f;
    added_point.z = 0.0f;
    PointVec added{added_point};
    tree->addPoints(added, false);

    EXPECT_EQ(tree->size(), 3U);

    // Verify remaining points via nearest-neighbor queries.
    auto has_neighbor_at = [&tree](float x) {
        PointType query;
        query.x = x;
        query.y = 0.0f;
        query.z = 0.0f;
        PointVec neighbors;
        std::vector<float> distances;
        return tree->nearestKSearchPoints(query, 1, neighbors, distances) > 0 &&
               distances.front() < 0.01f;
    };
    EXPECT_TRUE(has_neighbor_at(0.0f));
    EXPECT_TRUE(has_neighbor_at(2.0f));
    EXPECT_TRUE(has_neighbor_at(4.0f));

    BoxPointType removal{};
    removal.vertex_min[0] = -0.1f;
    removal.vertex_max[0] = 0.1f;
    removal.vertex_min[1] = -0.1f;
    removal.vertex_max[1] = 0.1f;
    removal.vertex_min[2] = -0.1f;
    removal.vertex_max[2] = 0.1f;
    tree->deletePoints({removal});

    EXPECT_EQ(tree->size(), 2U);
    EXPECT_FALSE(has_neighbor_at(0.0f));
    EXPECT_TRUE(has_neighbor_at(2.0f));
    EXPECT_TRUE(has_neighbor_at(4.0f));
}

TEST(IKDTreeBackendTest, VoxelCenterInsertionKeepsClosestRepresentative) {
    auto tree = MapTreeInterface::createIKDTree();
    tree->setDownsampleParam(1.0f);

    // Voxel [0, 1) has center 0.5.
    auto makePoint = [](float x) {
        PointType p;
        p.x = x;
        p.y = 0.0f;
        p.z = 0.0f;
        return p;
    };

    // ikd-Tree requires an initial Build before Add_Points can be used.
    CloudType::Ptr initial(new CloudType);
    initial->push_back(makePoint(0.4f));
    tree->build(initial);

    auto nearestTo = [&tree, &makePoint](float x) {
        PointVec neighbors;
        std::vector<float> distances;
        tree->nearestKSearchPoints(makePoint(x), 1, neighbors, distances);
        return neighbors.empty() ? std::numeric_limits<float>::infinity()
                                 : neighbors.front().x;
    };
    EXPECT_NEAR(nearestTo(0.5f), 0.4f, 1e-3f);

    // A farther point is rejected.
    PointVec second{makePoint(0.1f)};
    EXPECT_EQ(tree->addPoints(second, true), 0U);
    EXPECT_EQ(tree->size(), 1U);

    // A closer point replaces the representative.
    PointVec third{makePoint(0.55f)};
    EXPECT_EQ(tree->addPoints(third, true), 1U);
    EXPECT_EQ(tree->size(), 1U);
    EXPECT_NEAR(nearestTo(0.5f), 0.55f, 1e-3f);

    // A point in a new voxel is inserted without downsampling.
    PointVec fourth{makePoint(1.6f)};
    EXPECT_EQ(tree->addPoints(fourth, true), 1U);
    EXPECT_EQ(tree->size(), 2U);
    EXPECT_NEAR(nearestTo(1.5f), 1.6f, 1e-3f);
}

TEST(LocalMapTest, SlidingCubeRemovesOnlyDepartedSlab) {
    LocalMap local_map;
    local_map.initialized = true;
    for (int axis = 0; axis < 3; ++axis) {
        local_map.corner.vertex_min[axis] = -25.0f;
        local_map.corner.vertex_max[axis] = 25.0f;
    }

    local_map.update(Eigen::Vector3d(6.0, 0.0, 0.0), 50.0, 0.5, 40.0);

    ASSERT_EQ(local_map.boxes_to_remove.size(), 1U);
    const BoxPointType& slab = local_map.boxes_to_remove.front();
    EXPECT_FLOAT_EQ(slab.vertex_min[0], -25.0f);
    EXPECT_LT(slab.vertex_max[0], -19.0f);
    EXPECT_FLOAT_EQ(slab.vertex_min[1], -25.0f);
    EXPECT_FLOAT_EQ(slab.vertex_max[1], 25.0f);
    EXPECT_FLOAT_EQ(local_map.corner.vertex_min[0], -19.0f);
    EXPECT_FLOAT_EQ(local_map.corner.vertex_max[0], 31.0f);
}

TEST(LidarProcessorTest, InitialMapAndPublishedCloudUseWorldFrame) {
    Config config;
    config.scan_resolution = 0.0;
    config.lidar_min_range = 0.0;
    config.lidar_max_range = 100.0;
    config.cube_len = 20.0;

    auto filter = std::make_shared<IESKF>();
    State15 state;
    state.R_wb = SO3d::exp(Eigen::Vector3d(0.0, 0.0, std::acos(-1.0) / 2.0));
    state.p_w = Eigen::Vector3d(1.0, 2.0, 3.0);
    state.T_I_L = SE3d(SO3d(), Eigen::Vector3d(0.5, 0.0, 0.0));
    filter->setState(state);

    auto tree = MapTreeInterface::createIKDTree();
    LidarProcessor processor(config, filter, tree);
    SyncPackage package;
    PointType point;
    point.x = 1.0f;
    point.y = 0.0f;
    point.z = 0.0f;
    package.cloud->push_back(point);

    processor.process(package);

    ASSERT_EQ(processor.getWorldCloud()->size(), 1U);
    const PointType& transformed = processor.getWorldCloud()->front();
    EXPECT_NEAR(transformed.x, 1.0, 1e-6);
    EXPECT_NEAR(transformed.y, 3.5, 1e-6);
    EXPECT_NEAR(transformed.z, 3.0, 1e-6);

    EXPECT_EQ(tree->size(), 1U);
    PointType query;
    query.x = static_cast<float>(transformed.x);
    query.y = static_cast<float>(transformed.y);
    query.z = static_cast<float>(transformed.z);
    PointVec neighbors;
    std::vector<float> distances;
    ASSERT_EQ(tree->nearestKSearchPoints(query, 1, neighbors, distances), 1U);
    EXPECT_NEAR(distances.front(), 0.0f, 1e-3f);
}

TEST(MapBuilderTest, GravityInitializationSurvivesExtrinsicAssignment) {
    Config config;
    config.imu_init_num = 2;
    config.gravity_align = true;
    config.t_I_L = Eigen::Vector3d(0.1, -0.2, 0.3);
    config.R_I_L = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    auto filter = std::make_shared<IESKF>();
    MapBuilder builder(config, filter);

    SyncPackage package;
    package.cloud_start_time = 1.0;
    package.cloud_end_time = 1.0;
    package.imus.emplace_back(Eigen::Vector3d(0.0, 9.81, 0.0), Eigen::Vector3d::Zero(), 0.99);
    package.imus.emplace_back(Eigen::Vector3d(0.0, 9.81, 0.0), Eigen::Vector3d::Zero(), 1.00);

    builder.process(package);

    ASSERT_EQ(builder.status(), BuilderStatus::MAPPING);
    const State15& state = filter->getState();
    const Eigen::Vector3d aligned_acceleration = state.R_wb * Eigen::Vector3d(0.0, 1.0, 0.0);
    EXPECT_NEAR(aligned_acceleration.x(), 0.0, 1e-9);
    EXPECT_NEAR(aligned_acceleration.y(), 0.0, 1e-9);
    EXPECT_NEAR(aligned_acceleration.z(), 1.0, 1e-9);
    EXPECT_TRUE(state.T_I_L.translation().isApprox(config.t_I_L, 1e-12));
    EXPECT_TRUE(state.T_I_L.rotation().matrix().isApprox(config.R_I_L, 1e-12));
}

}  // namespace
}  // namespace fast_lio
