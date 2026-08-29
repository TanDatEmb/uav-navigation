#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <limits>
#include <numbers>

#include "fast_lio_core/deskew/scan_deskewer.hpp"
#include "fast_lio_core/estimation/imu_trajectory.hpp"
#include "fast_lio_core/geometry/frame_ids.hpp"

namespace uav::nav::lio {
namespace {

LidarScan simultaneousScan() {
  LidarScan scan;
  scan.start_time = Timestamp(100, ClockDomain::kSimulationTime);
  scan.end_time = Timestamp(101, ClockDomain::kSimulationTime);
  scan.has_per_point_time = false;
  scan.points = {LidarPoint{Eigen::Vector3f(1.0F, 2.0F, 3.0F), 0, 1, 2, 3},
                 LidarPoint{Eigen::Vector3f(4.0F, 5.0F, 6.0F), 0, 4, 5, 6}};
  return scan;
}

TEST(ScanDeskewerTest, SimModeExplicitlyBypassesWithoutFakeTime) {
  const LidarScan scan = simultaneousScan();
  ScanDeskewer deskewer(
      ScanDeskewerConfig{DeskewMode::kSimultaneousScan, DeskewReference::kScanEnd});
  const RigidTransform T_imu_lidar(imuFrame(), lidarFrame(), Eigen::Quaterniond::Identity(),
                                   Eigen::Vector3d::Zero());
  const auto result = deskewer.deskew(scan, ImuTrajectory(), T_imu_lidar);
  ASSERT_TRUE(result.ok()) << result.status().message();
  EXPECT_EQ(result.value().status, DeskewStatus::kBypassedSimultaneousScan);
  EXPECT_FALSE(result.value().deskew_applied);
  ASSERT_EQ(result.value().scan.points.size(), scan.points.size());
  for (std::size_t index = 0; index < scan.points.size(); ++index) {
    EXPECT_EQ(result.value().scan.points[index].relative_time_ns, 0U);
    EXPECT_TRUE(result.value().scan.points[index].position_lidar_m.isApprox(
        scan.points[index].position_lidar_m));
  }
}

TEST(ScanDeskewerTest, RejectsTrajectoryStateWhoseQuaternionNormOverflows) {
  ImuTrajectory trajectory;
  const double huge = std::numeric_limits<double>::max();
  const Status status = trajectory.addState(ImuTrajectoryState{
      Timestamp(1, ClockDomain::kSensorTime), Eigen::Quaterniond(huge, huge, huge, huge),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
  EXPECT_EQ(status.code(), StatusCode::kNumericalFailure);
}

TEST(ScanDeskewerTest, RealPerPointTranslationAlignsStaticWorldPoint) {
  ImuTrajectory trajectory;
  ASSERT_TRUE(trajectory
                  .addState(ImuTrajectoryState{
                      Timestamp(1, ClockDomain::kSensorTime), Eigen::Quaterniond::Identity(),
                      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)})
                  .ok());
  ASSERT_TRUE(trajectory
                  .addState(ImuTrajectoryState{Timestamp(1'000'000'001, ClockDomain::kSensorTime),
                                               Eigen::Quaterniond::Identity(),
                                               Eigen::Vector3d(1.0, 0.0, 0.0),
                                               Eigen::Vector3d(1.0, 0.0, 0.0)})
                  .ok());

  LidarScan scan;
  scan.start_time = Timestamp(1, ClockDomain::kSensorTime);
  scan.end_time = Timestamp(1'000'000'001, ClockDomain::kSensorTime);
  scan.has_per_point_time = true;
  // A static world point at x=10 is observed at x=10 at scan start and x=9
  // at scan end while the sensor translates +1 m.
  scan.points = {LidarPoint{Eigen::Vector3f(10.0F, 0.0F, 0.0F), 0, 0, 0, 0},
                 LidarPoint{Eigen::Vector3f(9.5F, 0.0F, 0.0F), 500'000'000, 0, 0, 0},
                 LidarPoint{Eigen::Vector3f(9.0F, 0.0F, 0.0F), 1'000'000'000, 0, 0, 0}};
  const RigidTransform T_imu_lidar(imuFrame(), lidarFrame(), Eigen::Quaterniond::Identity(),
                                   Eigen::Vector3d::Zero());
  ScanDeskewer deskewer(ScanDeskewerConfig{DeskewMode::kPerPoint, DeskewReference::kScanEnd});
  const auto result = deskewer.deskew(scan, trajectory, T_imu_lidar);
  ASSERT_TRUE(result.ok()) << result.status().message();
  EXPECT_EQ(result.value().status, DeskewStatus::kApplied);
  EXPECT_TRUE(result.value().deskew_applied);
  for (const auto& point : result.value().scan.points) {
    EXPECT_TRUE(point.position_lidar_m.isApprox(Eigen::Vector3f(9.0F, 0.0F, 0.0F), 1e-5F));
  }
}

TEST(ScanDeskewerTest, RealPerPointRotationAlignsStaticWorldPoint) {
  ImuTrajectory trajectory;
  ASSERT_TRUE(trajectory
                  .addState(ImuTrajectoryState{Timestamp(1, ClockDomain::kSensorTime),
                                               Eigen::Quaterniond::Identity(),
                                               Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()})
                  .ok());
  ASSERT_TRUE(
      trajectory
          .addState(ImuTrajectoryState{Timestamp(1'001, ClockDomain::kSensorTime),
                                       Eigen::Quaterniond(Eigen::AngleAxisd(
                                           std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ())),
                                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()})
          .ok());
  LidarScan scan;
  scan.start_time = Timestamp(1, ClockDomain::kSensorTime);
  scan.end_time = Timestamp(1'001, ClockDomain::kSensorTime);
  scan.has_per_point_time = true;
  // World point (1,0,0) appears as +x initially and -y after +90 deg yaw.
  scan.points = {LidarPoint{Eigen::Vector3f::UnitX(), 0, 0, 0, 0},
                 LidarPoint{-Eigen::Vector3f::UnitY(), 1'000, 0, 0, 0}};
  const RigidTransform T_imu_lidar(imuFrame(), lidarFrame(), Eigen::Quaterniond::Identity(),
                                   Eigen::Vector3d::Zero());
  ScanDeskewer deskewer(ScanDeskewerConfig{DeskewMode::kPerPoint, DeskewReference::kScanEnd});
  const auto result = deskewer.deskew(scan, trajectory, T_imu_lidar);
  ASSERT_TRUE(result.ok()) << result.status().message();
  for (const auto& point : result.value().scan.points) {
    EXPECT_TRUE(point.position_lidar_m.isApprox(-Eigen::Vector3f::UnitY(), 1e-5F));
  }
}

TEST(ScanDeskewerTest, RealModeRejectsMissingPerPointTiming) {
  ScanDeskewer deskewer(ScanDeskewerConfig{DeskewMode::kPerPoint, DeskewReference::kScanEnd});
  const RigidTransform T_imu_lidar(imuFrame(), lidarFrame(), Eigen::Quaterniond::Identity(),
                                   Eigen::Vector3d::Zero());
  const auto result = deskewer.deskew(simultaneousScan(), ImuTrajectory(), T_imu_lidar);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), StatusCode::kDeskewRejected);
}

}  // namespace
}  // namespace uav::nav::lio
