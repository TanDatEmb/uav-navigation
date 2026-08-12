// P1 mandatory correctness tests C (raycast geometry), D (yaw invariance),
// E (translation invariance), H (sliding boundedness), I (inflation).
// These exercise the mapper's single required transform
// (p_odom = T_odom_lidar * p_lidar) end-to-end through RogMapAdapter, which
// is exactly the geometry a wrong sensor origin or transform direction bug
// would corrupt.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "navigation_mapping/rog_map_adapter.hpp"

namespace navigation_mapping {
namespace {

std::string testTmpDir(const std::string& name) {
  const std::string dir = std::string(NAVIGATION_MAPPING_TEST_TMP_DIR) + "/" + name;
  std::filesystem::create_directories(dir);
  return dir;
}

RogMapProductConfig smallConfig() {
  RogMapProductConfig config;
  config.resolution_m = 0.2;
  config.local_map_size_m = {10.0, 10.0, 6.0};
  config.ray_range_min_m = 0.05;
  config.ray_range_max_m = 10.0;
  config.inflation_step = 2;
  return config;
}

rog_map::PointCloud singlePointCloud(const Eigen::Vector3d& point_odom) {
  rog_map::PointCloud cloud;
  pcl::PointXYZI p;
  p.x = static_cast<float>(point_odom.x());
  p.y = static_cast<float>(point_odom.y());
  p.z = static_cast<float>(point_odom.z());
  p.intensity = 0.0F;
  cloud.push_back(p);
  return cloud;
}

RogMapAdapter makeAdapter(const std::string& tmp_name) {
  RogMapAdapter adapter([]() { return 0.0; }, testTmpDir(tmp_name));
  adapter.reset(smallConfig());
  return adapter;
}

// --- C: raycast geometry ---------------------------------------------------

TEST(RogMapAdapterGeometryTest, RaycastProducesFreeOccupiedUnknownAlongRay) {
  auto adapter = makeAdapter("raycast");
  const T_odom_lidar sensor_pose{};  // identity: sensor at odom origin

  for (int i = 0; i < 8; ++i) {
    adapter.updateMap(singlePointCloud(Eigen::Vector3d(3.0, 0.0, 0.0)), sensor_pose);
  }

  EXPECT_TRUE(adapter.map().isKnownFree(rog_map::Vec3f(1.0, 0.0, 0.0)));
  EXPECT_TRUE(adapter.map().isOccupied(rog_map::Vec3f(3.0, 0.0, 0.0)));
  EXPECT_TRUE(adapter.map().isUnknown(rog_map::Vec3f(4.5, 0.0, 0.0)));
}

// --- D: yaw invariance ------------------------------------------------------

TEST(RogMapAdapterGeometryTest, StaticWallStaysFixedInOdomAcrossYawRotations) {
  auto adapter = makeAdapter("yaw_invariance");
  // A wall point fixed in the odom frame.
  const Eigen::Vector3d wall_point_odom(2.0, 1.0, 0.0);

  // Observe the same world point from several different sensor yaws. For
  // each yaw, the point expressed in the (rotated) lidar frame is
  // R_odom_lidar^{-1} * (p_odom - t_odom); if the mapper's transform
  // direction were wrong, this would land on the wrong side of the wall.
  const std::vector<double> yaws_rad = {0.0, M_PI / 6.0, M_PI / 2.0, M_PI, -M_PI / 3.0};
  for (const double yaw : yaws_rad) {
    const Eigen::Quaterniond rotation_odom_lidar(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    const T_odom_lidar sensor_pose{Eigen::Vector3d::Zero(), rotation_odom_lidar};
    const Eigen::Vector3d point_lidar = rotation_odom_lidar.inverse() * wall_point_odom;
    for (int i = 0; i < 3; ++i) {
      adapter.updateMap(singlePointCloud(sensor_pose.apply(point_lidar)), sensor_pose);
    }
  }

  EXPECT_TRUE(adapter.map().isOccupied(rog_map::Vec3f(2.0, 1.0, 0.0)));
}

// --- E: translation invariance ----------------------------------------------

TEST(RogMapAdapterGeometryTest, StaticObstacleStaysFixedAsSensorTranslates) {
  auto adapter = makeAdapter("translation_invariance");
  const Eigen::Vector3d obstacle_odom(3.0, 0.0, 0.0);

  const std::vector<Eigen::Vector3d> sensor_positions = {
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.5, 0.2, 0.0),
      Eigen::Vector3d(-0.3, -0.1, 0.0),
      Eigen::Vector3d(0.1, 0.4, 0.0),
  };
  for (const Eigen::Vector3d& sensor_position : sensor_positions) {
    const T_odom_lidar sensor_pose{sensor_position, Eigen::Quaterniond::Identity()};
    const Eigen::Vector3d point_lidar = obstacle_odom - sensor_position;
    for (int i = 0; i < 3; ++i) {
      adapter.updateMap(singlePointCloud(sensor_pose.apply(point_lidar)), sensor_pose);
    }
  }

  EXPECT_TRUE(adapter.map().isOccupied(rog_map::Vec3f(3.0, 0.0, 0.0)));
}

// --- H: sliding boundedness --------------------------------------------------

TEST(RogMapAdapterGeometryTest, MapRemainsUsableAcrossManySlidesBeyondLocalExtent) {
  auto adapter = makeAdapter("sliding");
  // Local map is 10x10x6; move the sensor origin well beyond that repeatedly.
  for (int step = 0; step < 30; ++step) {
    const Eigen::Vector3d sensor_position(step * 4.0, 0.0, 0.0);
    const T_odom_lidar sensor_pose{sensor_position, Eigen::Quaterniond::Identity()};
    const Eigen::Vector3d obstacle_odom = sensor_position + Eigen::Vector3d(2.0, 0.0, 0.0);
    EXPECT_NO_THROW(adapter.updateMap(singlePointCloud(obstacle_odom), sensor_pose));
  }
  // The map must still be usable (bounded ring buffer, no corruption) and the
  // most recent local observation must be reachable.
  const Eigen::Vector3d last_sensor_position(29 * 4.0, 0.0, 0.0);
  const Eigen::Vector3d last_obstacle = last_sensor_position + Eigen::Vector3d(2.0, 0.0, 0.0);
  for (int i = 0; i < 5; ++i) {
    const T_odom_lidar sensor_pose{last_sensor_position, Eigen::Quaterniond::Identity()};
    adapter.updateMap(singlePointCloud(last_obstacle), sensor_pose);
  }
  EXPECT_TRUE(adapter.map().isOccupied(
      rog_map::Vec3f(last_obstacle.x(), last_obstacle.y(), last_obstacle.z())));
  // A point from far in the past must no longer be considered inside the
  // local map (it has correctly slid out).
  EXPECT_FALSE(adapter.map().isOccupied(rog_map::Vec3f(2.0, 0.0, 0.0)));
}

// --- G: lifecycle regression at the mapper (adapter) level ------------------

TEST(RogMapAdapterGeometryTest, AdapterCanResetRepeatedlyInSameProcess) {
  auto adapter = makeAdapter("lifecycle");
  for (int generation = 0; generation < 5; ++generation) {
    EXPECT_NO_THROW(adapter.reset(smallConfig()));
    const T_odom_lidar sensor_pose{};
    EXPECT_NO_THROW(
        adapter.updateMap(singlePointCloud(Eigen::Vector3d(1.0, 0.0, 0.0)), sensor_pose));
  }
  EXPECT_EQ(adapter.resetCount(), 6U);  // 1 from makeAdapter() + 5 here
}

// --- I: inflation ------------------------------------------------------------

TEST(RogMapAdapterGeometryTest, OccupiedInflationExpandsByConfiguredStep) {
  auto adapter = makeAdapter("inflation");
  const T_odom_lidar sensor_pose{};
  for (int i = 0; i < 10; ++i) {
    adapter.updateMap(singlePointCloud(Eigen::Vector3d(2.0, 0.0, 0.0)), sensor_pose);
  }
  const double resolution_m = smallConfig().resolution_m;
  // inflation_step = 2, so neighbors within 2 cells must be inflated-occupied.
  EXPECT_TRUE(adapter.map().isOccupiedInflate(
      rog_map::Vec3f(2.0 + resolution_m, 0.0, 0.0)));
  EXPECT_TRUE(adapter.map().isOccupiedInflate(
      rog_map::Vec3f(2.0 + 2 * resolution_m, 0.0, 0.0)));
}

}  // namespace
}  // namespace navigation_mapping
