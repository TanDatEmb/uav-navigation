#include <navigation_mapping/mapping_actor.hpp>
#include <navigation_mapping/mapping_observation.hpp>
#include <navigation_mapping/mapping_world_snapshot.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <type_traits>

#include <gtest/gtest.h>

namespace {

static_assert(!std::is_move_constructible_v<navigation_mapping::MappingActor>);
static_assert(!std::is_move_assignable_v<navigation_mapping::MappingActor>);

nav_msgs::msg::Odometry odometryAt(const std::int32_t seconds) {
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp.sec = seconds;
  odometry.header.stamp.nanosec = 0U;
  odometry.header.frame_id = "lio_odom";
  odometry.child_frame_id = "base_link";
  odometry.pose.pose.orientation.w = 1.0;
  return odometry;
}

nav_msgs::msg::Odometry odometryAtStamp(const std::int64_t stamp_ns) {
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  odometry.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  odometry.header.frame_id = "lio_odom";
  odometry.child_frame_id = "base_link";
  odometry.pose.pose.orientation.w = 1.0;
  return odometry;
}

navigation_mapping::MappingObservation observationAt(
    const std::int64_t stamp_ns, const std::int64_t odometry_stamp_ns) {
  auto cloud = std::make_unique<navigation_mapping::PointCloud>();
  cloud->push_back(navigation_mapping::PointXYZI{1.0F, 0.0F, 0.0F, 0.0F});
  auto odometry = odometryAtStamp(odometry_stamp_ns);
  return {std::move(cloud), std::move(odometry), 1U, 1U, stamp_ns, 0};
}

navigation_mapping::MappingObservation observationAtPose(
    const std::int64_t stamp_ns, const std::int64_t odometry_stamp_ns,
    const Eigen::Vector3d& position, const std::uint64_t scan_sequence) {
  auto cloud = std::make_unique<navigation_mapping::PointCloud>();
  cloud->push_back(navigation_mapping::PointXYZI{
      static_cast<float>(position.x() + 2.0), static_cast<float>(position.y()),
      static_cast<float>(position.z()), 0.0F});
  auto odometry = odometryAtStamp(odometry_stamp_ns);
  odometry.pose.pose.position.x = position.x();
  odometry.pose.pose.position.y = position.y();
  odometry.pose.pose.position.z = position.z();
  return {std::move(cloud), std::move(odometry), 1U, scan_sequence,
          stamp_ns, 0};
}

TEST(MappingActorContract, RejectsMismatchedObservationAndOdometryTime) {
  navigation_mapping::MappingActor actor(NAVIGATION_MAPPING_PLANNER_CONFIG_PATH);
  const auto initial = actor.initialSnapshot();
  ASSERT_TRUE(initial);
  EXPECT_TRUE(initial.view);
  EXPECT_GT(initial.metrics.bytes, 0U);
  EXPECT_THROW(actor.process(observationAt(2'000'000'000LL, 1'000'000'000LL)),
               std::invalid_argument);
}

TEST(MappingActorContract, RejectsNonFinitePoseBeforeBackendMutation) {
  navigation_mapping::MappingActor actor(NAVIGATION_MAPPING_PLANNER_CONFIG_PATH);
  auto observation = observationAt(1'000'000'000LL, 1'000'000'000LL);
  observation.corrected_odometry.pose.pose.position.x = NAN;
  EXPECT_THROW(actor.process(observation), std::invalid_argument);
}

TEST(MappingActorContract, RejectsNonMonotonicObservationTimestamp) {
  navigation_mapping::MappingActor actor(NAVIGATION_MAPPING_PLANNER_CONFIG_PATH);
  auto first = observationAt(1'000'000'000LL, 1'000'000'000LL);
  const auto result = actor.process(first);
  ASSERT_TRUE(result.snapshot);
  EXPECT_GT(result.snapshot_metrics.bytes, 0U);
  EXPECT_EQ(result.snapshot_metrics.bytes,
            result.snapshot_metrics.owned_bytes + result.snapshot_metrics.shared_metadata_bytes);

  auto second = observationAt(1'000'000'000LL, 1'000'000'000LL);
  EXPECT_THROW(actor.process(second), std::runtime_error);
}

TEST(MappingActorContract, ValidatesNewEpochBeforeResettingTheMap) {
  navigation_mapping::MappingActor actor(NAVIGATION_MAPPING_PLANNER_CONFIG_PATH);
  auto first = observationAt(1'000'000'000LL, 1'000'000'000LL);
  ASSERT_NO_THROW(actor.process(first));

  auto invalid_new_epoch = observationAt(2'000'000'000LL, 2'000'000'000LL);
  invalid_new_epoch.localization_epoch = 2U;
  invalid_new_epoch.corrected_odometry.pose.pose.orientation.w = NAN;
  EXPECT_THROW(actor.process(invalid_new_epoch), std::invalid_argument);

  auto next_old_epoch = observationAt(3'000'000'000LL, 3'000'000'000LL);
  next_old_epoch.scan_sequence = 2U;
  EXPECT_NO_THROW(actor.process(next_old_epoch));
}

TEST(MappingActorContract, CarriesFiniteNoReturnEndpointsIntoMapEvidence) {
  navigation_mapping::MappingActor actor(NAVIGATION_MAPPING_PLANNER_CONFIG_PATH);
  ASSERT_TRUE(actor.initialSnapshot());
  auto observation = observationAt(1'000'000'000LL, 1'000'000'000LL);
  auto free_space = std::make_unique<navigation_mapping::PointCloud>();
  free_space->push_back(
      navigation_mapping::PointXYZI{4.0F, 0.0F, 0.0F, 0.0F});
  observation.free_space_endpoints = std::move(free_space);

  const auto result = actor.process(observation);
  ASSERT_TRUE(result.snapshot);
  EXPECT_EQ(result.diagnostics.free_space_endpoint_count, 1U);
  EXPECT_EQ(result.diagnostics.free_space_processed_count, 1U);
}

TEST(MappingActorContract, RejectsNonFiniteNoReturnEndpointsBeforeMapMutation) {
  navigation_mapping::MappingActor actor(NAVIGATION_MAPPING_PLANNER_CONFIG_PATH);
  ASSERT_TRUE(actor.initialSnapshot());
  auto observation = observationAt(1'000'000'000LL, 1'000'000'000LL);
  auto free_space = std::make_unique<navigation_mapping::PointCloud>();
  free_space->push_back(
      navigation_mapping::PointXYZI{NAN, 0.0F, 0.0F, 0.0F});
  observation.free_space_endpoints = std::move(free_space);

  EXPECT_THROW(actor.process(observation), std::invalid_argument);
}

TEST(MappingActorContract, ReconstructsBackendForNewLocalizationEpoch) {
  navigation_mapping::MappingActor actor(NAVIGATION_MAPPING_PLANNER_CONFIG_PATH);
  ASSERT_TRUE(actor.process(observationAt(1'000'000'000LL, 1'000'000'000LL)).snapshot);

  auto next_epoch = observationAt(2'000'000'000LL, 2'000'000'000LL);
  next_epoch.localization_epoch = 2U;
  next_epoch.scan_sequence = 1U;
  const auto result = actor.process(next_epoch);

  ASSERT_TRUE(result.snapshot);
  EXPECT_EQ(result.localization_epoch, 2U);
  EXPECT_EQ(result.world_generation, 2U);
  EXPECT_EQ(result.world_revision, 1U);
}

TEST(MappingActorContract, FirstFrameClearRemainsKnownFreeNearFiniteMapCeiling) {
  navigation_mapping::MappingActor actor(NAVIGATION_MAPPING_PLANNER_CONFIG_PATH);
  ASSERT_TRUE(actor.initialSnapshot());

  auto cloud = std::make_unique<navigation_mapping::PointCloud>();
  cloud->push_back(navigation_mapping::PointXYZI{2.0F, 0.0F, 2.9F, 0.0F});
  auto odometry = odometryAt(1);
  odometry.pose.pose.position.z = 2.9;
  navigation_mapping::MappingObservation observation{
      std::move(cloud), std::move(odometry), 1U, 1U, 1'000'000'000LL, 0};

  const auto result = actor.process(observation);
  ASSERT_TRUE(result.snapshot);
  EXPECT_EQ(result.snapshot->classify(
                navigation_world_model::Point3{0.0, 0.0, 2.9},
                navigation_world_model::GridLayer::kEvidence),
            navigation_world_model::CellState::kKnownFree);
  EXPECT_EQ(result.snapshot->classify(
                navigation_world_model::Point3{0.0, 0.0, 1.5},
                navigation_world_model::GridLayer::kEvidence),
            navigation_world_model::CellState::kUnknown);
}

TEST(MappingActorContract, FirstFrameClearSurvivesBoundedSnapshotPatches) {
  navigation_mapping::MappingActor actor(NAVIGATION_MAPPING_PLANNER_CONFIG_PATH);
  ASSERT_TRUE(actor.initialSnapshot());
  const Eigen::Vector3d position{-0.3, -0.3, 2.9};
  const navigation_world_model::Point3 robot_point = position;

  for (std::int64_t seconds = 1; seconds <= 20; ++seconds) {
    const auto result = actor.process(observationAtPose(
        seconds * 1'000'000'000LL, seconds * 1'000'000'000LL, position,
        static_cast<std::uint64_t>(seconds)));
    ASSERT_TRUE(result.snapshot);
    EXPECT_EQ(result.snapshot->classify(
                  robot_point, navigation_world_model::GridLayer::kEvidence),
              navigation_world_model::CellState::kKnownFree)
        << "revision=" << result.world_revision;
  }
}

TEST(MappingActorContract, UsesBoundedPatchForSteadyStateMapUpdate) {
  navigation_mapping::MappingActor actor(NAVIGATION_MAPPING_PLANNER_CONFIG_PATH);
  ASSERT_TRUE(actor.initialSnapshot());

  auto first = observationAt(1'000'000'000LL, 1'000'000'000LL);
  const auto first_result = actor.process(first);
  ASSERT_TRUE(first_result.snapshot);

  auto second = observationAt(2'000'000'000LL, 2'000'000'000LL);
  second.scan_sequence = 2U;
  const auto second_result = actor.process(second);
  ASSERT_TRUE(second_result.snapshot);
  const auto patch_snapshot =
      std::dynamic_pointer_cast<const navigation_mapping::MappingWorldSnapshot>(
          second_result.snapshot);
  ASSERT_TRUE(patch_snapshot);
  EXPECT_EQ(patch_snapshot->patchDepth(), 1U);
  EXPECT_EQ(second_result.snapshot_export_mode,
            navigation_mapping::SnapshotExportMode::kPatch);
  EXPECT_EQ(second_result.snapshot_full_export_reason,
            navigation_mapping::SnapshotFullExportReason::kNone);
  EXPECT_EQ(second_result.snapshot_patch_depth, 1U);
  EXPECT_GT(second_result.snapshot_export_base_cells, 0U);
  EXPECT_GT(second_result.snapshot_export_inflated_cells, 0U);
  EXPECT_LT(second_result.snapshot_metrics.owned_bytes,
            first_result.snapshot_metrics.owned_bytes);

  const auto center = second_result.snapshot->geometry().local_center_m;
  EXPECT_EQ(second_result.snapshot->classify(
                center, navigation_world_model::GridLayer::kEvidence),
            first_result.snapshot->classify(
                center, navigation_world_model::GridLayer::kEvidence));

  std::size_t maximum_depth = patch_snapshot->patchDepth();
  for (std::int64_t seconds = 3; seconds <= 11; ++seconds) {
    auto observation = observationAt(seconds * 1'000'000'000LL,
                                     seconds * 1'000'000'000LL);
    observation.scan_sequence = static_cast<std::uint64_t>(seconds);
    const auto result = actor.process(observation);
    ASSERT_TRUE(result.snapshot);
    const auto bounded_snapshot =
        std::dynamic_pointer_cast<const navigation_mapping::MappingWorldSnapshot>(
            result.snapshot);
    ASSERT_TRUE(bounded_snapshot);
    maximum_depth = std::max(maximum_depth, bounded_snapshot->patchDepth());
  }
  EXPECT_LE(maximum_depth, 8U);
}

TEST(MappingActorContract, CoalescesSnapshotExportAcrossRecentMapUpdates) {
  navigation_mapping::MappingActor actor(NAVIGATION_MAPPING_PLANNER_CONFIG_PATH);
  ASSERT_TRUE(actor.initialSnapshot());

  const auto first = actor.process(observationAtPose(
      1'000'000'000LL, 1'000'000'000LL, Eigen::Vector3d::Zero(), 1U));
  ASSERT_TRUE(first.snapshot);

  const auto deferred = actor.process(observationAtPose(
      1'020'000'000LL, 1'020'000'000LL, Eigen::Vector3d::Zero(), 2U));
  EXPECT_FALSE(deferred.snapshot);
  EXPECT_EQ(deferred.world_revision, 2U);

  const auto published = actor.process(observationAtPose(
      1'060'000'000LL, 1'060'000'000LL, Eigen::Vector3d::Zero(), 3U));
  ASSERT_TRUE(published.snapshot);
  EXPECT_EQ(published.world_revision, 3U);
  EXPECT_EQ(published.snapshot->identity().observation_stamp_ns, 1'060'000'000LL);
  const auto patch_snapshot =
      std::dynamic_pointer_cast<const navigation_mapping::MappingWorldSnapshot>(
          published.snapshot);
  ASSERT_TRUE(patch_snapshot);
  EXPECT_EQ(patch_snapshot->patchDepth(), 1U);
  EXPECT_EQ(published.snapshot_export_mode,
            navigation_mapping::SnapshotExportMode::kPatch);
  EXPECT_EQ(deferred.snapshot_export_mode,
            navigation_mapping::SnapshotExportMode::kDeferred);
}

TEST(MappingActorContract, RejectsNonPositiveSnapshotPublicationPeriod) {
  EXPECT_THROW(
      navigation_mapping::MappingActor(
          NAVIGATION_MAPPING_PLANNER_CONFIG_PATH, {}, {}, 0.0),
      std::invalid_argument);
}

}  // namespace
