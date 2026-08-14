#include <gtest/gtest.h>

#include <filesystem>
#include <stdexcept>

#include "navigation_mapping/mapping_pipeline.hpp"
#include "navigation_mapping/world_model.hpp"
#include "navigation_planning/a_star.hpp"
#include "navigation_planning/planner.hpp"

namespace navigation_planning {
namespace {

navigation_mapping::MappingPipelineConfig config() {
  navigation_mapping::MappingPipelineConfig value;
  value.rog.resolution_m = 0.2;
  value.rog.inflation_resolution_m = 0.4;
  value.rog.local_map_size_m = {10.0, 10.0, 6.0};
  value.rog.ray_range_min_m = 0.1;
  value.rog.ray_range_max_m = 10.0;
  value.point_filter.voxel_size_m = 0.1;
  // Synthetic test geometry only; the product runtime has no authoritative
  // vehicle model yet and therefore leaves this contract unset.
  value.collision.vehicle_radius_m = 0.1;
  value.collision.safety_margin_m = 0.1;
  return value;
}

navigation_mapping::ObservationInput observation() {
  navigation_mapping::ObservationInput input;
  input.header_frame_id = "lio_odom";
  input.points_frame_id = "livox_frame";
  input.header_stamp.sec = 1;
  input.points_stamp = input.header_stamp;
  input.sensor_pose.orientation.w = 1.0;
  input.points_lidar_m = {navigation_mapping::Point3f{3.0, 0.0, 0.0}};
  input.public_frame_generation = 1;
  return input;
}

navigation_mapping::ObservationInput rayObservation(const navigation_mapping::Vec3& sensor,
                                                     const navigation_mapping::Vec3& endpoint,
                                                     std::int32_t stamp = 1) {
  navigation_mapping::ObservationInput input;
  input.header_frame_id = "lio_odom";
  input.points_frame_id = "livox_frame";
  input.header_stamp.sec = stamp;
  input.points_stamp = input.header_stamp;
  input.sensor_pose.position.x = sensor.x();
  input.sensor_pose.position.y = sensor.y();
  input.sensor_pose.position.z = sensor.z();
  input.sensor_pose.orientation.w = 1.0;
  input.points_lidar_m = {navigation_mapping::Point3f{
      endpoint.x() - sensor.x(), endpoint.y() - sensor.y(), endpoint.z() - sensor.z()}};
  input.public_frame_generation = 1;
  return input;
}

std::string testDirectory(const char* name) {
  const std::string directory = std::string(NAVIGATION_PLANNING_TEST_TMP_DIR) + "/" + name;
  std::filesystem::create_directories(directory);
  return directory;
}

TEST(RogWorldModelIntegrationTest, MappingObservationFeedsAStarThroughWorldModel) {
  const std::string directory =
      std::string(NAVIGATION_PLANNING_TEST_TMP_DIR) + "/rog_world_model";
  std::filesystem::create_directories(directory);
  navigation_mapping::MappingPipeline pipeline(config(), []() { return 0.0; }, directory);
  for (int i = 0; i < 8; ++i) pipeline.process(observation());

  ASSERT_EQ(pipeline.diagnostics().accepted_observation_count, 8U);
  navigation_mapping::WorldModel world(pipeline.adapter());
  EXPECT_EQ(world.generation(), 1U);
  EXPECT_DOUBLE_EQ(world.resolution(navigation_mapping::WorldLayer::Probability), 0.2);
  EXPECT_DOUBLE_EQ(world.resolution(navigation_mapping::WorldLayer::Inflated), 0.4);

  const auto start = navigation_mapping::GridIndex3{2, 0, 0};
  const auto goal = navigation_mapping::GridIndex3{12, 0, 0};
  EXPECT_EQ(world.cellState(navigation_mapping::WorldLayer::Probability, start),
            navigation_mapping::CellState::KnownFree);
  EXPECT_EQ(world.cellState(navigation_mapping::WorldLayer::Probability, goal),
            navigation_mapping::CellState::KnownFree);

  const auto result = AStar{}.search(
      world, SearchRequest{navigation_mapping::WorldLayer::Probability,
                           UnknownPolicy::TreatUnknownAsBlocked,
                           world.gridToWorld(navigation_mapping::WorldLayer::Probability, start),
                           world.gridToWorld(navigation_mapping::WorldLayer::Probability, goal)});
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.world_generation, 1U);
  EXPECT_EQ(result.path.front(), start);
  EXPECT_EQ(result.path.back(), goal);
  EXPECT_DOUBLE_EQ(result.statistics.path_length_m, 2.0);
  EXPECT_GT(result.statistics.cell_state_queries, 0U);
}

TEST(RogWorldModelIntegrationTest, WorldModelAndCurrentStateProducePlannerTrajectory) {
  auto planner_config = config();
  planner_config.rog.inflation_resolution_m = 0.2;
  navigation_mapping::MappingPipeline pipeline(
      planner_config, []() { return 0.0; }, testDirectory("rog_world_model_planner"));
  for (int i = 0; i < 8; ++i) pipeline.process(observation());

  navigation_mapping::WorldModel world(pipeline.adapter());
  const navigation_mapping::Vec3 start{0.8, 0.0, 0.0};
  const navigation_mapping::Vec3 goal{2.0, 0.0, 0.0};
  const auto result = Planner{}.plan(
      VehicleState{start, navigation_mapping::Vec3::Zero(), navigation_mapping::Vec3::Zero()},
      Goal{goal}, world);

  ASSERT_TRUE(result.success) << static_cast<int>(result.failure_code);
  EXPECT_TRUE(result.trajectory.finiteAndMonotonic());
  EXPECT_EQ(result.world_generation, world.generation());
  EXPECT_EQ(result.world_revision, world.revision());
  EXPECT_DOUBLE_EQ(world.clearanceRadius(), 0.2);
}

TEST(RogWorldModelIntegrationTest, InflatedLayerPreservesUnknownPolicyWithoutUnknownInflation) {
  const std::string directory =
      std::string(NAVIGATION_PLANNING_TEST_TMP_DIR) + "/rog_world_model_unknown";
  std::filesystem::create_directories(directory);
  navigation_mapping::MappingPipeline pipeline(config(), []() { return 0.0; }, directory);
  for (int i = 0; i < 8; ++i) pipeline.process(observation());

  navigation_mapping::WorldModel world(pipeline.adapter());
  const auto unknown = world.worldToGrid(navigation_mapping::WorldLayer::Inflated,
                                         navigation_mapping::Vec3{4.5, 0.0, 0.0});
  EXPECT_EQ(world.cellState(navigation_mapping::WorldLayer::Inflated, unknown),
            navigation_mapping::CellState::Unknown);

  const auto blocked = AStar{}.search(
      world, SearchRequest{navigation_mapping::WorldLayer::Inflated,
                           UnknownPolicy::TreatUnknownAsBlocked,
                           world.gridToWorld(navigation_mapping::WorldLayer::Inflated, unknown),
                           world.gridToWorld(navigation_mapping::WorldLayer::Inflated, unknown)});
  EXPECT_FALSE(blocked.success);
  EXPECT_EQ(blocked.failure, SearchFailureCode::NoPath);

  const auto traversable = AStar{}.search(
      world, SearchRequest{navigation_mapping::WorldLayer::Inflated,
                           UnknownPolicy::TreatUnknownAsTraversable,
                           world.gridToWorld(navigation_mapping::WorldLayer::Inflated, unknown),
                           world.gridToWorld(navigation_mapping::WorldLayer::Inflated, unknown)});
  EXPECT_TRUE(traversable.success);
}

TEST(RogWorldModelIntegrationTest, InflatedMixedCoarseCellUsesCounterMapUnknownState) {
  navigation_mapping::MappingPipeline pipeline(
      config(), []() { return 0.0; }, testDirectory("rog_world_model_mixed_unknown"));
  for (int i = 0; i < 2; ++i) {
    pipeline.process(rayObservation(navigation_mapping::Vec3{0.0, 0.2, 0.2},
                                    navigation_mapping::Vec3{3.0, 0.2, 0.2}, i + 1));
  }

  navigation_mapping::WorldModel world(pipeline.adapter());
  const auto inflated = world.worldToGrid(navigation_mapping::WorldLayer::Inflated,
                                          navigation_mapping::Vec3{0.2, 0.2, 0.2});
  const auto center_probability = world.worldToGrid(
      navigation_mapping::WorldLayer::Probability,
      world.gridToWorld(navigation_mapping::WorldLayer::Inflated, inflated));

  // The ray makes the center fine voxel known free, but only two of the
  // eight fine voxels represented by this 0.40 m CounterMap cell known free.
  EXPECT_EQ(world.cellState(navigation_mapping::WorldLayer::Probability, center_probability),
            navigation_mapping::CellState::KnownFree);
  EXPECT_EQ(world.cellState(navigation_mapping::WorldLayer::Inflated, inflated),
            navigation_mapping::CellState::Unknown);

  const auto blocked = AStar{}.search(
      world, SearchRequest{navigation_mapping::WorldLayer::Inflated,
                           UnknownPolicy::TreatUnknownAsBlocked,
                           world.gridToWorld(navigation_mapping::WorldLayer::Inflated, inflated),
                           world.gridToWorld(navigation_mapping::WorldLayer::Inflated, inflated)});
  EXPECT_FALSE(blocked.success);
  EXPECT_EQ(blocked.failure, SearchFailureCode::NoPath);

  const auto traversable = AStar{}.search(
      world, SearchRequest{navigation_mapping::WorldLayer::Inflated,
                           UnknownPolicy::TreatUnknownAsTraversable,
                           world.gridToWorld(navigation_mapping::WorldLayer::Inflated, inflated),
                           world.gridToWorld(navigation_mapping::WorldLayer::Inflated, inflated)});
  EXPECT_TRUE(traversable.success);
}

TEST(RogWorldModelIntegrationTest, InflatedCoarseCellUsesKnownFreeCounterMapAggregate) {
  navigation_mapping::MappingPipeline pipeline(
      config(), []() { return 0.0; }, testDirectory("rog_world_model_mixed_free"));
  for (int i = 0; i < 2; ++i) {
    pipeline.process(rayObservation(navigation_mapping::Vec3{0.0, 0.1, 0.1},
                                    navigation_mapping::Vec3{3.0, 0.1, 0.1}, 1 + i * 3));
    pipeline.process(rayObservation(navigation_mapping::Vec3{0.0, 0.3, 0.1},
                                    navigation_mapping::Vec3{3.0, 0.3, 0.1}, 2 + i * 3));
    pipeline.process(rayObservation(navigation_mapping::Vec3{0.0, 0.1, 0.3},
                                    navigation_mapping::Vec3{3.0, 0.1, 0.3}, 3 + i * 3));
  }

  navigation_mapping::WorldModel world(pipeline.adapter());
  const auto inflated = world.worldToGrid(navigation_mapping::WorldLayer::Inflated,
                                          navigation_mapping::Vec3{0.2, 0.2, 0.2});
  const auto center_probability = world.worldToGrid(
      navigation_mapping::WorldLayer::Probability,
      world.gridToWorld(navigation_mapping::WorldLayer::Inflated, inflated));

  // Three rays make six of the eight fine voxels known free. The center fine
  // voxel remains unknown, so center sampling would return the wrong state.
  EXPECT_EQ(world.cellState(navigation_mapping::WorldLayer::Probability, center_probability),
            navigation_mapping::CellState::Unknown);
  EXPECT_EQ(world.cellState(navigation_mapping::WorldLayer::Inflated, inflated),
            navigation_mapping::CellState::KnownFree);
}

TEST(RogWorldModelIntegrationTest, InflatedOccupiedCounterOverridesUnderlyingAggregate) {
  navigation_mapping::MappingPipeline pipeline(
      config(), []() { return 0.0; }, testDirectory("rog_world_model_occupied_inflation"));
  for (int i = 0; i < 2; ++i) {
    pipeline.process(rayObservation(navigation_mapping::Vec3{-1.0, 0.2, 0.2},
                                    navigation_mapping::Vec3{0.6, 0.2, 0.2}, i + 1));
  }

  navigation_mapping::WorldModel world(pipeline.adapter());
  const auto inflated = world.worldToGrid(navigation_mapping::WorldLayer::Inflated,
                                          navigation_mapping::Vec3{0.2, 0.2, 0.2});
  EXPECT_EQ(world.cellState(navigation_mapping::WorldLayer::Inflated, inflated),
            navigation_mapping::CellState::Occupied);
}

TEST(RogWorldModelIntegrationTest, InflatedSameResolutionMatchesFineMapSemantics) {
  auto same_resolution = config();
  same_resolution.rog.inflation_resolution_m = 0.2;
  navigation_mapping::MappingPipeline pipeline(
      same_resolution, []() { return 0.0; }, testDirectory("rog_world_model_same_resolution"));
  for (int i = 0; i < 2; ++i) {
    pipeline.process(observation());
  }

  navigation_mapping::WorldModel world(pipeline.adapter());
  const navigation_mapping::Vec3 position{1.0, 0.0, 0.0};
  const auto probability = world.worldToGrid(navigation_mapping::WorldLayer::Probability, position);
  const auto inflated = world.worldToGrid(navigation_mapping::WorldLayer::Inflated, position);
  EXPECT_EQ(world.cellState(navigation_mapping::WorldLayer::Probability, probability),
            navigation_mapping::CellState::KnownFree);
  EXPECT_EQ(world.cellState(navigation_mapping::WorldLayer::Inflated, inflated),
            navigation_mapping::CellState::KnownFree);
}

TEST(RogWorldModelIntegrationTest, InflatedBoundsExcludeRogMaintenanceHalo) {
  const std::string directory =
      std::string(NAVIGATION_PLANNING_TEST_TMP_DIR) + "/rog_world_model_bounds";
  std::filesystem::create_directories(directory);
  navigation_mapping::MappingPipeline pipeline(config(), []() { return 0.0; }, directory);
  pipeline.process(observation());

  navigation_mapping::WorldModel world(pipeline.adapter());
  const auto& rog_config = pipeline.adapter().map().getMapConfig();
  rog_map::Vec3f halo_position;
  pipeline.adapter().map().infMapGlobalIndexToPos(rog_config.inf_half_map_size_i, halo_position);
  const auto halo_index = world.worldToGrid(
      navigation_mapping::WorldLayer::Inflated,
      navigation_mapping::Vec3{halo_position.x(), halo_position.y(), halo_position.z()});
  EXPECT_FALSE(world.bounds(navigation_mapping::WorldLayer::Inflated).contains(halo_index));
}

TEST(RogWorldModelIntegrationTest, GenerationIsCapturedAndUpdatedAtomically) {
  const std::string directory =
      std::string(NAVIGATION_PLANNING_TEST_TMP_DIR) + "/rog_world_model_generation";
  std::filesystem::create_directories(directory);
  navigation_mapping::MappingPipeline pipeline(config(), []() { return 0.0; }, directory);
  pipeline.process(observation());
  navigation_mapping::WorldModel world(pipeline.adapter());
  EXPECT_EQ(world.generation(), 1U);

  auto next_observation = observation();
  next_observation.public_frame_generation = 2;
  pipeline.process(next_observation);
  EXPECT_EQ(world.generation(), 2U);

  const auto result = AStar{}.search(
      world, SearchRequest{navigation_mapping::WorldLayer::Probability,
                           UnknownPolicy::TreatUnknownAsTraversable,
                           navigation_mapping::Vec3{0.0, 0.0, 0.0},
                           navigation_mapping::Vec3{0.0, 0.0, 0.0}});
  EXPECT_EQ(result.world_generation, 2U);
}

TEST(RogWorldModelIntegrationTest, InvalidInflationRatioIsRejectedAtPipelineConstruction) {
  auto invalid = config();
  invalid.rog.inflation_resolution_m = 0.3;
  EXPECT_THROW(
      { navigation_mapping::MappingPipeline pipeline(invalid, []() { return 0.0; }, "/tmp"); },
      std::invalid_argument);
}

TEST(RogWorldModelIntegrationTest, PlannerReportsUnavailableWorldModelBeforeMapInitialization) {
  const std::string directory =
      std::string(NAVIGATION_PLANNING_TEST_TMP_DIR) + "/rog_world_model_unready";
  std::filesystem::create_directories(directory);
  navigation_mapping::RogMapAdapter adapter([]() { return 0.0; }, directory);
  navigation_mapping::WorldModel world(adapter);
  EXPECT_FALSE(world.isReady());
  const auto result = AStar{}.search(
      world, SearchRequest{navigation_mapping::WorldLayer::Probability,
                           UnknownPolicy::TreatUnknownAsBlocked,
                           navigation_mapping::Vec3::Zero(),
                           navigation_mapping::Vec3::Zero()});
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, SearchFailureCode::WorldModelUnavailable);
}

}  // namespace
}  // namespace navigation_planning
