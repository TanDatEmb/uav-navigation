#include <gtest/gtest.h>

#include <filesystem>
#include <stdexcept>

#include "navigation_mapping/mapping_pipeline.hpp"
#include "navigation_mapping/world_model.hpp"
#include "navigation_planning/a_star.hpp"

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
