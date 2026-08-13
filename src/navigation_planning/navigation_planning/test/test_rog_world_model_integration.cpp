#include <gtest/gtest.h>

#include <filesystem>

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
  EXPECT_EQ(result.path.front(), start);
  EXPECT_EQ(result.path.back(), goal);
  EXPECT_DOUBLE_EQ(result.statistics.path_length_m, 2.0);
  EXPECT_GT(result.statistics.cell_state_queries, 0U);
}

}  // namespace
}  // namespace navigation_planning
