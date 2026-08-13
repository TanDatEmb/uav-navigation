// P1 mandatory correctness test F (docs/architecture/navigation_layers.md,
// section 7): generation N populates the map; generation N+1 must reset the
// map and adopt the new generation; a subsequent generation-N observation
// must be rejected as stale.
#include <gtest/gtest.h>

#include <filesystem>

#include "navigation_mapping/mapping_pipeline.hpp"

namespace navigation_mapping {
namespace {

std::string testTmpDir() {
  const std::string dir = std::string(NAVIGATION_MAPPING_TEST_TMP_DIR) + "/generation_reset";
  std::filesystem::create_directories(dir);
  return dir;
}

MappingPipelineConfig smallConfig() {
  MappingPipelineConfig config;
  config.rog.resolution_m = 0.2;
  config.rog.local_map_size_m = {10.0, 10.0, 6.0};
  config.rog.ray_range_min_m = 0.1;
  config.rog.ray_range_max_m = 10.0;
  config.point_filter.voxel_size_m = 0.1;
  return config;
}

ObservationInput makeObservation(std::uint64_t generation, double obstacle_x) {
  ObservationInput input;
  input.header_frame_id = "lio_odom";
  input.points_frame_id = "livox_frame";
  input.header_stamp.sec = 1;
  input.header_stamp.nanosec = 0;
  input.points_stamp = input.header_stamp;
  input.sensor_pose.orientation.w = 1.0;
  input.points_lidar_m = {Point3f(obstacle_x, 0.0, 0.0)};
  input.public_frame_generation = generation;
  return input;
}

TEST(MappingPipelineGenerationResetTest, HigherGenerationResetsMapAndAdopts) {
  MappingPipeline pipeline(smallConfig(), []() { return 0.0; }, testTmpDir());

  for (int i = 0; i < 5; ++i) {
    pipeline.process(makeObservation(1, 3.0));
  }
  ASSERT_TRUE(pipeline.adapter().isInitialized());
  const auto reset_count_before = pipeline.adapter().resetCount();
  EXPECT_EQ(pipeline.diagnostics().generation, 1U);
  EXPECT_GT(pipeline.diagnostics().accepted_observation_count, 0U);

  pipeline.process(makeObservation(2, 3.0));
  EXPECT_EQ(pipeline.diagnostics().generation, 2U);
  EXPECT_GT(pipeline.adapter().resetCount(), reset_count_before);
}

TEST(MappingPipelineGenerationResetTest, StaleGenerationIsRejectedAfterAdvance) {
  MappingPipeline pipeline(smallConfig(), []() { return 0.0; }, testTmpDir());
  pipeline.process(makeObservation(1, 3.0));
  pipeline.process(makeObservation(2, 3.0));
  const auto accepted_before = pipeline.diagnostics().accepted_observation_count;
  const auto generation_before = pipeline.diagnostics().generation;

  pipeline.process(makeObservation(1, 3.0));  // stale: must be dropped

  EXPECT_EQ(pipeline.diagnostics().old_generation_drop_count, 1U);
  EXPECT_EQ(pipeline.diagnostics().accepted_observation_count, accepted_before);
  EXPECT_EQ(pipeline.diagnostics().generation, generation_before);
}

TEST(MappingPipelineGenerationResetTest, SameGenerationDoesNotResetMap) {
  MappingPipeline pipeline(smallConfig(), []() { return 0.0; }, testTmpDir());
  pipeline.process(makeObservation(1, 3.0));
  const auto reset_count_after_first = pipeline.adapter().resetCount();
  for (int i = 0; i < 10; ++i) {
    pipeline.process(makeObservation(1, 3.0));
  }
  EXPECT_EQ(pipeline.adapter().resetCount(), reset_count_after_first);
}

}  // namespace
}  // namespace navigation_mapping
