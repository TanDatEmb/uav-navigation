// Generation N populates the map; generation N+1 must reset the map and adopt
// the new generation; a subsequent generation-N observation must be rejected.
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

MappingPipelineConfig selfReturnConfig() {
  auto config = smallConfig();
  config.point_filter.voxel_size_m = 0.0;
  config.point_filter.minimum_range_m = 0.5;
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

TEST(MappingPipelineGenerationResetTest, RevisionIncrementsOnlyAfterCommittedUpdates) {
  MappingPipeline pipeline(smallConfig(), []() { return 0.0; }, testTmpDir());
  EXPECT_EQ(pipeline.adapter().revision(), 0U);
  pipeline.process(makeObservation(1, 3.0));
  EXPECT_EQ(pipeline.adapter().revision(), 1U);
  pipeline.process(makeObservation(1, 3.0));
  EXPECT_EQ(pipeline.adapter().revision(), 2U);
  pipeline.process(makeObservation(2, 3.0));
  EXPECT_EQ(pipeline.adapter().revision(), 3U);
  EXPECT_EQ(pipeline.diagnostics().revision, 3U);
}

TEST(MappingPipelineGenerationResetTest, RejectsSelfReturnBeforeRogAndKeepsObstacle) {
  MappingPipeline pipeline(selfReturnConfig(), []() { return 0.0; }, testTmpDir());
  ObservationInput input = makeObservation(1, 2.0);
  input.points_lidar_m = {Point3f(0.3, 0.0, 0.0), Point3f(2.0, 0.0, 0.0)};

  for (int i = 0; i < 8; ++i) {
    input.header_stamp.sec = i + 1;
    input.points_stamp = input.header_stamp;
    pipeline.process(input);
  }

  EXPECT_EQ(pipeline.diagnostics().mapping_filter_input_point_count, 16U);
  EXPECT_EQ(pipeline.diagnostics().range_filtered_point_count, 8U);
  EXPECT_EQ(pipeline.diagnostics().mapping_filter_output_point_count, 8U);
  EXPECT_EQ(pipeline.diagnostics().rog_endpoint_count, 8U);
  EXPECT_EQ(pipeline.diagnostics().rog_ray_attempt_count, 8U);
  EXPECT_EQ(pipeline.diagnostics().rog_skip_below_raycast_min_range, 0U);
  EXPECT_TRUE(pipeline.adapter().map().isOccupied(rog_map::Vec3f(2.0, 0.0, 0.0)));
  EXPECT_FALSE(pipeline.adapter().map().isOccupied(rog_map::Vec3f(0.3, 0.0, 0.0)));
}

}  // namespace
}  // namespace navigation_mapping
