#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "fast_lio_ros/ros_livox_custom_adapter.hpp"

namespace uav::nav::lio {
namespace {

livox_ros_driver2::msg::CustomMsg validMessage() {
  livox_ros_driver2::msg::CustomMsg message;
  message.header.frame_id = "livox_frame";
  message.header.stamp.sec = 2;
  message.timebase = 2'000'000'000ULL;
  message.point_num = 2U;
  message.lidar_id = 1U;
  message.points.resize(2U);
  message.points[0].x = 1.0F;
  message.points[0].y = 2.0F;
  message.points[0].z = 3.0F;
  message.points[0].reflectivity = 42U;
  message.points[0].tag = 0x10U;
  message.points[0].line = 3U;
  message.points[1].offset_time = 500U;
  message.points[1].x = 4.0F;
  message.points[1].y = 5.0F;
  message.points[1].z = 6.0F;
  message.points[1].reflectivity = 21U;
  message.points[1].tag = 0x20U;
  message.points[1].line = 2U;
  return message;
}

}  // namespace

TEST(RosLivoxCustomAdapterTest, PreservesOfficialFieldsAndNanosecondOffsets) {
  const auto scan =
      RosLivoxCustomAdapter{"livox_frame", ClockDomain::kSensorTime}.convert(validMessage());

  ASSERT_EQ(scan.points.size(), 2U);
  EXPECT_EQ(scan.start_time, Timestamp(2'000'000'000LL, ClockDomain::kSensorTime));
  EXPECT_EQ(scan.end_time, Timestamp(2'000'000'500LL, ClockDomain::kSensorTime));
  EXPECT_TRUE(scan.has_per_point_time);
  EXPECT_EQ(scan.points[1].relative_time_ns, 500U);
  EXPECT_EQ(scan.points[0].reflectivity, 42U);
  EXPECT_EQ(scan.points[0].tag, 0x10U);
  EXPECT_EQ(scan.points[0].line, 3U);
}

TEST(RosLivoxCustomAdapterTest, RejectsPointCountMismatch) {
  auto message = validMessage();
  message.point_num = 1U;
  EXPECT_THROW(
      RosLivoxCustomAdapter("livox_frame", ClockDomain::kSensorTime).convert(message),
      std::invalid_argument);
}

TEST(RosLivoxCustomAdapterTest, StrictPolicyRejectsHeaderTimebaseMismatch) {
  auto message = validMessage();
  message.header.stamp.nanosec = 1U;
  EXPECT_THROW(
      RosLivoxCustomAdapter("livox_frame", ClockDomain::kSensorTime).convert(message),
      std::invalid_argument);
}

TEST(RosLivoxCustomAdapterTest, ExplicitTimebasePolicyAcceptsHeaderMismatch) {
  auto message = validMessage();
  message.header.stamp.nanosec = 1U;
  const auto scan = RosLivoxCustomAdapter{
      "livox_frame", ClockDomain::kSensorTime,
      LivoxTimestampPolicy::kTimebaseAuthoritative}
                        .convert(message);
  EXPECT_EQ(scan.start_time.nanoseconds(), 2'000'000'000LL);
}

TEST(RosLivoxCustomAdapterTest, RejectsTimebaseOutsideCoreRange) {
  auto message = validMessage();
  message.timebase = std::numeric_limits<std::uint64_t>::max();
  EXPECT_THROW(
      RosLivoxCustomAdapter("livox_frame", ClockDomain::kSensorTime).convert(message),
      std::invalid_argument);
}

TEST(RosLivoxCustomAdapterTest, RejectsInvalidAdapterConfiguration) {
  EXPECT_THROW(RosLivoxCustomAdapter("", ClockDomain::kSensorTime),
               std::invalid_argument);
  EXPECT_THROW(RosLivoxCustomAdapter("livox_frame", static_cast<ClockDomain>(255)),
               std::invalid_argument);
  EXPECT_THROW(RosLivoxCustomAdapter(
                   "livox_frame", ClockDomain::kSensorTime,
                   static_cast<LivoxTimestampPolicy>(255)),
               std::invalid_argument);
  PointTimeConfig invalid_overlap;
  invalid_overlap.maximum_boundary_overlap_ns = -1;
  EXPECT_THROW(RosLivoxCustomAdapter(
                   "livox_frame", ClockDomain::kSensorTime,
                   LivoxTimestampPolicy::kRequireHeaderMatchesTimebase,
                   invalid_overlap),
               std::invalid_argument);
  PointTimeConfig invalid_minimum;
  invalid_minimum.minimum_points_after_overlap_trim = 0U;
  EXPECT_THROW(RosLivoxCustomAdapter(
                   "livox_frame", ClockDomain::kSensorTime,
                   LivoxTimestampPolicy::kRequireHeaderMatchesTimebase,
                   invalid_minimum),
               std::invalid_argument);
}

TEST(RosLivoxCustomAdapterTest, EnforcesSharedScanDurationLimit) {
  PointTimeConfig point_time;
  point_time.maximum_scan_duration_ns = 100U;
  EXPECT_THROW(
      RosLivoxCustomAdapter("livox_frame", ClockDomain::kSensorTime,
                            LivoxTimestampPolicy::kTimebaseAuthoritative,
                            point_time)
          .convert(validMessage()),
      std::invalid_argument);
}

TEST(RosLivoxCustomAdapterTest, EnforcesHeaderOffsetForAuthoritativeTimebase) {
  auto message = validMessage();
  message.header.stamp.nanosec = 300'000'000U;
  PointTimeConfig point_time;
  point_time.maximum_header_offset_ns = 100'000'000;
  EXPECT_THROW(
      RosLivoxCustomAdapter("livox_frame", ClockDomain::kSensorTime,
                            LivoxTimestampPolicy::kTimebaseAuthoritative,
                            point_time)
      .convert(message),
      std::invalid_argument);
}

TEST(RosLivoxCustomAdapterTest, DeterministicallyTrimsBoundedBoundaryOverlap) {
  PointTimeConfig point_time;
  point_time.maximum_boundary_overlap_ns = 100U;
  point_time.minimum_points_after_overlap_trim = 1U;
  RosLivoxCustomAdapter adapter{"livox_frame", ClockDomain::kSensorTime,
                               LivoxTimestampPolicy::kRequireHeaderMatchesTimebase,
                               point_time};
  static_cast<void>(adapter.convert(validMessage()));
  auto second = validMessage();
  second.header.stamp.nanosec = 400U;
  second.timebase = 2'000'000'400ULL;
  second.points[0].offset_time = 0U;
  second.points[1].offset_time = 200U;
  const auto scan = adapter.convert(second);
  ASSERT_EQ(scan.points.size(), 1U);
  EXPECT_EQ(scan.points.front().relative_time_ns, 200U);
  EXPECT_EQ(scan.end_time.nanoseconds(), 2'000'000'600LL);
  const auto stats = adapter.normalizationStatistics();
  EXPECT_EQ(stats.input_point_count, 4U);
  EXPECT_EQ(stats.emitted_point_count, 3U);
  EXPECT_EQ(stats.dropped_overlapping_point_count, 1U);
}

TEST(RosLivoxCustomAdapterTest, RejectsBoundaryOverlapBeyondConfiguredLimit) {
  PointTimeConfig point_time;
  point_time.maximum_boundary_overlap_ns = 50U;
  RosLivoxCustomAdapter adapter{"livox_frame", ClockDomain::kSensorTime,
                               LivoxTimestampPolicy::kRequireHeaderMatchesTimebase,
                               point_time};
  static_cast<void>(adapter.convert(validMessage()));
  auto second = validMessage();
  second.header.stamp.nanosec = 400U;
  second.timebase = 2'000'000'400ULL;
  second.points[1].offset_time = 200U;
  EXPECT_THROW(adapter.convert(second), std::invalid_argument);
}

TEST(RosLivoxCustomAdapterTest, RejectsOversizedPointPayloadBeforeReserve) {
  auto message = validMessage();
  message.point_num = 2'000'001U;
  message.points.resize(2'000'001U);
  EXPECT_THROW(
      RosLivoxCustomAdapter("livox_frame", ClockDomain::kSensorTime)
          .convert(message),
      std::invalid_argument);
}

TEST(RosLivoxCustomAdapterTest, RejectsTimestampRegressionWithSharedPointTimePolicy) {
  auto first = validMessage();
  auto second = validMessage();
  second.header.stamp.sec = 1;
  second.header.stamp.nanosec = 900'000'000U;
  second.timebase = 1'900'000'000ULL;
  RosLivoxCustomAdapter adapter{"livox_frame", ClockDomain::kSensorTime};
  static_cast<void>(adapter.convert(first));
  EXPECT_THROW(adapter.convert(second), std::invalid_argument);
}

}  // namespace uav::nav::lio
