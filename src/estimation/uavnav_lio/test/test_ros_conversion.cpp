// Message conversion at the boundary of the lio node (SYSTEM_DESIGN §3.3, §6.5, D33/O20, O21; Review Focus 5).
// Pure conversions: no rclcpp context, no DDS.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Geometry>

#include "cloud_msg.hpp"
#include "uavnav/lio/lifecycle.hpp"
#include "uavnav/lio/outputs.hpp"
#include "uavnav/lio_node/ingest.hpp"
#include "uavnav/lio_node/ros_conversion.hpp"
#include "uavnav_interfaces/msg/lio_health.hpp"

using namespace uavnav;
using namespace uavnav::lio_node;
using sensor_msgs::msg::PointField;

namespace {

constexpr std::int64_t kStampNs = 1'700'000'000'123'456'789;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr float kInfF = std::numeric_limits<float>::infinity();

std::vector<Eigen::Vector3f> Points(int n) {
  std::vector<Eigen::Vector3f> p;
  for (int i = 0; i < n; ++i) p.emplace_back(0.1F * static_cast<float>(i), -2.0F, 1.5F + 0.01F * static_cast<float>(i));
  return p;
}

sensor_msgs::msg::PointCloud2 GoodCloud() { return testmsg::Cloud(Points(10), kStampNs); }
sensor_msgs::msg::Imu GoodImu() {
  return testmsg::Imu(kStampNs, Eigen::Vector3d(0.01, -0.02, 0.3), Eigen::Vector3d(0.1, 0.2, 9.8));
}

ConversionError ScanError(const sensor_msgs::msg::PointCloud2& c) {
  CloudStats stats{1234, 56};
  const auto r = to_scan_input(c, stats);
  EXPECT_FALSE(r.has_value());
  EXPECT_EQ(stats.points_in, 1234U) << "a refused cloud changes nothing, not even the stats";
  EXPECT_EQ(stats.points_non_finite, 56U);
  return r ? ConversionError::kBadStamp : r.error();
}

ConversionError ImuError(const sensor_msgs::msg::Imu& m) {
  const auto r = to_imu_input(m);
  EXPECT_FALSE(r.has_value());
  return r ? ConversionError::kBadStamp : r.error();
}

}  // namespace

TEST(LioRosConversion, ImuMessageToInput) {
  sensor_msgs::msg::Imu m = GoodImu();
  m.orientation.x = kNaN;  // Gazebo fills it; the LIO ignores it, so even a NaN orientation is not a refusal
  m.orientation_covariance[0] = -1.0;
  const auto r = to_imu_input(m);
  ASSERT_TRUE(r.has_value()) << to_string(r.error());
  EXPECT_EQ(r->t.ns, kStampNs);
  EXPECT_EQ(r->gyro_rad_s, Eigen::Vector3d(0.01, -0.02, 0.3));
  EXPECT_EQ(r->accel_mps2, Eigen::Vector3d(0.1, 0.2, 9.8)) << "m/s^2 as published (O20), no rescaling";
}

TEST(LioRosConversion, PointCloudDropsNonFinitePointsAndCountsThem) {
  std::vector<Eigen::Vector3f> pts = Points(100);
  std::vector<Eigen::Vector3f> expected;
  int bad = 0;
  for (int i = 0; i < 100; ++i) {
    if (i % 14 == 3) {  // 7 points: inf (Gazebo's no-return) and NaN, in each coordinate
      pts[static_cast<std::size_t>(i)][bad % 3] = (bad % 2 == 0) ? kInfF : std::numeric_limits<float>::quiet_NaN();
      ++bad;
    } else {
      expected.push_back(pts[static_cast<std::size_t>(i)]);
    }
  }
  ASSERT_EQ(bad, 7);
  CloudStats stats{0, 0};
  const auto r = to_scan_input(testmsg::Cloud(pts, kStampNs), stats);
  ASSERT_TRUE(r.has_value()) << to_string(r.error());
  EXPECT_EQ(stats.points_in, 100U);
  EXPECT_EQ(stats.points_non_finite, 7U);
  ASSERT_EQ(r->points.size(), 93U);
  for (std::size_t i = 0; i < expected.size(); ++i) {
    ASSERT_EQ(r->points[i].position_lidar_m, expected[i]) << "point " << i << " kept in order, unchanged";
    EXPECT_EQ(r->points[i].relative_time_ns, 0U);
  }
  EXPECT_EQ(r->start.ns, kStampNs) << "a Gazebo scan is one instant (O20)";
  EXPECT_EQ(r->end.ns, kStampNs);
  EXPECT_FALSE(r->per_point_time);

  // An organised cloud (height > 1) with the same bytes is the same scan.
  sensor_msgs::msg::PointCloud2 organised = testmsg::Cloud(pts, kStampNs);
  organised.height = 4;
  organised.width = 25;
  organised.row_step = 25 * testmsg::kPointStep;
  CloudStats stats2{0, 0};
  const auto r2 = to_scan_input(organised, stats2);
  ASSERT_TRUE(r2.has_value());
  EXPECT_EQ(r2->points.size(), 93U);
  EXPECT_EQ(stats2.points_in, 100U);
}

// Review Focus 5.
TEST(LioRosConversion, RejectsGarbageMessages) {
  {  // IMU
    sensor_msgs::msg::Imu m = GoodImu();
    m.header.stamp.sec = -1;
    EXPECT_EQ(ImuError(m), ConversionError::kBadStamp);
    m = GoodImu();
    m.header.stamp.nanosec = 1'000'000'000;
    EXPECT_EQ(ImuError(m), ConversionError::kBadStamp);
    m = GoodImu();
    m.linear_acceleration.y = kNaN;
    EXPECT_EQ(ImuError(m), ConversionError::kNotFinite);
    m = GoodImu();
    m.angular_velocity.z = -std::numeric_limits<double>::infinity();
    EXPECT_EQ(ImuError(m), ConversionError::kNotFinite);
    m = GoodImu();
    m.header.frame_id = "base_link";
    EXPECT_EQ(ImuError(m), ConversionError::kWrongFrame);
  }
  {  // stamps and frame of a cloud
    sensor_msgs::msg::PointCloud2 c = GoodCloud();
    c.header.stamp.sec = -1;
    EXPECT_EQ(ScanError(c), ConversionError::kBadStamp);
    c = GoodCloud();
    c.header.stamp.nanosec = 1'000'000'000;
    EXPECT_EQ(ScanError(c), ConversionError::kBadStamp);
    c = GoodCloud();
    c.header.frame_id = "base_link";
    EXPECT_EQ(ScanError(c), ConversionError::kWrongFrame);
    c.header.frame_id = "";
    EXPECT_EQ(ScanError(c), ConversionError::kWrongFrame);
  }
  {  // layout
    sensor_msgs::msg::PointCloud2 c = GoodCloud();
    c.fields.erase(c.fields.begin() + 2);  // no "z"
    EXPECT_EQ(ScanError(c), ConversionError::kMalformedCloud) << "no z field";
    c = GoodCloud();
    c.fields[0].datatype = PointField::FLOAT64;
    EXPECT_EQ(ScanError(c), ConversionError::kMalformedCloud) << "FLOAT64 x";
    c = GoodCloud();
    c.fields[1].count = 3;
    EXPECT_EQ(ScanError(c), ConversionError::kMalformedCloud) << "y with count 3";
    c = GoodCloud();
    c.fields.push_back(testmsg::Field("x", 12, PointField::FLOAT32));
    EXPECT_EQ(ScanError(c), ConversionError::kMalformedCloud) << "two x fields";
    c = GoodCloud();
    c.point_step = 8;
    c.row_step = 8 * c.width;
    c.data.resize(static_cast<std::size_t>(c.row_step));
    EXPECT_EQ(ScanError(c), ConversionError::kMalformedCloud) << "point_step 8";
    c = GoodCloud();
    c.fields[2].offset = testmsg::kPointStep - 2;
    EXPECT_EQ(ScanError(c), ConversionError::kMalformedCloud) << "z reads past the point";
    c = GoodCloud();
    c.data.pop_back();
    EXPECT_EQ(ScanError(c), ConversionError::kMalformedCloud) << "data one byte short";
    c = GoodCloud();
    c.data.push_back(0);
    EXPECT_EQ(ScanError(c), ConversionError::kMalformedCloud) << "data one byte long";
    c = GoodCloud();
    c.row_step += 4;
    EXPECT_EQ(ScanError(c), ConversionError::kMalformedCloud) << "padded rows";
    c = GoodCloud();
    c.is_bigendian = true;
    EXPECT_EQ(ScanError(c), ConversionError::kMalformedCloud) << "big-endian data";
    c = GoodCloud();
    c.width = 0x80000000U;
    c.height = 0x80000000U;
    EXPECT_EQ(ScanError(c), ConversionError::kMalformedCloud) << "width * height * point_step overflows";
  }
  // An empty cloud is a well-formed scan (the core answers kScanEmpty), not garbage.
  CloudStats stats{0, 0};
  const auto empty = to_scan_input(testmsg::Cloud({}, kStampNs), stats);
  ASSERT_TRUE(empty.has_value());
  EXPECT_TRUE(empty->points.empty());

  EXPECT_EQ(to_string(ConversionError::kBadStamp), "BAD_STAMP");
  EXPECT_EQ(to_string(ConversionError::kNotFinite), "NOT_FINITE");
  EXPECT_EQ(to_string(ConversionError::kWrongFrame), "WRONG_FRAME");
  EXPECT_EQ(to_string(ConversionError::kMalformedCloud), "MALFORMED_CLOUD");
}

TEST(LioRosConversion, OutputsMapToMessages) {
  using Health = uavnav_interfaces::msg::LioHealth;
  static_assert(Health::INITIALIZING == static_cast<std::uint8_t>(lio::LioState::kInitializing));
  static_assert(Health::RESTARTING == static_cast<std::uint8_t>(lio::LioState::kRestarting));

  LioStateOut s;
  s.t = time::SensorTime{kStampNs};
  s.epoch = 3;
  s.reset_counter = 2;
  s.state = lio::LioState::kDegraded;
  s.p_world_base = Eigen::Vector3d(1.0, 2.0, 3.0);
  s.q_world_base = Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  s.v_base_mps = Eigen::Vector3d(0.5, -0.25, 0.125);
  s.w_base_rad_s = Eigen::Vector3d(0.01, 0.02, 0.03);
  s.last_reset = lio::ResetDelta{Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(-0.1, -0.2, -0.3), 0.7};
  const auto sm = to_msg(s);
  EXPECT_EQ(sm.stamp.sec, 1'700'000'000);
  EXPECT_EQ(sm.stamp.nanosec, 123'456'789U);
  EXPECT_EQ(sm.epoch, 3U);
  EXPECT_EQ(sm.reset_counter, 2U);
  EXPECT_EQ(sm.lio_state, Health::DEGRADED);
  EXPECT_EQ(sm.pose.position.x, 1.0);
  EXPECT_EQ(sm.pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(sm.pose.orientation.w, s.q_world_base.w());
  EXPECT_DOUBLE_EQ(sm.pose.orientation.z, s.q_world_base.z());
  EXPECT_EQ(sm.velocity.y, -0.25);
  EXPECT_EQ(sm.angular_velocity.z, 0.03);
  for (double c : sm.pose_covariance) EXPECT_TRUE(std::isnan(c)) << "the predictor has no covariance (O21)";
  for (double c : sm.velocity_covariance) EXPECT_TRUE(std::isnan(c));
  EXPECT_EQ(sm.reset_delta_position.y, 0.2);
  EXPECT_EQ(sm.reset_delta_velocity.z, -0.3);
  EXPECT_EQ(sm.reset_delta_yaw, 0.7);

  lio::HealthOutput h{time::SensorTime{kStampNs}, lio::LioState::kTracking, 4, lio::LioReason::kConfirmationReached,
                      1.5e5, 3.0e6, 0.08, Eigen::Vector3d(0.001, 0.02, 0.3)};
  const auto hm = to_msg(h);
  EXPECT_EQ(hm.stamp.sec, 1'700'000'000);
  EXPECT_EQ(hm.state, Health::TRACKING);
  EXPECT_EQ(hm.epoch, 4U);
  EXPECT_EQ(hm.reason, static_cast<std::uint16_t>(lio::LioReason::kConfirmationReached));
  EXPECT_EQ(hm.reason_name, "CONFIRMATION_REACHED");
  EXPECT_EQ(hm.translation_min_eigenvalue, 1.5e5);
  EXPECT_EQ(hm.rotation_min_eigenvalue, 3.0e6);
  EXPECT_EQ(hm.correction_age_s, 0.08);
  EXPECT_EQ(hm.output_error_attitude_rad, 0.001);
  EXPECT_EQ(hm.output_error_velocity_mps, 0.02);
  EXPECT_EQ(hm.output_error_position_m, 0.3);

  lio::OdometryOutput o;
  o.t = time::SensorTime{kStampNs};
  o.epoch = 5;
  o.reset_counter = 4;
  o.p_world_m = Eigen::Vector3d(-1.0, 0.5, 2.0);
  o.q_world_base = Eigen::Quaterniond(Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitX()));
  o.v_world_mps = Eigen::Vector3d(0.3, 0.2, 0.1);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) o.pose_cov(r, c) = 10.0 * r + c;  // not symmetric: a transposed copy shows
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) o.vel_cov(r, c) = 100.0 + 10.0 * r + c;
  }
  o.quality = 87;
  const auto om = to_msg(o);
  EXPECT_EQ(om.stamp.nanosec, 123'456'789U);
  EXPECT_EQ(om.epoch, 5U);
  EXPECT_EQ(om.reset_counter, 4U);
  EXPECT_EQ(om.pose.position.x, -1.0);
  EXPECT_DOUBLE_EQ(om.pose.orientation.x, o.q_world_base.x());
  EXPECT_DOUBLE_EQ(om.pose.orientation.w, o.q_world_base.w());
  EXPECT_EQ(om.velocity.x, 0.3);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      EXPECT_EQ(om.pose_covariance[static_cast<std::size_t>(6 * r + c)], o.pose_cov(r, c)) << "row-major";
    }
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) EXPECT_EQ(om.velocity_covariance[static_cast<std::size_t>(3 * r + c)], o.vel_cov(r, c));
  }
  EXPECT_EQ(om.quality, 87U);
}
