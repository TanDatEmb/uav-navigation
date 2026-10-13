// The lio node's tier-(b) mount table (D33/O20) and the base_T_imu composition.
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <string>

#include <Eigen/Geometry>

#include "uavnav/core/config.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio_node/node_config.hpp"

using namespace uavnav;
using namespace uavnav::lio_node;
using Kind = config::ConfigError::Kind;

namespace {

lio::LioConfig SimLioConfig() {
  const auto values = config::load_params_file(UAVNAV_LIO_DEFAULT_YAML, lio::kLioSpecs);
  EXPECT_TRUE(values.has_value());
  const auto cfg = lio::load_lio_config(*values);
  EXPECT_TRUE(cfg.has_value());
  return *cfg;
}

const char* kValidYaml =
    "extrinsic_base_lidar_x_m: 0.1\n"
    "extrinsic_base_lidar_y_m: -0.2\n"
    "extrinsic_base_lidar_z_m: 0.3\n"
    "extrinsic_base_lidar_roll_rad: 0.0\n"
    "extrinsic_base_lidar_pitch_rad: 0.0\n"
    "extrinsic_base_lidar_yaw_rad: 0.0\n";

config::ParamValues Values(double roll, double pitch, double yaw) {
  return config::ParamValues{{"extrinsic_base_lidar_x_m", 0.1},       {"extrinsic_base_lidar_y_m", -0.2},
                             {"extrinsic_base_lidar_z_m", 0.3},       {"extrinsic_base_lidar_roll_rad", roll},
                             {"extrinsic_base_lidar_pitch_rad", pitch}, {"extrinsic_base_lidar_yaw_rad", yaw}};
}

}  // namespace

TEST(LioNodeConfig, SimYamlLoadsAndComposesBaseTImu) {
  const auto values = config::load_params_file(UAVNAV_LIO_NODE_YAML, kLioNodeSpecs);
  ASSERT_TRUE(values.has_value()) << values.error().key << ": " << values.error().detail;
  const auto node = load_lio_node_config(*values);
  ASSERT_TRUE(node.has_value()) << node.error().detail;
  // x500_mid360/model.sdf: livox_frame 0.28 m above base_link, axes aligned.
  EXPECT_TRUE(node->base_T_lidar.translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.28), 1e-12));
  EXPECT_TRUE(node->base_T_lidar.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12));

  const lio::LioConfig lio = SimLioConfig();
  const Eigen::Isometry3d base_T_imu = compose_base_T_imu(*node, lio);
  // lidar_mid360/model.sdf: the IMU at (0.011, 0.02329, -0.04412) in livox_frame.
  const Eigen::Vector3d expected(0.011, 0.02329, 0.28 - 0.04412);
  EXPECT_LT((base_T_imu.translation() - expected).norm(), 1e-12) << base_T_imu.translation().transpose();
  EXPECT_TRUE(base_T_imu.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  // The direction convention: base_T_imu * imu_T_lidar == base_T_lidar.
  EXPECT_TRUE((base_T_imu * imu_T_lidar(lio)).matrix().isApprox(node->base_T_lidar.matrix(), 1e-12));
  EXPECT_TRUE(imu_T_lidar(lio).translation().isApprox(Eigen::Vector3d(-0.011, -0.02329, 0.04412), 1e-12));
}

TEST(LioNodeConfig, AnglesAreFixedAxisRollPitchYaw) {
  const double roll = 0.2, pitch = -0.4, yaw = 1.1;
  const auto node = load_lio_node_config(Values(roll, pitch, yaw));
  ASSERT_TRUE(node.has_value()) << node.error().detail;
  const Eigen::Matrix3d expected = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                                       .toRotationMatrix();
  EXPECT_TRUE(node->base_T_lidar.linear().isApprox(expected, 1e-12));
  EXPECT_TRUE(node->base_T_lidar.translation().isApprox(Eigen::Vector3d(0.1, -0.2, 0.3), 1e-12));
  // A LiDAR yawed by +90 deg sees its x axis along base y.
  const auto yawed = load_lio_node_config(Values(0.0, 0.0, std::numbers::pi / 2.0));
  ASSERT_TRUE(yawed.has_value());
  EXPECT_TRUE((yawed->base_T_lidar.linear() * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitY(), 1e-12));
}

TEST(LioNodeConfig, UnknownAndOutOfRangeKeysAreRejected) {
  ASSERT_TRUE(config::load_params(kValidYaml, kLioNodeSpecs).has_value());

  const auto unknown = config::load_params(std::string(kValidYaml) + "extrinsic_base_lidar_w_m: 0.0\n", kLioNodeSpecs);
  ASSERT_FALSE(unknown.has_value());
  EXPECT_EQ(unknown.error().kind, Kind::kUnknownKey);
  EXPECT_EQ(unknown.error().key, "extrinsic_base_lidar_w_m");

  std::string yaw4(kValidYaml);
  yaw4.replace(yaw4.find("extrinsic_base_lidar_yaw_rad: 0.0"), 33, "extrinsic_base_lidar_yaw_rad: 4.0");
  const auto out_of_range = config::load_params(yaw4, kLioNodeSpecs);
  ASSERT_FALSE(out_of_range.has_value());
  EXPECT_EQ(out_of_range.error().kind, Kind::kOutOfRange);
  EXPECT_EQ(out_of_range.error().key, "extrinsic_base_lidar_yaw_rad");

  std::string missing(kValidYaml);
  missing.erase(missing.find("extrinsic_base_lidar_z_m"));
  missing += "extrinsic_base_lidar_roll_rad: 0.0\nextrinsic_base_lidar_pitch_rad: 0.0\nextrinsic_base_lidar_yaw_rad: 0.0\n";
  const auto absent = config::load_params(missing, kLioNodeSpecs);
  ASSERT_FALSE(absent.has_value());
  EXPECT_EQ(absent.error().kind, Kind::kMissingKey);
  EXPECT_EQ(absent.error().key, "extrinsic_base_lidar_z_m");

  // The loader re-checks hand-built values: out of range, not finite, missing.
  const auto hand_yaw = load_lio_node_config(Values(0.0, 0.0, 4.0));
  ASSERT_FALSE(hand_yaw.has_value());
  EXPECT_EQ(hand_yaw.error().kind, Kind::kOutOfRange);
  EXPECT_EQ(hand_yaw.error().key, "extrinsic_base_lidar_yaw_rad");
  const auto hand_pitch = load_lio_node_config(Values(0.0, 1.6, 0.0));
  ASSERT_FALSE(hand_pitch.has_value());
  EXPECT_EQ(hand_pitch.error().key, "extrinsic_base_lidar_pitch_rad");
  const auto hand_nan = load_lio_node_config(Values(std::nan(""), 0.0, 0.0));
  ASSERT_FALSE(hand_nan.has_value());
  EXPECT_EQ(hand_nan.error().kind, Kind::kNotFinite);
  config::ParamValues no_x = Values(0.0, 0.0, 0.0);
  no_x.erase("extrinsic_base_lidar_x_m");
  const auto hand_missing = load_lio_node_config(no_x);
  ASSERT_FALSE(hand_missing.has_value());
  EXPECT_EQ(hand_missing.error().kind, Kind::kMissingKey);
  EXPECT_EQ(hand_missing.error().key, "extrinsic_base_lidar_x_m");
}
