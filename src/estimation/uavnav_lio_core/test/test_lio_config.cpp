#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>
#include <limits>
#include <utility>
#include <string>
#include <string_view>

#include "uavnav/core/config.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio/limits.hpp"

using namespace uavnav;
using namespace uavnav::lio;
using Kind = config::ConfigError::Kind;

namespace {

constexpr std::string_view kBeta =
    "lifecycle_confirm_scans: 5\n"
    "lifecycle_degenerate_scans: 3\n"
    "lifecycle_gap_degraded_s: 0.25\n"
    "lifecycle_gap_lost_s: 0.5\n"
    "lifecycle_degeneracy_lost_s: 1.0\n"
    "lifecycle_position_sigma_lost_m: 0.5\n"
    "degeneracy_translation_min_info: 1.1e5\n"
    "degeneracy_rotation_min_info: 2.8e6\n"
    "predictor_tau_vel_s: 0.25\n"
    "predictor_tau_pos_s: 0.25\n"
    "extrinsic_imu_lidar_x_m: -0.011\n"
    "extrinsic_imu_lidar_y_m: -0.02329\n"
    "extrinsic_imu_lidar_z_m: 0.04412\n"
    "preprocess_min_range_m: 0.5\n"
    "preprocess_max_range_m: 40\n"
    "preprocess_voxel_m: 0.2\n"
    "map_voxel_m: 0.3\n"
    "map_half_extent_x_m: 30\n"
    "map_half_extent_y_m: 30\n"
    "map_half_extent_z_m: 15\n"
    "registration_max_iterations: 4\n"
    "imu_init_min_samples: 200\n"
    "imu_max_gap_s: 0.02\n";

// kBeta with each `key: value` pair of `changes` replacing the line that starts with `key:`.
std::string With(std::initializer_list<std::pair<std::string_view, std::string_view>> changes) {
  std::string out;
  std::string_view rest = kBeta;
  while (!rest.empty()) {
    const auto nl = rest.find('\n');
    const std::string_view line = rest.substr(0, nl);
    rest.remove_prefix(nl + 1);
    std::string replaced(line);
    for (const auto& [key, value] : changes) {
      if (line.starts_with(std::string(key) + ":")) replaced = std::string(key) + ": " + std::string(value);
    }
    out += replaced + "\n";
  }
  return out;
}

std::string With(std::string_view key, std::string_view value) { return With({{key, value}}); }

Result<LioConfig, config::ConfigError> Load(const std::string& text) {
  const auto values = config::load_params(text, kLioSpecs);
  if (!values) return std::unexpected(values.error());
  return load_lio_config(*values);
}

void ExpectError(const std::string& text, Kind kind, std::string_view key) {
  const auto r = Load(text);
  ASSERT_FALSE(r.has_value()) << "accepted: " << text;
  EXPECT_EQ(r.error().kind, kind) << to_string(r.error().kind) << " " << r.error().key << ": " << r.error().detail;
  EXPECT_EQ(r.error().key, key);
}

}  // namespace

TEST(LioConfig, BetaValuesLoad) {
  const auto r = Load(std::string(kBeta));
  ASSERT_TRUE(r.has_value()) << r.error().key << ": " << r.error().detail;
  const LifecycleConfig& c = r->lifecycle;
  EXPECT_EQ(c.confirm_scans, 5U);
  EXPECT_EQ(c.degenerate_scans, 3U);
  EXPECT_EQ(c.gap_degraded, time::milliseconds(250));
  EXPECT_EQ(c.gap_lost, time::milliseconds(500));
  EXPECT_EQ(c.degeneracy_lost, time::seconds(1));
  EXPECT_DOUBLE_EQ(c.position_sigma_lost_m, 0.5);
  EXPECT_DOUBLE_EQ(r->degeneracy.translation_min_info, 1.1e5);
  EXPECT_DOUBLE_EQ(r->degeneracy.rotation_min_info, 2.8e6);
  EXPECT_EQ(r->predictor.tau_vel, time::milliseconds(250));
  EXPECT_EQ(r->predictor.tau_pos, time::milliseconds(250));
}

TEST(LioConfig, SpecsAreValidAndComplete) {
  static_assert(kLioSpecs.size() == 23);
  // A schema with a bad spec would be rejected before the text is read.
  const auto r = config::load_params(kBeta, kLioSpecs);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->size(), kLioSpecs.size());
}

TEST(LioConfig, GapLostNotAboveGapDegradedNamesBothKeys) {
  const auto r = Load(With({{"lifecycle_gap_lost_s", "0.2"}, {"lifecycle_gap_degraded_s", "0.25"}}));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kOutOfRange);
  const std::string both = r.error().key + " | " + r.error().detail;
  EXPECT_NE(both.find("lifecycle_gap_lost_s"), std::string::npos) << both;
  EXPECT_NE(both.find("lifecycle_gap_degraded_s"), std::string::npos) << both;
  EXPECT_NE(r.error().detail.find("lifecycle_gap_lost_s"), std::string::npos);
  EXPECT_NE(r.error().detail.find("lifecycle_gap_degraded_s"), std::string::npos);
}

TEST(LioConfig, GapLostEqualToGapDegradedIsRejected) {
  ExpectError(With("lifecycle_gap_lost_s", "0.25"), Kind::kOutOfRange, "lifecycle_gap_lost_s");
}

TEST(LioConfig, GapLostJustAboveGapDegradedIsAccepted) {
  EXPECT_TRUE(Load(With("lifecycle_gap_lost_s", "0.26")).has_value());
}

TEST(LioConfig, DegenerateScansAboveTenTimesConfirmNamesBothKeys) {
  const auto r = Load(With({{"lifecycle_confirm_scans", "2"}, {"lifecycle_degenerate_scans", "21"}}));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kOutOfRange);
  EXPECT_NE(r.error().detail.find("lifecycle_degenerate_scans"), std::string::npos);
  EXPECT_NE(r.error().detail.find("lifecycle_confirm_scans"), std::string::npos);
  const std::string whole = r.error().key + " | " + r.error().detail;
  EXPECT_NE(whole.find("lifecycle_degenerate_scans"), std::string::npos);
  EXPECT_NE(whole.find("lifecycle_confirm_scans"), std::string::npos);
}

TEST(LioConfig, DegenerateScansExactlyTenTimesConfirmIsAccepted) {
  const auto r = Load(With({{"lifecycle_confirm_scans", "2"}, {"lifecycle_degenerate_scans", "20"}}));
  ASSERT_TRUE(r.has_value()) << r.error().key << ": " << r.error().detail;
  EXPECT_EQ(r->lifecycle.degenerate_scans, 20U);
}

TEST(LioConfig, UnknownKeyIsRejected) {
  ExpectError(std::string(kBeta) + "lifecycle_confirm_scan: 5\n", Kind::kUnknownKey, "lifecycle_confirm_scan");
}

TEST(LioConfig, MissingKeyIsRejected) {
  const std::string text = "lifecycle_confirm_scans: 5\n";
  const auto r = Load(text);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kMissingKey);
}

TEST(LioConfig, PerKeyBoundsAreEnforced) {
  ExpectError(With("lifecycle_confirm_scans", "0"), Kind::kOutOfRange, "lifecycle_confirm_scans");
  ExpectError(With("lifecycle_confirm_scans", "51"), Kind::kOutOfRange, "lifecycle_confirm_scans");
  ExpectError(With("lifecycle_gap_degraded_s", "0.04"), Kind::kOutOfRange, "lifecycle_gap_degraded_s");
  ExpectError(With("lifecycle_degeneracy_lost_s", "10.5"), Kind::kOutOfRange, "lifecycle_degeneracy_lost_s");
  ExpectError(With("lifecycle_position_sigma_lost_m", "0.01"), Kind::kOutOfRange, "lifecycle_position_sigma_lost_m");
}

TEST(LioConfig, DegeneracyKeysLoadAndAreRangeChecked) {
  const auto r = Load(With({{"degeneracy_translation_min_info", "2000"}, {"degeneracy_rotation_min_info", "3e7"}}));
  ASSERT_TRUE(r.has_value()) << r.error().key << ": " << r.error().detail;
  EXPECT_DOUBLE_EQ(r->degeneracy.translation_min_info, 2000.0);
  EXPECT_DOUBLE_EQ(r->degeneracy.rotation_min_info, 3e7);

  ExpectError(With("degeneracy_translation_min_info", "0.5"), Kind::kOutOfRange, "degeneracy_translation_min_info");
  ExpectError(With("degeneracy_translation_min_info", "1.1e9"), Kind::kOutOfRange, "degeneracy_translation_min_info");
  ExpectError(With("degeneracy_rotation_min_info", "0"), Kind::kOutOfRange, "degeneracy_rotation_min_info");
  ExpectError(With("degeneracy_rotation_min_info", "2e12"), Kind::kOutOfRange, "degeneracy_rotation_min_info");
}

TEST(LioConfig, DegeneracyKeyBoundsAreInclusive) {
  EXPECT_TRUE(Load(With({{"degeneracy_translation_min_info", "1"}, {"degeneracy_rotation_min_info", "1"}})).has_value());
  EXPECT_TRUE(
      Load(With({{"degeneracy_translation_min_info", "1e9"}, {"degeneracy_rotation_min_info", "1e12"}})).has_value());
}

TEST(LioConfig, MissingDegeneracyKeyIsRejectedNamingTheKey) {
  for (const std::string_view key : {"degeneracy_translation_min_info", "degeneracy_rotation_min_info"}) {
    std::string text;
    std::string_view rest = kBeta;
    while (!rest.empty()) {
      const auto nl = rest.find('\n');
      const std::string_view line = rest.substr(0, nl);
      rest.remove_prefix(nl + 1);
      if (!line.starts_with(std::string(key) + ":")) text += std::string(line) + "\n";
    }
    ExpectError(text, Kind::kMissingKey, key);
  }
}

TEST(LioConfig, NonIntegerCountIsRejected) {
  ExpectError(With("lifecycle_confirm_scans", "2.5"), Kind::kWrongType, "lifecycle_confirm_scans");
  ExpectError(With("lifecycle_degenerate_scans", "3.5"), Kind::kWrongType, "lifecycle_degenerate_scans");
}

TEST(LioConfig, LoadLioConfigReportsMissingKeyWithoutThrowing) {
  const config::ParamValues empty;
  const auto r = load_lio_config(empty);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kMissingKey);
}

TEST(LioConfig, PredictorKeysLoad) {
  const auto r = Load(With({{"predictor_tau_vel_s", "0.5"}, {"predictor_tau_pos_s", "1.5"}}));
  ASSERT_TRUE(r.has_value()) << r.error().key << ": " << r.error().detail;
  EXPECT_EQ(r->predictor.tau_vel, time::milliseconds(500));
  EXPECT_EQ(r->predictor.tau_pos, time::milliseconds(1500));
}

TEST(LioConfig, PredictorKeyBoundsAreInclusive) {
  const auto low = Load(With({{"predictor_tau_vel_s", "0.05"}, {"predictor_tau_pos_s", "0.05"}}));
  ASSERT_TRUE(low.has_value()) << low.error().key << ": " << low.error().detail;
  EXPECT_EQ(low->predictor.tau_vel, time::milliseconds(50));
  EXPECT_EQ(low->predictor.tau_pos, time::milliseconds(50));
  const auto high = Load(With({{"predictor_tau_vel_s", "5.0"}, {"predictor_tau_pos_s", "5.0"}}));
  ASSERT_TRUE(high.has_value()) << high.error().key << ": " << high.error().detail;
  EXPECT_EQ(high->predictor.tau_vel, time::seconds(5));
  EXPECT_EQ(high->predictor.tau_pos, time::seconds(5));
}

TEST(LioConfig, PredictorKeysOutOfRangeAreRejectedNamingTheKey) {
  for (const std::string_view key : {"predictor_tau_vel_s", "predictor_tau_pos_s"}) {
    ExpectError(With(key, "0.04"), Kind::kOutOfRange, key);
    ExpectError(With(key, "5.01"), Kind::kOutOfRange, key);
    ExpectError(With(key, "0"), Kind::kOutOfRange, key);
    ExpectError(With(key, "-0.25"), Kind::kOutOfRange, key);
  }
}

TEST(LioConfig, MissingPredictorKeyIsRejectedNamingTheKey) {
  for (const std::string_view key : {"predictor_tau_vel_s", "predictor_tau_pos_s"}) {
    std::string text;
    std::string_view rest = kBeta;
    while (!rest.empty()) {
      const auto nl = rest.find('\n');
      const std::string_view line = rest.substr(0, nl);
      rest.remove_prefix(nl + 1);
      if (!line.starts_with(std::string(key) + ":")) text += std::string(line) + "\n";
    }
    ExpectError(text, Kind::kMissingKey, key);
  }
}

TEST(LioConfig, MathKeysLoadWithBetaValues) {
  const auto r = Load(std::string(kBeta));
  ASSERT_TRUE(r.has_value()) << r.error().key << ": " << r.error().detail;
  const MathConfig& m = r->math;
  EXPECT_DOUBLE_EQ(m.t_imu_lidar_m.x(), -0.011);
  EXPECT_DOUBLE_EQ(m.t_imu_lidar_m.y(), -0.02329);
  EXPECT_DOUBLE_EQ(m.t_imu_lidar_m.z(), 0.04412);
  EXPECT_DOUBLE_EQ(m.preprocess_min_range_m, 0.5);
  EXPECT_DOUBLE_EQ(m.preprocess_max_range_m, 40.0);
  EXPECT_DOUBLE_EQ(m.preprocess_voxel_m, 0.2);
  EXPECT_DOUBLE_EQ(m.map_voxel_m, 0.3);
  EXPECT_DOUBLE_EQ(m.map_half_extent_m.x(), 30.0);
  EXPECT_DOUBLE_EQ(m.map_half_extent_m.y(), 30.0);
  EXPECT_DOUBLE_EQ(m.map_half_extent_m.z(), 15.0);
  EXPECT_EQ(m.registration_max_iterations, 4U);
  EXPECT_EQ(m.imu_init_min_samples, 200U);
  EXPECT_EQ(m.imu_max_gap, time::milliseconds(20));
}

TEST(LioConfig, MathKeyBoundsAreEnforced) {
  ExpectError(With("registration_max_iterations", "0"), Kind::kOutOfRange, "registration_max_iterations");
  ExpectError(With("registration_max_iterations", "11"), Kind::kOutOfRange, "registration_max_iterations");
  ExpectError(With("registration_max_iterations", "2.5"), Kind::kWrongType, "registration_max_iterations");
  ExpectError(With("imu_init_min_samples", "801"), Kind::kOutOfRange, "imu_init_min_samples");
  ExpectError(With("imu_max_gap_s", "0.1"), Kind::kOutOfRange, "imu_max_gap_s");
  ExpectError(With("map_voxel_m", "0.05"), Kind::kOutOfRange, "map_voxel_m");
  ExpectError(With("extrinsic_imu_lidar_z_m", "0.5"), Kind::kOutOfRange, "extrinsic_imu_lidar_z_m");
  EXPECT_TRUE(Load(With({{"registration_max_iterations", "1"}})).has_value());
  EXPECT_TRUE(Load(With({{"registration_max_iterations", "10"}})).has_value());
}

// The tracked default file config/lio/sim.yaml (path from a compile definition, see CMakeLists.txt) holds
// every kLioSpecs key and loads without error.
TEST(LioConfig, DefaultSimYamlLoadsAndHasEveryKey) {
  const auto values = config::load_params_file(UAVNAV_LIO_DEFAULT_YAML, kLioSpecs);
  ASSERT_TRUE(values.has_value()) << UAVNAV_LIO_DEFAULT_YAML << ": " << values.error().key << ": "
                                  << values.error().detail;
  for (const config::ParamSpec& spec : kLioSpecs) {
    EXPECT_TRUE(values->contains(spec.key)) << spec.key;
  }
  const auto cfg = load_lio_config(*values);
  ASSERT_TRUE(cfg.has_value()) << cfg.error().key << ": " << cfg.error().detail;
  // Same values as the beta text used by the other tests.
  const auto beta = config::load_params(kBeta, kLioSpecs);
  ASSERT_TRUE(beta.has_value());
  EXPECT_EQ(*values, *beta);
}

TEST(LioLimits, ScanPointCap) { EXPECT_EQ(limits::kMaxScanPoints, 200'000U); }

TEST(LioLimits, PredictorConstants) {
  EXPECT_EQ(limits::kPredictorBufferSpan, time::milliseconds(300));
  EXPECT_EQ(limits::kPredictorBufferCapacity, 64U);
  EXPECT_EQ(limits::kPredictorDtMin, time::nanoseconds(100'000));
  EXPECT_EQ(limits::kPredictorDtMax, time::milliseconds(30));
  // The buffer holds at least the full span at the design IMU rate (200 Hz).
  EXPECT_GE(static_cast<std::int64_t>(limits::kPredictorBufferCapacity - 1) * limits::kPredictorDesignImuPeriod.ns,
            limits::kPredictorBufferSpan.ns);
}

// --- hand-built ParamValues (load_lio_config is public) ----------------------------------------

namespace {

config::ParamValues Beta() {
  const auto values = config::load_params(kBeta, kLioSpecs);
  EXPECT_TRUE(values.has_value());
  return *values;
}

}  // namespace

TEST(LioConfigHandBuilt, CountKeysRejectHostileValuesWithoutUndefinedCasts) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  struct Case { double value; Kind kind; };
  for (const std::string_view key : {"lifecycle_confirm_scans", "lifecycle_degenerate_scans"}) {
    for (const Case c : {Case{1e30, Kind::kOutOfRange}, Case{-3.0, Kind::kOutOfRange}, Case{0.0, Kind::kOutOfRange},
                         Case{51.0, Kind::kOutOfRange}, Case{2.5, Kind::kWrongType}, Case{nan, Kind::kNotFinite},
                         Case{inf, Kind::kNotFinite}, Case{-inf, Kind::kNotFinite}}) {
      auto values = Beta();
      values[std::string(key)] = c.value;
      const auto r = load_lio_config(values);
      ASSERT_FALSE(r.has_value()) << key << " = " << c.value;
      EXPECT_EQ(r.error().kind, c.kind) << key << " = " << c.value << ": " << r.error().detail;
      EXPECT_EQ(r.error().key, key) << c.value;
    }
  }
}

TEST(LioConfigHandBuilt, SecondsAndMetersKeysAreRangeCheckedToo) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const std::string_view key : {"lifecycle_gap_degraded_s", "lifecycle_gap_lost_s", "lifecycle_degeneracy_lost_s",
                                     "lifecycle_position_sigma_lost_m"}) {
    for (const double bad : {-1.0, 1e30, nan}) {
      auto values = Beta();
      values[std::string(key)] = bad;
      const auto r = load_lio_config(values);
      ASSERT_FALSE(r.has_value()) << key << " = " << bad;
      EXPECT_EQ(r.error().key, key);
    }
  }
}

TEST(LioConfigHandBuilt, BoundsAreInclusive) {
  auto values = Beta();
  values["lifecycle_confirm_scans"] = 1.0;
  values["lifecycle_degenerate_scans"] = 10.0;
  EXPECT_TRUE(load_lio_config(values).has_value());
  values["lifecycle_confirm_scans"] = 50.0;
  values["lifecycle_degenerate_scans"] = 50.0;
  EXPECT_TRUE(load_lio_config(values).has_value());
}

TEST(LioConfigHandBuilt, MissingDegeneracyKeyInHandBuiltValuesNamesTheKey) {
  for (const std::string_view key : {"degeneracy_translation_min_info", "degeneracy_rotation_min_info"}) {
    auto values = Beta();
    values.erase(std::string(key));
    const auto r = load_lio_config(values);
    ASSERT_FALSE(r.has_value()) << key;
    EXPECT_EQ(r.error().kind, Kind::kMissingKey);
    EXPECT_EQ(r.error().key, key);
  }
}

TEST(LioConfigHandBuilt, PredictorKeysRejectHostileValues) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const std::string_view key : {"predictor_tau_vel_s", "predictor_tau_pos_s"}) {
    for (const double bad : {-1.0, 0.0, 0.0499, 5.0001, 1e30, nan, inf, -inf}) {
      auto values = Beta();
      values[std::string(key)] = bad;
      const auto r = load_lio_config(values);
      ASSERT_FALSE(r.has_value()) << key << " = " << bad;
      EXPECT_EQ(r.error().key, key);
    }
    auto values = Beta();
    values.erase(std::string(key));
    const auto r = load_lio_config(values);
    ASSERT_FALSE(r.has_value()) << key;
    EXPECT_EQ(r.error().kind, Kind::kMissingKey);
    EXPECT_EQ(r.error().key, key);
  }
}

TEST(LioConfigHandBuilt, DegeneracyKeysRejectHostileValues) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const std::string_view key : {"degeneracy_translation_min_info", "degeneracy_rotation_min_info"}) {
    for (const double bad : {-1.0, 0.0, 1e30, nan, inf, -inf}) {
      auto values = Beta();
      values[std::string(key)] = bad;
      const auto r = load_lio_config(values);
      ASSERT_FALSE(r.has_value()) << key << " = " << bad;
      EXPECT_EQ(r.error().key, key);
    }
  }
}
