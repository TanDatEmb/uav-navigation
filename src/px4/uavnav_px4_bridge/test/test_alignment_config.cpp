#include <gtest/gtest.h>

#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

#include "uavnav/core/config.hpp"
#include "uavnav/px4bridge/config.hpp"
#include "uavnav/px4bridge/limits.hpp"

using namespace uavnav;
using namespace uavnav::px4bridge;
using Kind = config::ConfigError::Kind;

namespace {

constexpr std::string_view kBeta =
    "alignment_tau_s: 2.0\n"
    "alignment_consistent_pairs: 20\n"
    "alignment_jump_position_m: 0.5\n"
    "alignment_jump_yaw_rad: 0.0873\n"
    "alignment_max_rate_mps: 0.5\n"
    "alignment_max_yaw_rate_rad_s: 0.0873\n"
    "alignment_valid_stale_s: 1.0\n"
    "alignment_frozen_max_s: 10.0\n";

// kBeta with each `key: value` of `changes` replacing the line that starts with `key:`.
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

Result<AlignmentConfig, config::ConfigError> Load(const std::string& text) {
  const auto values = config::load_params(text, kAlignmentSpecs);
  if (!values) return std::unexpected(values.error());
  return load_alignment_config(*values);
}

void ExpectError(const std::string& text, Kind kind, std::string_view key) {
  const auto r = Load(text);
  ASSERT_FALSE(r.has_value()) << "accepted: " << text;
  EXPECT_EQ(r.error().kind, kind) << to_string(r.error().kind) << " " << r.error().key << ": " << r.error().detail;
  EXPECT_EQ(r.error().key, key);
}

}  // namespace

TEST(AlignmentConfig, BetaValuesLoad) {
  const auto r = Load(std::string(kBeta));
  ASSERT_TRUE(r.has_value()) << r.error().key << ": " << r.error().detail;
  EXPECT_EQ(r->tau, time::seconds(2));
  EXPECT_EQ(r->consistent_pairs, 20U);
  EXPECT_DOUBLE_EQ(r->jump_position_m, 0.5);
  EXPECT_DOUBLE_EQ(r->jump_yaw_rad, 0.0873);
  EXPECT_DOUBLE_EQ(r->max_rate_mps, 0.5);
  EXPECT_DOUBLE_EQ(r->max_yaw_rate_rad_s, 0.0873);
  EXPECT_EQ(r->valid_stale, time::seconds(1));
  EXPECT_EQ(r->frozen_max, time::seconds(10));
}

TEST(AlignmentConfig, SpecsAreValidAndComplete) {
  static_assert(kAlignmentSpecs.size() == 8);
  const auto r = config::load_params(kBeta, kAlignmentSpecs);
  ASSERT_TRUE(r.has_value()) << r.error().key << ": " << r.error().detail;
  EXPECT_EQ(r->size(), kAlignmentSpecs.size());
}

TEST(AlignmentConfig, SpecBoundsMatchTheBrief) {
  struct Expected {
    std::string_view key;
    config::Unit unit;
    double min, max;
  };
  constexpr Expected kExpected[] = {
      {"alignment_tau_s", config::Unit::kSeconds, 0.2, 20.0},
      {"alignment_consistent_pairs", config::Unit::kNone, 1.0, 200.0},
      {"alignment_jump_position_m", config::Unit::kMeters, 0.05, 5.0},
      {"alignment_jump_yaw_rad", config::Unit::kRadians, 0.01, 0.5},
      {"alignment_max_rate_mps", config::Unit::kMetersPerSecond, 0.01, 5.0},
      {"alignment_max_yaw_rate_rad_s", config::Unit::kRadiansPerSecond, 0.001, 0.5},
      {"alignment_valid_stale_s", config::Unit::kSeconds, 0.2, 10.0},
      {"alignment_frozen_max_s", config::Unit::kSeconds, 1.0, 120.0},
  };
  ASSERT_EQ(std::size(kExpected), kAlignmentSpecs.size());
  for (std::size_t i = 0; i < kAlignmentSpecs.size(); ++i) {
    EXPECT_EQ(kAlignmentSpecs[i].key, kExpected[i].key);
    EXPECT_EQ(kAlignmentSpecs[i].unit, kExpected[i].unit) << kExpected[i].key;
    EXPECT_EQ(kAlignmentSpecs[i].min, kExpected[i].min) << kExpected[i].key;
    EXPECT_EQ(kAlignmentSpecs[i].max, kExpected[i].max) << kExpected[i].key;
  }
}

TEST(AlignmentConfig, PerKeyBoundsAreEnforced) {
  struct Case {
    std::string_view key, below, at_min, at_max, above;
  };
  constexpr Case kCases[] = {
      {"alignment_tau_s", "0.19", "0.2", "20", "20.1"},
      {"alignment_consistent_pairs", "0", "1", "200", "201"},
      {"alignment_jump_position_m", "0.04", "0.05", "5", "5.01"},
      {"alignment_jump_yaw_rad", "0.009", "0.01", "0.5", "0.51"},
      {"alignment_max_rate_mps", "0.009", "0.01", "5", "5.1"},
      {"alignment_max_yaw_rate_rad_s", "0.0009", "0.001", "0.5", "0.51"},
      {"alignment_valid_stale_s", "0.19", "0.2", "10", "10.1"},
      {"alignment_frozen_max_s", "0.99", "1", "120", "120.1"},
  };
  for (const Case& c : kCases) {
    // valid_stale 0.5 s and frozen_max 60 s unless under test, so the cross check never interferes.
    auto text = [&c](std::string_view v) {
      return With({{c.key, v},
                   {"alignment_valid_stale_s", c.key == "alignment_valid_stale_s" ? v : "0.5"},
                   {"alignment_frozen_max_s", c.key == "alignment_frozen_max_s" ? v : "60"}});
    };
    ExpectError(text(c.below), Kind::kOutOfRange, c.key);
    ExpectError(text(c.above), Kind::kOutOfRange, c.key);
    EXPECT_TRUE(Load(text(c.at_min)).has_value()) << c.key << " at min";
    EXPECT_TRUE(Load(text(c.at_max)).has_value()) << c.key << " at max";
  }
}

TEST(AlignmentConfig, UnknownKeyIsRejected) {
  ExpectError(std::string(kBeta) + "alignment_tau: 2\n", Kind::kUnknownKey, "alignment_tau");
}

TEST(AlignmentConfig, MissingKeyIsRejected) {
  const auto r = Load("alignment_tau_s: 2.0\n");
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kMissingKey);
}

TEST(AlignmentConfig, MissingKeyInHandBuiltValuesIsRejected) {
  auto values = config::load_params(kBeta, kAlignmentSpecs);
  ASSERT_TRUE(values.has_value());
  values->erase("alignment_jump_yaw_rad");
  const auto r = load_alignment_config(*values);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kMissingKey);
  EXPECT_EQ(r.error().key, "alignment_jump_yaw_rad");
}

TEST(AlignmentConfig, HandBuiltValuesAreReChecked) {
  auto values = config::load_params(kBeta, kAlignmentSpecs);
  ASSERT_TRUE(values.has_value());
  auto out_of_range = *values;
  out_of_range["alignment_tau_s"] = 0.0;
  auto r = load_alignment_config(out_of_range);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kOutOfRange);
  EXPECT_EQ(r.error().key, "alignment_tau_s");
  auto nan = *values;
  nan["alignment_jump_position_m"] = std::numeric_limits<double>::quiet_NaN();
  r = load_alignment_config(nan);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kNotFinite);
  auto huge = *values;
  huge["alignment_consistent_pairs"] = 1e30;
  r = load_alignment_config(huge);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kOutOfRange);
}

TEST(AlignmentConfig, ConsistentPairsMustBeWhole) {
  ExpectError(With("alignment_consistent_pairs", "20.5"), Kind::kWrongType, "alignment_consistent_pairs");
}

TEST(AlignmentConfig, NonNumericAndNonFiniteAreRejected) {
  ExpectError(With("alignment_tau_s", "\"2\""), Kind::kWrongType, "alignment_tau_s");
  ExpectError(With("alignment_max_rate_mps", ".nan"), Kind::kNotFinite, "alignment_max_rate_mps");
}

TEST(AlignmentConfig, FrozenMaxNotAboveValidStaleNamesBothKeys) {
  const auto r = Load(With({{"alignment_frozen_max_s", "2"}, {"alignment_valid_stale_s", "2"}}));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kOutOfRange);
  EXPECT_EQ(r.error().key, "alignment_frozen_max_s");
  EXPECT_NE(r.error().detail.find("alignment_frozen_max_s"), std::string::npos) << r.error().detail;
  EXPECT_NE(r.error().detail.find("alignment_valid_stale_s"), std::string::npos) << r.error().detail;
}

TEST(AlignmentConfig, FrozenMaxJustAboveValidStaleIsAccepted) {
  EXPECT_TRUE(Load(With({{"alignment_frozen_max_s", "2.01"}, {"alignment_valid_stale_s", "2"}})).has_value());
}

TEST(AlignmentConfig, TierAInvariantsHoldForEveryLoadableConfig) {
  // alpha = dt / tau <= 1 for every dt the filter can see (dt is clamped to kFilterDtMax).
  static_assert(find_alignment_spec("alignment_tau_s")->min * 1e9 >= static_cast<double>(limits::kFilterDtMax.ns));
  // The INIT accumulation is a fixed array sized for the largest allowed consistent_pairs.
  static_assert(find_alignment_spec("alignment_consistent_pairs")->max ==
                static_cast<double>(limits::kMaxConsistentPairs));
  static_assert(find_alignment_spec("alignment_no_such_key") == nullptr);
  SUCCEED();
}
