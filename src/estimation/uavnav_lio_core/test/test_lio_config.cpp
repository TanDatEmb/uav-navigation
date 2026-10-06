#include <gtest/gtest.h>

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
    "lifecycle_position_sigma_lost_m: 0.5\n";

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
}

TEST(LioConfig, SpecsAreValidAndComplete) {
  static_assert(kLioSpecs.size() == 6);
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

TEST(LioLimits, ScanPointCap) { EXPECT_EQ(limits::kMaxScanPoints, 200'000U); }
