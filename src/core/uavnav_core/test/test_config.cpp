#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "uavnav/core/config.hpp"

using namespace uavnav::config;
using Kind = ConfigError::Kind;

namespace {

constexpr ParamSpec kSpecs[] = {{"cruise_speed_mps", Unit::kMetersPerSecond, 0.5, 5.0},
                                {"lio_recovery_timeout_s", Unit::kSeconds, 1.0, 60.0}};

constexpr std::string_view kValid = "cruise_speed_mps: 3.0\nlio_recovery_timeout_s: 10\n";

// Loads `text` against kSpecs and expects the failure `kind` naming `key`.
void ExpectError(std::string_view text, Kind kind, std::string_view key) {
  const auto r = load_params(text, kSpecs);
  ASSERT_FALSE(r.has_value()) << "accepted: " << text;
  EXPECT_EQ(r.error().kind, kind) << "text: " << text << " got " << to_string(r.error().kind)
                                  << " key '" << r.error().key << "' detail: " << r.error().detail;
  EXPECT_EQ(r.error().key, key) << "text: " << text;
  EXPECT_FALSE(r.error().detail.empty()) << "text: " << text;
}

// A single-spec schema with wide bounds, used to probe scalar conversion.
uavnav::Result<ParamValues, ConfigError> LoadOne(std::string_view value_text) {
  static constexpr ParamSpec kOne[] = {{"gain", Unit::kNone, -1e6, 1e6}};
  return load_params(std::string("gain: ") + std::string(value_text) + "\n", kOne);
}

}  // namespace

TEST(Config, SuffixTable) {
  EXPECT_EQ(suffix(Unit::kNone), "");
  EXPECT_EQ(suffix(Unit::kSeconds), "_s");
  EXPECT_EQ(suffix(Unit::kMeters), "_m");
  EXPECT_EQ(suffix(Unit::kMetersPerSecond), "_mps");
  EXPECT_EQ(suffix(Unit::kMetersPerSecond2), "_mps2");
  EXPECT_EQ(suffix(Unit::kMetersPerSecond3), "_mps3");
  EXPECT_EQ(suffix(Unit::kRadians), "_rad");
  EXPECT_EQ(suffix(Unit::kRadiansPerSecond), "_rad_s");
  EXPECT_EQ(suffix(Unit::kHertz), "_hz");
}

TEST(Config, KindNames) {
  EXPECT_EQ(to_string(Kind::kFileUnreadable), "FILE_UNREADABLE");
  EXPECT_EQ(to_string(Kind::kParse), "PARSE");
  EXPECT_EQ(to_string(Kind::kNotFlatMap), "NOT_FLAT_MAP");
  EXPECT_EQ(to_string(Kind::kDuplicateKey), "DUPLICATE_KEY");
  EXPECT_EQ(to_string(Kind::kUnknownKey), "UNKNOWN_KEY");
  EXPECT_EQ(to_string(Kind::kMissingKey), "MISSING_KEY");
  EXPECT_EQ(to_string(Kind::kWrongType), "WRONG_TYPE");
  EXPECT_EQ(to_string(Kind::kNotFinite), "NOT_FINITE");
  EXPECT_EQ(to_string(Kind::kOutOfRange), "OUT_OF_RANGE");
  EXPECT_EQ(to_string(Kind::kBadSpec), "BAD_SPEC");
  static_assert(uavnav::ReasonEnum<Kind>);
}

TEST(Config, LoadsValidFlatMap) {
  const auto r = load_params(kValid, kSpecs);
  ASSERT_TRUE(r.has_value()) << r.error().key << ": " << r.error().detail;
  ASSERT_EQ(r->size(), 2U);
  EXPECT_DOUBLE_EQ(r->at("cruise_speed_mps"), 3.0);
  EXPECT_DOUBLE_EQ(r->at("lio_recovery_timeout_s"), 10.0);
}

TEST(Config, LoadsKeysInAnyOrderWithCommentsAndFlowMap) {
  const auto r = load_params("# tuning\nlio_recovery_timeout_s: 2 # seconds\ncruise_speed_mps: 1.5\n", kSpecs);
  ASSERT_TRUE(r.has_value());
  EXPECT_DOUBLE_EQ(r->at("cruise_speed_mps"), 1.5);
  EXPECT_DOUBLE_EQ(r->at("lio_recovery_timeout_s"), 2.0);
  const auto flow = load_params("{cruise_speed_mps: 1.5, lio_recovery_timeout_s: 2}", kSpecs);
  ASSERT_TRUE(flow.has_value());
}

TEST(Config, BoundsAreInclusive) {
  const auto lo = load_params("cruise_speed_mps: 0.5\nlio_recovery_timeout_s: 1\n", kSpecs);
  ASSERT_TRUE(lo.has_value());
  const auto hi = load_params("cruise_speed_mps: 5\nlio_recovery_timeout_s: 60\n", kSpecs);
  ASSERT_TRUE(hi.has_value());
  ExpectError("cruise_speed_mps: 5.000001\nlio_recovery_timeout_s: 10\n", Kind::kOutOfRange, "cruise_speed_mps");
  ExpectError("cruise_speed_mps: 0.499999\nlio_recovery_timeout_s: 10\n", Kind::kOutOfRange, "cruise_speed_mps");
}

TEST(Config, RejectsOutOfRangeNamingKeyAndBounds) {
  const auto r = load_params("cruise_speed_mps: 7.0\nlio_recovery_timeout_s: 10\n", kSpecs);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kOutOfRange);
  EXPECT_EQ(r.error().key, "cruise_speed_mps");
  EXPECT_NE(r.error().detail.find('7'), std::string::npos) << r.error().detail;
  EXPECT_NE(r.error().detail.find("0.5"), std::string::npos) << r.error().detail;
  EXPECT_NE(r.error().detail.find('5'), std::string::npos) << r.error().detail;
}

TEST(Config, RejectsMissingKey) {
  ExpectError("cruise_speed_mps: 3\n", Kind::kMissingKey, "lio_recovery_timeout_s");
}

TEST(Config, RejectsUnknownKey) {
  ExpectError("cruise_speed_mps: 3\nlio_recovery_timeout_s: 10\ncruise_sped_mps: 1\n", Kind::kUnknownKey,
              "cruise_sped_mps");
}

TEST(Config, UnknownKeyIsReportedBeforeMissingKey) {
  // The misspelling is the root cause of the missing key, so it is named first.
  ExpectError("cruise_sped_mps: 3\nlio_recovery_timeout_s: 10\n", Kind::kUnknownKey, "cruise_sped_mps");
}

TEST(Config, RejectsDuplicateKey) {
  ExpectError("cruise_speed_mps: 3\ncruise_speed_mps: 4\nlio_recovery_timeout_s: 10\n", Kind::kDuplicateKey,
              "cruise_speed_mps");
  // Non-adjacent duplicate, and in flow style.
  ExpectError("cruise_speed_mps: 3\nlio_recovery_timeout_s: 10\ncruise_speed_mps: 3\n", Kind::kDuplicateKey,
              "cruise_speed_mps");
  ExpectError("{cruise_speed_mps: 3, lio_recovery_timeout_s: 10, lio_recovery_timeout_s: 10}", Kind::kDuplicateKey,
              "lio_recovery_timeout_s");
}

TEST(Config, DuplicateKeyWinsOverLaterRules) {
  // Duplicate (rule 3) is reported before the unknown key (rule 4) and bad value (rule 5).
  ExpectError("bogus: 1\ncruise_speed_mps: 99\ncruise_speed_mps: 1\n", Kind::kDuplicateKey, "cruise_speed_mps");
}

TEST(Config, RejectsWrongTypeAndNonFinite) {
  ExpectError("cruise_speed_mps: fast\nlio_recovery_timeout_s: 10\n", Kind::kWrongType, "cruise_speed_mps");
  ExpectError("cruise_speed_mps: .nan\nlio_recovery_timeout_s: 10\n", Kind::kNotFinite, "cruise_speed_mps");
  ExpectError("cruise_speed_mps: 3\nlio_recovery_timeout_s: .inf\n", Kind::kNotFinite, "lio_recovery_timeout_s");
  ExpectError("cruise_speed_mps: -.inf\nlio_recovery_timeout_s: 10\n", Kind::kNotFinite, "cruise_speed_mps");
}

TEST(Config, RejectsNestedMap) {
  ExpectError("planner:\n  cruise_speed_mps: 1\n", Kind::kNotFlatMap, "planner");
  ExpectError("cruise_speed_mps: 3\nlio_recovery_timeout_s: 10\nextra: {a: 1}\n", Kind::kNotFlatMap, "extra");
}

TEST(Config, RejectsSequenceValue) {
  ExpectError("cruise_speed_mps: [1, 2]\nlio_recovery_timeout_s: 10\n", Kind::kNotFlatMap, "cruise_speed_mps");
}

TEST(Config, RejectsNonMapDocuments) {
  ExpectError("- 1\n- 2\n", Kind::kNotFlatMap, "");
  ExpectError("3.0\n", Kind::kNotFlatMap, "");
  ExpectError("hello\n", Kind::kNotFlatMap, "");
}

TEST(Config, RejectsEmptyWhitespaceAndCommentOnlyDocuments) {
  for (const std::string_view text : {"", "   ", "\n\n", "# only a comment\n", "---\n", "~", "null"}) {
    const auto r = load_params(text, kSpecs);
    ASSERT_FALSE(r.has_value()) << "accepted: '" << text << "'";
    EXPECT_EQ(r.error().kind, Kind::kNotFlatMap) << "text: '" << text << "'";
    EXPECT_FALSE(r.error().detail.empty());
  }
}

TEST(Config, RejectsMultipleDocuments) {
  const auto r = load_params(std::string(kValid) + "---\n" + std::string(kValid), kSpecs);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kParse);
}

TEST(Config, RejectsMalformedYamlWithoutThrowing) {
  for (const std::string_view text : {"cruise_speed_mps: [1, 2\n", "a: b: c\n", "{unterminated", "\t- x\n: :\n",
                                      "a: !!bogus [\n"}) {
    const auto r = load_params(text, kSpecs);
    ASSERT_FALSE(r.has_value()) << "accepted: '" << text << "'";
    EXPECT_TRUE(r.error().kind == Kind::kParse || r.error().kind == Kind::kNotFlatMap)
        << "text: '" << text << "' got " << to_string(r.error().kind);
  }
  ExpectError("cruise_speed_mps: [1, 2\n", Kind::kParse, "");
}

TEST(Config, UnterminatedQuoteNeverLoadsAsANumber) {
  // yaml-cpp tolerates an unterminated quote at end of input; it must still never load as a number.
  for (const std::string_view text : {"lio_recovery_timeout_s: 10\ncruise_speed_mps: \"3",
                                      "lio_recovery_timeout_s: 10\ncruise_speed_mps: \"3\n"}) {
    const auto r = load_params(text, kSpecs);
    ASSERT_FALSE(r.has_value()) << text;
    EXPECT_TRUE(r.error().kind == Kind::kParse || r.error().kind == Kind::kWrongType) << to_string(r.error().kind);
  }
}

TEST(Config, BareDashValueIsRejected) {
  const auto r = LoadOne("-");  // "gain: -" is a (broken) sequence entry for the YAML parser
  ASSERT_FALSE(r.has_value());
  EXPECT_TRUE(r.error().kind == Kind::kParse || r.error().kind == Kind::kWrongType || r.error().kind == Kind::kNotFlatMap);
}

TEST(Config, DeeplyNestedInputIsRejectedNotCrashed) {
  const std::string deep(100000, '[');
  const auto r = load_params(deep, kSpecs);
  ASSERT_FALSE(r.has_value());
  const auto m = load_params("a: " + deep, kSpecs);
  ASSERT_FALSE(m.has_value());
}

TEST(Config, NullValuedKeyIsWrongType) {
  ExpectError("cruise_speed_mps:\nlio_recovery_timeout_s: 10\n", Kind::kWrongType, "cruise_speed_mps");
  ExpectError("cruise_speed_mps: null\nlio_recovery_timeout_s: 10\n", Kind::kWrongType, "cruise_speed_mps");
  ExpectError("cruise_speed_mps: ~\nlio_recovery_timeout_s: 10\n", Kind::kWrongType, "cruise_speed_mps");
}

TEST(Config, WrongTypeIsReportedBeforeNotFiniteBeforeRange) {
  // Rule 5 runs as three passes (convert, finite, range), each in spec order.
  ExpectError("cruise_speed_mps: 99\nlio_recovery_timeout_s: fast\n", Kind::kWrongType, "lio_recovery_timeout_s");
  ExpectError("cruise_speed_mps: 99\nlio_recovery_timeout_s: .nan\n", Kind::kNotFinite, "lio_recovery_timeout_s");
  ExpectError("cruise_speed_mps: 99\nlio_recovery_timeout_s: 1000\n", Kind::kOutOfRange, "cruise_speed_mps");
}

TEST(Config, ReportsFirstErrorInStableOrderWithinARule) {
  // Value errors follow spec order, whatever the YAML order.
  ExpectError("cruise_speed_mps: x\nlio_recovery_timeout_s: y\n", Kind::kWrongType, "cruise_speed_mps");
  ExpectError("lio_recovery_timeout_s: y\ncruise_speed_mps: x\n", Kind::kWrongType, "cruise_speed_mps");
  // Unknown keys follow document order.
  ExpectError("zzz: 1\naaa: 1\ncruise_speed_mps: 3\nlio_recovery_timeout_s: 10\n", Kind::kUnknownKey, "zzz");
  ExpectError("aaa: 1\nzzz: 1\ncruise_speed_mps: 3\nlio_recovery_timeout_s: 10\n", Kind::kUnknownKey, "aaa");
}

TEST(Config, MissingKeysAreReportedInSpecOrder) {
  static constexpr ParamSpec kThree[] = {{"b_s", Unit::kSeconds, 0, 1},
                                         {"a_s", Unit::kSeconds, 0, 1},
                                         {"c_s", Unit::kSeconds, 0, 1}};
  const auto r = load_params("c_s: 1", kThree);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kMissingKey);
  EXPECT_EQ(r.error().key, "b_s");  // spec order, not alphabetical
}

TEST(Config, AcceptsCleanDecimalAndExponentNumbers) {
  const std::pair<std::string_view, double> good[] = {
      {"3", 3.0},        {"3.0", 3.0},    {"-3.5", -3.5},       {"+3.5", 3.5},  {".5", 0.5},  {"5.", 5.0},
      {"1e3", 1000.0},   {"1E3", 1000.0}, {"2.5e-2", 0.025},    {"-1.5e+2", -150.0}, {"007", 7.0}, {"0", 0.0},
      {"-0", 0.0}};
  for (const auto& [text, value] : good) {
    const auto r = LoadOne(text);
    ASSERT_TRUE(r.has_value()) << text << ": " << r.error().detail;
    EXPECT_DOUBLE_EQ(r->at("gain"), value) << text;
  }
}

TEST(Config, UnderflowingLiteralIsNotFinite) {
  // Documented: a literal that does not fit a double (either direction) is rejected.
  const auto r = LoadOne("1e-999");
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kNotFinite);
}

TEST(Config, RejectsNonNumericScalarsAsWrongType) {
  for (const std::string_view text :
       {"3.0abc", "abc3", "0x10", "0o17", "0b11", "", "null", "Null", "~", "true", "false", "yes", "1,5", "1 5", "1_000",
        "--1", "+-1", "1e", "e5", "1e+", ".", "+", "1.2.3", "1..2", "'3.0'", "\"3.0\"", "!!str 3", "!!float 3",
        "٣", "Infinity", "inf", "nan", "NaN", "1e5x", "0x", "1f", "3.0f"}) {
    const auto r = LoadOne(text);
    ASSERT_FALSE(r.has_value()) << "accepted: '" << text << "'";
    EXPECT_EQ(r.error().kind, Kind::kWrongType) << "text: '" << text << "' got " << to_string(r.error().kind);
    EXPECT_EQ(r.error().key, "gain") << "text: '" << text << "'";
  }
}

TEST(Config, SpecialFloatSpellingsAreNotFinite) {
  for (const std::string_view text : {".nan", ".NaN", ".NAN", ".inf", ".Inf", ".INF", "-.inf", "+.inf", "-.Inf", "-.INF"}) {
    const auto r = LoadOne(text);
    ASSERT_FALSE(r.has_value()) << "accepted: '" << text << "'";
    EXPECT_EQ(r.error().kind, Kind::kNotFinite) << "text: '" << text << "'";
    EXPECT_EQ(r.error().key, "gain");
  }
}

TEST(Config, HugeLiteralIsNotFiniteNotWrapped) {
  for (const std::string& text : {std::string("1e999"), std::string("-1e999"), std::string("1e400"),
                                  std::string(400, '9'), "-" + std::string(400, '9')}) {
    const auto r = LoadOne(text);
    ASSERT_FALSE(r.has_value()) << text;
    EXPECT_EQ(r.error().kind, Kind::kNotFinite) << text;
    EXPECT_EQ(r.error().key, "gain");
  }
}

TEST(Config, AcceptsLargestFiniteDoubleWhenInRange) {
  static constexpr ParamSpec kWide[] = {{"big", Unit::kNone, -1.7976931348623157e308, 1.7976931348623157e308}};
  const auto r = load_params("big: 1.7976931348623157e308", kWide);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->at("big"), std::numeric_limits<double>::max());
}

TEST(Config, RejectsNonPlainScalarsEvenIfTheyLookNumeric) {
  ExpectError("cruise_speed_mps: \"3.0\"\nlio_recovery_timeout_s: 10\n", Kind::kWrongType, "cruise_speed_mps");
  ExpectError("cruise_speed_mps: '3.0'\nlio_recovery_timeout_s: 10\n", Kind::kWrongType, "cruise_speed_mps");
}

TEST(Config, ValueErrorDetailMentionsOffendingText) {
  const auto r = load_params("cruise_speed_mps: 3.0abc\nlio_recovery_timeout_s: 10\n", kSpecs);
  ASSERT_FALSE(r.has_value());
  EXPECT_NE(r.error().detail.find("3.0abc"), std::string::npos) << r.error().detail;
}

TEST(Config, AnchorAliasScalarIsAccepted) {
  const auto r = load_params("cruise_speed_mps: &v 3\nlio_recovery_timeout_s: *v\n", kSpecs);
  ASSERT_TRUE(r.has_value()) << r.error().detail;
  EXPECT_DOUBLE_EQ(r->at("lio_recovery_timeout_s"), 3.0);
}

TEST(Config, MergeKeyIsNotAFlatScalarMap) {
  const auto r = load_params("base: &b {cruise_speed_mps: 3}\n<<: *b\n", kSpecs);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kNotFlatMap);
  EXPECT_EQ(r.error().key, "base");
}

TEST(Config, ComplexKeyIsNotFlatMap) {
  const auto r = load_params("? [a, b]\n: 1\n", kSpecs);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kNotFlatMap);
}

TEST(Config, EmptySpecsAcceptOnlyAnEmptyMap) {
  const auto ok = load_params("{}", std::span<const ParamSpec>{});
  ASSERT_TRUE(ok.has_value());
  EXPECT_TRUE(ok->empty());
  const auto bad = load_params("a: 1", std::span<const ParamSpec>{});
  ASSERT_FALSE(bad.has_value());
  EXPECT_EQ(bad.error().kind, Kind::kUnknownKey);
  EXPECT_EQ(bad.error().key, "a");
}

// ---- Rule 1: spec validity -------------------------------------------------------

namespace {
void ExpectBadSpec(std::span<const ParamSpec> specs, std::string_view key) {
  // Spec errors are reported before the YAML is even looked at, so garbage text must not matter.
  for (const std::string_view text : {kValid, std::string_view{"{{{ not yaml"}}) {
    const auto r = load_params(text, specs);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().kind, Kind::kBadSpec);
    EXPECT_EQ(r.error().key, key);
    EXPECT_FALSE(r.error().detail.empty());
  }
}
}  // namespace

TEST(Config, RejectsSpecWithWrongUnitSuffix) {
  constexpr ParamSpec bad[] = {{"cruise_speed", Unit::kMetersPerSecond, 0, 1}};
  ExpectBadSpec(bad, "cruise_speed");
  constexpr ParamSpec wrong_unit[] = {{"cruise_speed_mps", Unit::kSeconds, 0, 1}};
  ExpectBadSpec(wrong_unit, "cruise_speed_mps");
  constexpr ParamSpec mps2[] = {{"accel_mps", Unit::kMetersPerSecond2, 0, 1}};
  ExpectBadSpec(mps2, "accel_mps");
}

TEST(Config, RejectsSpecWhoseKeyIsOnlyTheSuffix) {
  constexpr ParamSpec bad[] = {{"_s", Unit::kSeconds, 0, 1}};
  ExpectBadSpec(bad, "_s");
}

TEST(Config, RejectsUnitlessSpecWithUnitSuffix) {
  for (const std::string_view key : {"gain_s", "gain_m", "gain_mps", "gain_mps2", "gain_mps3", "gain_rad",
                                     "gain_rad_s", "gain_hz"}) {
    const ParamSpec bad[] = {{key, Unit::kNone, 0, 1}};
    ExpectBadSpec(bad, key);
  }
  constexpr ParamSpec ok[] = {{"gain", Unit::kNone, 0, 1}, {"max_iterations", Unit::kNone, 1, 100}};
  EXPECT_TRUE(load_params("gain: 0.5\nmax_iterations: 3\n", ok).has_value());
}

TEST(Config, RejectsSpecKeyWithLongerForeignSuffix) {
  // "_rad_s" ends with "_s" but is a different unit: a seconds spec may not hide a rate.
  constexpr ParamSpec bad[] = {{"yaw_rate_rad_s", Unit::kSeconds, 0, 1}};
  ExpectBadSpec(bad, "yaw_rate_rad_s");
  constexpr ParamSpec good[] = {{"yaw_rate_rad_s", Unit::kRadiansPerSecond, 0, 1}};
  EXPECT_TRUE(load_params("yaw_rate_rad_s: 0.5", good).has_value());
}

TEST(Config, RejectsSpecWithBadBounds) {
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  constexpr double inf = std::numeric_limits<double>::infinity();
  constexpr ParamSpec inverted[] = {{"a_s", Unit::kSeconds, 2, 1}};
  ExpectBadSpec(inverted, "a_s");
  constexpr ParamSpec nan_min[] = {{"a_s", Unit::kSeconds, nan, 1}};
  ExpectBadSpec(nan_min, "a_s");
  constexpr ParamSpec nan_max[] = {{"a_s", Unit::kSeconds, 0, nan}};
  ExpectBadSpec(nan_max, "a_s");
  constexpr ParamSpec inf_max[] = {{"a_s", Unit::kSeconds, 0, inf}};
  ExpectBadSpec(inf_max, "a_s");
  constexpr ParamSpec inf_min[] = {{"a_s", Unit::kSeconds, -inf, 1}};
  ExpectBadSpec(inf_min, "a_s");
  constexpr ParamSpec equal[] = {{"a_s", Unit::kSeconds, 1, 1}};
  EXPECT_TRUE(load_params("a_s: 1", equal).has_value());  // min == max is valid
}

TEST(Config, RejectsEmptyAndDuplicateSpecKeys) {
  constexpr ParamSpec empty[] = {{"", Unit::kNone, 0, 1}};
  ExpectBadSpec(empty, "");
  constexpr ParamSpec dup[] = {{"a_s", Unit::kSeconds, 0, 1}, {"b_s", Unit::kSeconds, 0, 1},
                               {"a_s", Unit::kSeconds, 0, 2}};
  ExpectBadSpec(dup, "a_s");
}

TEST(Config, ReportsFirstBadSpecInArrayOrder) {
  constexpr ParamSpec specs[] = {{"ok_s", Unit::kSeconds, 0, 1}, {"second", Unit::kSeconds, 0, 1},
                                 {"third", Unit::kMeters, 0, 1}};
  ExpectBadSpec(specs, "second");
}

// ---- Files -------------------------------------------------------------------------

namespace {
struct TempDir {
  std::filesystem::path path;
  TempDir() {
    path = std::filesystem::temp_directory_path() /
           ("uavnav_config_test_" + std::to_string(::getpid()) + "_" +
            ::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::remove_all(path);
    std::filesystem::create_directories(path);
  }
  ~TempDir() {
    std::error_code ec;
    std::filesystem::permissions(path, std::filesystem::perms::owner_all, std::filesystem::perm_options::add, ec);
    std::filesystem::remove_all(path, ec);
  }
};
}  // namespace

TEST(Config, FileUnreadable) {
  const auto r = load_params_file("/nonexistent.yaml", kSpecs);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kFileUnreadable);
  EXPECT_NE(r.error().detail.find("/nonexistent.yaml"), std::string::npos) << r.error().detail;
}

TEST(Config, DirectoryIsFileUnreadable) {
  TempDir dir;
  const auto r = load_params_file(dir.path, kSpecs);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kFileUnreadable);
}

TEST(Config, PermissionDeniedIsFileUnreadable) {
  if (::geteuid() == 0) GTEST_SKIP() << "root bypasses file permissions";
  TempDir dir;
  const auto file = dir.path / "locked.yaml";
  { std::ofstream(file) << kValid; }
  std::filesystem::permissions(file, std::filesystem::perms::none);
  const auto r = load_params_file(file, kSpecs);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kFileUnreadable);
}

TEST(Config, LoadsFromFile) {
  TempDir dir;
  const auto file = dir.path / "params.yaml";
  { std::ofstream(file) << kValid; }
  const auto r = load_params_file(file, kSpecs);
  ASSERT_TRUE(r.has_value()) << r.error().detail;
  EXPECT_DOUBLE_EQ(r->at("cruise_speed_mps"), 3.0);
  EXPECT_DOUBLE_EQ(r->at("lio_recovery_timeout_s"), 10.0);
}

TEST(Config, FileErrorsKeepTheirKind) {
  TempDir dir;
  const auto file = dir.path / "params.yaml";
  { std::ofstream(file) << "cruise_speed_mps: 7\nlio_recovery_timeout_s: 10\n"; }
  const auto r = load_params_file(file, kSpecs);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kOutOfRange);
  EXPECT_EQ(r.error().key, "cruise_speed_mps");
  { std::ofstream(file, std::ios::trunc) << ""; }
  const auto e = load_params_file(file, kSpecs);
  ASSERT_FALSE(e.has_value());
  EXPECT_EQ(e.error().kind, Kind::kNotFlatMap);
}

TEST(Config, BadSpecIsReportedBeforeTheFileIsTouched) {
  constexpr ParamSpec bad[] = {{"cruise_speed", Unit::kMetersPerSecond, 0, 1}};
  const auto r = load_params_file("/nonexistent.yaml", bad);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, Kind::kBadSpec);
}
