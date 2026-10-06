#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <span>
#include <string>
#include <string_view>

#include "uavnav/core/result.hpp"

// Tier-(b) parameters (SYSTEM_DESIGN §6.2, D22): operating values loaded from YAML and
// checked against a schema of unit-suffixed keys with inclusive [min, max] bounds.
// A bad file is rejected with the offending key named; nothing is ever defaulted,
// clamped or guessed. Nothing here throws: yaml-cpp exceptions are caught inside and
// returned as ConfigError values.
namespace uavnav::config {

/// Physical unit encoded in the key suffix, so a value's unit is visible at every use.
enum class Unit : std::uint8_t {
  kNone,
  kSeconds,
  kMeters,
  kMetersPerSecond,
  kMetersPerSecond2,
  kMetersPerSecond3,
  kRadians,
  kRadiansPerSecond,
  kHertz
};

constexpr std::string_view suffix(Unit u) {
  switch (u) {
    case Unit::kNone: return "";
    case Unit::kSeconds: return "_s";
    case Unit::kMeters: return "_m";
    case Unit::kMetersPerSecond: return "_mps";
    case Unit::kMetersPerSecond2: return "_mps2";
    case Unit::kMetersPerSecond3: return "_mps3";
    case Unit::kRadians: return "_rad";
    case Unit::kRadiansPerSecond: return "_rad_s";
    case Unit::kHertz: return "_hz";
  }
  return "";
}

/// One declared parameter. `key` is a view: it must outlive the load call (typically a
/// string literal in a constexpr table).
struct ParamSpec {
  std::string_view key;
  Unit unit;
  double min;  // inclusive
  double max;  // inclusive
};

struct ConfigError {
  enum class Kind : std::uint8_t {
    kFileUnreadable,
    kParse,
    kNotFlatMap,
    kDuplicateKey,
    kUnknownKey,
    kMissingKey,
    kWrongType,
    kNotFinite,
    kOutOfRange,
    kBadSpec
  };
  Kind kind;
  std::string key;     // the offending key; empty when the failure is not tied to a key
  std::string detail;  // human-readable; for kOutOfRange it names the value and both bounds
};

constexpr std::string_view to_string(ConfigError::Kind k) {
  switch (k) {
    case ConfigError::Kind::kFileUnreadable: return "FILE_UNREADABLE";
    case ConfigError::Kind::kParse: return "PARSE";
    case ConfigError::Kind::kNotFlatMap: return "NOT_FLAT_MAP";
    case ConfigError::Kind::kDuplicateKey: return "DUPLICATE_KEY";
    case ConfigError::Kind::kUnknownKey: return "UNKNOWN_KEY";
    case ConfigError::Kind::kMissingKey: return "MISSING_KEY";
    case ConfigError::Kind::kWrongType: return "WRONG_TYPE";
    case ConfigError::Kind::kNotFinite: return "NOT_FINITE";
    case ConfigError::Kind::kOutOfRange: return "OUT_OF_RANGE";
    case ConfigError::Kind::kBadSpec: return "BAD_SPEC";
  }
  return "";
}

/// Config files larger than this are rejected before parsing (a hostile or wrong file must not be slurped).
inline constexpr std::uintmax_t kMaxConfigFileBytes = 1048576;

/// Every spec key mapped to its validated value.
using ParamValues = std::map<std::string, double, std::less<>>;

/// The validated value of `key`. kMissingKey (naming the key) when `values` does not hold it.
/// Never throws, unlike std::map::at.
Result<double, ConfigError> value(const ParamValues& values, std::string_view key);

/// Loads `yaml_text` against `specs`. On success the result holds exactly one finite,
/// in-range value per spec. When several things are wrong, the FIRST error by this
/// rule order is returned (and within a rule, the first in the stated order):
///
///  1. Spec validity (array order), before the text is looked at. A spec is bad when
///     its key is empty or duplicated; when the key does not end with suffix(unit)
///     after a non-empty stem; when the key's LONGEST known unit suffix is not its own
///     (so "yaw_rate_rad_s" cannot be declared kSeconds, and a kNone key may end in no
///     unit suffix at all); or when min/max is not finite or min > max. -> kBadSpec
///  2. The text must be exactly one YAML document, else kParse (malformed, or more than
///     one document). That document must be a map whose keys and values are scalars:
///     an empty/comment-only text, a top-level scalar or sequence, a nested map or
///     sequence value, or a non-scalar key is kNotFlatMap, naming the key when known.
///  3. A key that appears twice -> kDuplicateKey (yaml-cpp accepts these silently, so
///     the map is walked here). Reported at the first repeated key in document order.
///  4. A YAML key not in `specs` -> kUnknownKey (document order); then a spec key absent
///     from the YAML -> kMissingKey (spec order). Unknown wins so a typo is named
///     rather than reported as the key it was meant to be.
///  5. Value checks, as three passes each in spec order: every value must be a number
///     (kWrongType); then finite (kNotFinite); then within [min, max] (kOutOfRange).
///
/// What counts as a number is strict: a PLAIN (unquoted, untagged) YAML scalar that is
/// exactly  [+-]? ( digits+ ('.' digits*)? | '.' digits+ ) ( [eE] [+-]? digits+ )?
/// i.e. a YAML 1.2 core-schema decimal integer or float. Quoted strings ("3"), tags,
/// null (`key:`, `null`, `~`), booleans, hex/octal/binary, digit separators, "1,5",
/// trailing junk ("3.0abc") and surrounding text are kWrongType. The special spellings
/// .nan/.NaN/.NAN and [+-].inf/.Inf/.INF are recognised only to be reported as
/// kNotFinite. A literal that does not fit a finite double (1e999, or underflow such as
/// 1e-999) is also kNotFinite. Scalars are parsed locale-independently.
Result<ParamValues, ConfigError> load_params(std::string_view yaml_text, std::span<const ParamSpec> specs) noexcept;

/// Reads `file` and calls load_params. The specs are validated first (rule 1), before
/// the file is touched. A missing, non-regular (e.g. directory) or unreadable file, a
/// read failure, or a file larger than kMaxConfigFileBytes (1 MiB), is kFileUnreadable
/// with the path or the reason ("larger than 1048576 bytes") in `detail`.
Result<ParamValues, ConfigError> load_params_file(const std::filesystem::path& file,
                                                  std::span<const ParamSpec> specs) noexcept;

}  // namespace uavnav::config
