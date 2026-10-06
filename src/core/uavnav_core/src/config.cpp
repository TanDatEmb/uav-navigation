#include "uavnav/core/config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <exception>
#include <format>
#include <fstream>
#include <optional>
#include <set>
#include <system_error>
#include <utility>
#include <vector>

namespace uavnav::config {

namespace {

using Kind = ConfigError::Kind;

std::unexpected<ConfigError> fail(Kind kind, std::string key, std::string detail) {
  return std::unexpected<ConfigError>(ConfigError{kind, std::move(key), std::move(detail)});
}

// Offending text is echoed in messages; keep a hostile 1 MB scalar from being copied whole.
std::string clip(std::string_view s) {
  constexpr std::size_t kMax = 64;
  if (s.size() <= kMax) return std::string(s);
  return std::string(s.substr(0, kMax)) + "...";
}

// ---- Rule 1: spec validity ---------------------------------------------------------

constexpr std::array<Unit, 8> kUnits = {Unit::kSeconds,          Unit::kMeters,    Unit::kMetersPerSecond,
                                        Unit::kMetersPerSecond2, Unit::kMetersPerSecond3, Unit::kRadians,
                                        Unit::kRadiansPerSecond, Unit::kHertz};

// The unit whose suffix is the LONGEST one the key ends with (kNone if it ends with none).
// Longest wins because "_rad_s" ends with "_s" but is a rate, not a duration.
Unit unit_of_key(std::string_view key) {
  Unit best = Unit::kNone;
  std::size_t best_len = 0;
  for (const Unit u : kUnits) {
    const std::string_view s = suffix(u);
    if (key.ends_with(s) && s.size() > best_len) {
      best = u;
      best_len = s.size();
    }
  }
  return best;
}

std::optional<ConfigError> check_specs(std::span<const ParamSpec> specs) {
  std::set<std::string_view> seen;
  for (const ParamSpec& spec : specs) {
    auto bad = [&](std::string detail) {
      return ConfigError{Kind::kBadSpec, std::string(spec.key), std::move(detail)};
    };
    if (spec.key.empty()) return bad("spec key is empty");
    if (!seen.insert(spec.key).second) return bad("spec key is declared more than once");
    const std::string_view sfx = suffix(spec.unit);
    if (spec.unit == Unit::kNone) {
      if (const Unit u = unit_of_key(spec.key); u != Unit::kNone) {
        return bad(std::format("unitless spec key ends with unit suffix '{}'", suffix(u)));
      }
    } else {
      if (!spec.key.ends_with(sfx) || spec.key.size() == sfx.size()) {
        return bad(std::format("key must be a name followed by unit suffix '{}'", sfx));
      }
      if (const Unit u = unit_of_key(spec.key); u != spec.unit) {
        return bad(std::format("key ends with unit suffix '{}', which contradicts the declared suffix '{}'",
                               suffix(u), sfx));
      }
    }
    if (!std::isfinite(spec.min) || !std::isfinite(spec.max)) return bad("bounds must be finite");
    if (spec.min > spec.max) {
      return bad(std::format("min {} is greater than max {}", spec.min, spec.max));
    }
  }
  return std::nullopt;
}

// ---- Rule 5: strict scalar conversion ----------------------------------------------

enum class Scan : std::uint8_t { kNumber, kNonFinite, kNotNumber };

// Grammar (YAML 1.2 core schema decimal int/float, nothing else):
//   [+-]? ( digits+ ('.' digits*)? | '.' digits+ ) ( [eE] [+-]? digits+ )?
// plus the special spellings [+-]?.inf/.Inf/.INF and .nan/.NaN/.NAN, recognised only so they
// can be reported as non-finite rather than as a wrong type.
Scan scan_number(std::string_view s) {
  std::string_view rest = s;
  bool signed_ = false;
  if (!rest.empty() && (rest.front() == '+' || rest.front() == '-')) {
    signed_ = true;
    rest.remove_prefix(1);
  }
  if (rest == ".inf" || rest == ".Inf" || rest == ".INF") return Scan::kNonFinite;
  if (!signed_ && (rest == ".nan" || rest == ".NaN" || rest == ".NAN")) return Scan::kNonFinite;

  std::size_t i = 0;
  auto digits = [&] {
    const std::size_t start = i;
    while (i < rest.size() && rest[i] >= '0' && rest[i] <= '9') ++i;
    return i - start;
  };
  const std::size_t int_digits = digits();
  std::size_t frac_digits = 0;
  if (i < rest.size() && rest[i] == '.') {
    ++i;
    frac_digits = digits();
  }
  if (int_digits == 0 && frac_digits == 0) return Scan::kNotNumber;
  if (i < rest.size() && (rest[i] == 'e' || rest[i] == 'E')) {
    ++i;
    if (i < rest.size() && (rest[i] == '+' || rest[i] == '-')) ++i;
    if (digits() == 0) return Scan::kNotNumber;
  }
  return i == rest.size() ? Scan::kNumber : Scan::kNotNumber;
}

struct Converted {
  enum class State : std::uint8_t { kOk, kWrongType, kNotFinite } state;
  double value;
  std::string detail;
};

Converted convert(const YAML::Node& node) {
  if (node.IsNull()) return {Converted::State::kWrongType, 0.0, "value is empty/null, expected a number"};
  const std::string& text = node.Scalar();
  // Plain scalars carry the non-specific tag "?"; quoted ones carry "!" and explicit tags
  // (!!str, !!float, ...) their own. Only plain scalars are accepted as numbers.
  if (node.Tag() != "?") {
    return {Converted::State::kWrongType, 0.0,
            std::format("value '{}' is quoted or tagged, expected a plain number", clip(text))};
  }
  switch (scan_number(text)) {
    case Scan::kNotNumber:
      return {Converted::State::kWrongType, 0.0, std::format("value '{}' is not a number", clip(text))};
    case Scan::kNonFinite:
      return {Converted::State::kNotFinite, 0.0, std::format("value '{}' is not finite", clip(text))};
    case Scan::kNumber:
      break;
  }
  // std::from_chars is locale-independent but rejects a leading '+'.
  const char* first = text.data() + (text.front() == '+' ? 1 : 0);
  const char* last = text.data() + text.size();
  double v = 0.0;
  const auto [ptr, ec] = std::from_chars(first, last, v);
  if (ec == std::errc::result_out_of_range) {
    return {Converted::State::kNotFinite, 0.0,
            std::format("value '{}' does not fit a finite double", clip(text))};
  }
  if (ec != std::errc{} || ptr != last) {
    return {Converted::State::kWrongType, 0.0, std::format("value '{}' is not a number", clip(text))};
  }
  if (!std::isfinite(v)) {
    return {Converted::State::kNotFinite, 0.0, std::format("value '{}' is not finite", clip(text))};
  }
  return {Converted::State::kOk, v, {}};
}

// ---- Core --------------------------------------------------------------------------

struct Entry {
  std::string key;
  YAML::Node value;
};

Result<ParamValues, ConfigError> load_impl(std::string_view yaml_text, std::span<const ParamSpec> specs) {
  // Rule 1.
  if (auto bad = check_specs(specs)) return std::unexpected<ConfigError>(std::move(*bad));

  // Rule 2: parse.
  std::vector<YAML::Node> docs;
  try {
    docs = YAML::LoadAll(std::string(yaml_text));
  } catch (const YAML::Exception& e) {
    return fail(Kind::kParse, "", e.what());
  }
  if (docs.empty()) {
    return fail(Kind::kNotFlatMap, "", "the text has no YAML content, expected a flat map of key: number");
  }
  if (docs.size() > 1) {
    return fail(Kind::kParse, "", "the text holds more than one YAML document, expected exactly one");
  }
  const YAML::Node& root = docs.front();
  if (!root.IsMap()) {
    return fail(Kind::kNotFlatMap, "",
                root.IsNull() ? "the document is empty, expected a flat map of key: number"
                              : "the top level is not a map, expected a flat map of key: number");
  }

  // Rule 2 (flat): scalar keys, scalar-or-null values, in document order.
  std::vector<Entry> entries;
  for (const auto& kv : root) {
    if (!kv.first.IsScalar()) {
      return fail(Kind::kNotFlatMap, "", "a map key is not a plain scalar");
    }
    std::string key = kv.first.Scalar();
    if (kv.second.IsMap() || kv.second.IsSequence()) {
      return fail(Kind::kNotFlatMap, key,
                  std::format("value of '{}' is a {}, expected a number", clip(key),
                              kv.second.IsMap() ? "map" : "sequence"));
    }
    entries.push_back({std::move(key), kv.second});
  }

  // Rule 3: duplicates (yaml-cpp keeps both pairs of a repeated key when iterating).
  {
    std::set<std::string_view> seen;
    for (const Entry& e : entries) {
      if (!seen.insert(e.key).second) {
        return fail(Kind::kDuplicateKey, e.key, std::format("key '{}' appears more than once", clip(e.key)));
      }
    }
  }

  // Rule 4: unknown keys (document order), then missing keys (spec order).
  auto find_spec = [&](std::string_view key) {
    return std::find_if(specs.begin(), specs.end(), [&](const ParamSpec& s) { return s.key == key; });
  };
  for (const Entry& e : entries) {
    if (find_spec(e.key) == specs.end()) {
      return fail(Kind::kUnknownKey, e.key, std::format("key '{}' is not a declared parameter", clip(e.key)));
    }
  }
  auto find_entry = [&](std::string_view key) {
    return std::find_if(entries.begin(), entries.end(), [&](const Entry& e) { return e.key == key; });
  };
  for (const ParamSpec& spec : specs) {
    if (find_entry(spec.key) == entries.end()) {
      return fail(Kind::kMissingKey, std::string(spec.key),
                  std::format("required key '{}' is missing", spec.key));
    }
  }

  // Rule 5: three passes in spec order: convertible, finite, in range.
  std::vector<Converted> values;
  values.reserve(specs.size());
  for (const ParamSpec& spec : specs) values.push_back(convert(find_entry(spec.key)->value));
  for (std::size_t i = 0; i < specs.size(); ++i) {
    if (values[i].state == Converted::State::kWrongType) {
      return fail(Kind::kWrongType, std::string(specs[i].key),
                  std::format("'{}': {}", specs[i].key, values[i].detail));
    }
  }
  for (std::size_t i = 0; i < specs.size(); ++i) {
    if (values[i].state == Converted::State::kNotFinite) {
      return fail(Kind::kNotFinite, std::string(specs[i].key),
                  std::format("'{}': {}", specs[i].key, values[i].detail));
    }
  }
  ParamValues out;
  for (std::size_t i = 0; i < specs.size(); ++i) {
    const ParamSpec& spec = specs[i];
    const double v = values[i].value;
    if (v < spec.min || v > spec.max) {
      return fail(Kind::kOutOfRange, std::string(spec.key),
                  std::format("'{}': value {} is outside [{}, {}]", spec.key, v, spec.min, spec.max));
    }
    out.emplace(std::string(spec.key), v);
  }
  return out;
}

Result<ParamValues, ConfigError> guarded(std::string_view yaml_text, std::span<const ParamSpec> specs) noexcept {
  try {
    return load_impl(yaml_text, specs);
  } catch (const YAML::Exception& e) {  // belt and braces: conversions never throw today
    return fail(Kind::kParse, "", e.what());
  } catch (const std::exception& e) {
    return fail(Kind::kParse, "", std::string("unexpected failure while loading config: ") + e.what());
  } catch (...) {
    return fail(Kind::kParse, "", "unexpected failure while loading config");
  }
}

}  // namespace

Result<double, ConfigError> value(const ParamValues& values, std::string_view key) {
  const auto it = values.find(key);
  if (it == values.end()) {
    return fail(Kind::kMissingKey, std::string(key), std::format("key '{}' is not in the loaded values", clip(key)));
  }
  return it->second;
}

Result<ParamValues, ConfigError> load_params(std::string_view yaml_text, std::span<const ParamSpec> specs) noexcept {
  return guarded(yaml_text, specs);
}

Result<ParamValues, ConfigError> load_params_file(const std::filesystem::path& file,
                                                  std::span<const ParamSpec> specs) noexcept {
  try {
    // Rule 1 comes before the file is touched.
    if (auto bad = check_specs(specs)) return std::unexpected<ConfigError>(std::move(*bad));

    const std::string path = file.string();
    std::error_code ec;
    const auto status = std::filesystem::status(file, ec);
    if (ec || !std::filesystem::is_regular_file(status)) {
      return fail(Kind::kFileUnreadable, "", std::format("'{}' is missing or is not a regular file", path));
    }
    const std::uintmax_t size = std::filesystem::file_size(file, ec);
    if (ec) return fail(Kind::kFileUnreadable, "", std::format("cannot get the size of '{}'", path));
    const std::string too_big = std::format("'{}' is larger than {} bytes", path, kMaxConfigFileBytes);
    if (size > kMaxConfigFileBytes) return fail(Kind::kFileUnreadable, "", too_big);
    std::ifstream in(file, std::ios::binary);
    if (!in) return fail(Kind::kFileUnreadable, "", std::format("cannot open '{}' for reading", path));
    // The file can grow after the size check, or be a special file: never read more than the cap + 1.
    std::string text;
    std::array<char, 4096> buf;
    while (text.size() <= kMaxConfigFileBytes) {
      const auto want = static_cast<std::streamsize>(
          std::min<std::uintmax_t>(buf.size(), kMaxConfigFileBytes + 1 - text.size()));
      in.read(buf.data(), want);
      if (in.gcount() <= 0) break;
      text.append(buf.data(), static_cast<std::size_t>(in.gcount()));
    }
    if (text.size() > kMaxConfigFileBytes) return fail(Kind::kFileUnreadable, "", too_big);
    if (in.bad()) return fail(Kind::kFileUnreadable, "", std::format("read error on '{}'", path));
    return guarded(text, specs);
  } catch (const std::exception& e) {
    return fail(Kind::kFileUnreadable, "", std::string("failure while reading config file: ") + e.what());
  } catch (...) {
    return fail(Kind::kFileUnreadable, "", "failure while reading config file");
  }
}

}  // namespace uavnav::config
