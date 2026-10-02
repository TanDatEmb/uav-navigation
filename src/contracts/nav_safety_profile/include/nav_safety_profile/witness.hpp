#pragma once

#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "nav_safety_profile/generated/profile.hpp"

namespace nav_safety_profile {

inline std::string format_scalar(const Scalar& value) {
  return std::visit([](const auto& scalar) {
    using T = std::decay_t<decltype(scalar)>;
    std::ostringstream out;
    if constexpr (std::is_same_v<T, bool>) {
      out << (scalar ? "true" : "false");
    } else if constexpr (std::is_same_v<T, std::string>) {
      out << scalar;
    } else if constexpr (std::is_same_v<T, std::vector<double>>) {
      out << '[';
      for (std::size_t i = 0; i < scalar.size(); ++i) {
        if (i != 0) out << ',';
        out << std::setprecision(17) << scalar[i];
      }
      out << ']';
    } else {
      out << std::setprecision(17) << scalar;
    }
    return out.str();
  }, value);
}

inline std::string witness_line(
    const Profile& profile,
    const std::string& owner,
    const std::map<std::string, Scalar>& effective_values) {
  std::vector<std::string> mismatches;
  std::size_t keys = 0;
  for (const auto& [key, entry] : profile.entries) {
    if (std::find(entry.owners.begin(), entry.owners.end(), owner) == entry.owners.end()) {
      continue;
    }
    ++keys;
    const auto effective = effective_values.find(key);
    if (effective == effective_values.end()) {
      mismatches.push_back(key + "=<unavailable>/" + format_scalar(entry.value));
      continue;
    }
    if (effective->second != entry.value) {
      mismatches.push_back(key + "=" + format_scalar(effective->second) + "/" +
                           format_scalar(entry.value));
    }
  }
  std::ostringstream out;
  out << "SAFETY_PROFILE_WITNESS hash=" << std::hex << std::setfill('0')
      << std::setw(16) << profile.hash64 << std::dec << " keys=" << keys
      << " mismatches=[";
  for (std::size_t i = 0; i < mismatches.size(); ++i) {
    if (i != 0) out << ',';
    out << mismatches[i];
  }
  out << ']';
  return out.str();
}

}  // namespace nav_safety_profile
