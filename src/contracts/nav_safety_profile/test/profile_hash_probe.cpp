#include <iostream>

#include "nav_safety_profile/generated/profile.hpp"

int main() {
  const auto result = nav_safety_profile::load(SAFETY_PROFILE_SOURCE);
  if (!result) {
    std::cerr << result.error << '\n';
    return 1;
  }
  std::cout << result.value->sha256 << '\n';
  return 0;
}
