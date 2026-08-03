#pragma once

#include <optional>

#include "odometry_supervisor/supervisor_types.hpp"

namespace odometry_supervisor {

// Applies a captured world alignment to one PX4 sample. Body-frame velocity
// remains unchanged because both producers publish base_link body velocity.
[[nodiscard]] std::optional<OdometryState> applyWorldAlignment(
    const OdometryState& source, const WorldAlignment& alignment);

}  // namespace odometry_supervisor
