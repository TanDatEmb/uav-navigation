#include <px4_navigation/virtual_scan.hpp>

#include <algorithm>
#include <limits>

namespace px4_navigation {

VirtualScan::VirtualScan()
    : angle_increment_(px4_common::math::kPi / 180.0),  // 1 degree
      max_range_(kDefaultMaxRange),
      vehicle_radius_(kDefaultVehicleRadius),
      vehicle_radius_sq_(kDefaultVehicleRadius * kDefaultVehicleRadius),
      max_range_sq_(kDefaultMaxRange * kDefaultMaxRange) {
    scan_ranges_.resize(kNumBins, static_cast<float>(max_range_));

    // Reserve space in the cached scan
    if (!cached_scan_) {
        cached_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>();
        cached_scan_->ranges.resize(kNumBins);
        cached_scan_->angle_min = -px4_common::math::kPi;
        cached_scan_->angle_max = px4_common::math::kPi;
        cached_scan_->angle_increment = angle_increment_;
        cached_scan_->range_min = 0.1f;
        cached_scan_->range_max = static_cast<float>(max_range_);
    }
}

void VirtualScan::reset(double angle_resolution, double max_range, double vehicle_radius) {
    angle_increment_ = angle_resolution;
    max_range_ = max_range;
    vehicle_radius_ = vehicle_radius;
    vehicle_radius_sq_ = vehicle_radius * vehicle_radius;
    max_range_sq_ = max_range * max_range;

    scan_ranges_.assign(kNumBins, static_cast<float>(max_range_));

    // Update cached scan parameters
    if (cached_scan_) {
        cached_scan_->angle_increment = angle_increment_;
        cached_scan_->range_max = static_cast<float>(max_range_);
    }
}

void VirtualScan::update(const std::vector<px4_common::PointLivox>& occupied_points,
                         const px4_common::DroneStateNed& drone_state, double height_above,
                         double height_below) {
    // Reset all bins to max range, indicates no obstacle
    std::fill(scan_ranges_.begin(), scan_ranges_.end(), static_cast<float>(max_range_));

    // Height filter in NED, Z is down
    // drone_state.position.z() is negative when above ground
    // More negative Z is higher altitude
    // z_lo is the higher altitude limit, z_hi is the lower limit
    const double z_lo = drone_state.position.z() - height_above;
    const double z_hi = drone_state.position.z() + height_below;

    for (const auto& pt : occupied_points) {
        // Skip points outside altitude band
        if (pt.z < z_lo || pt.z > z_hi)
            continue;

        // Compute XY offset from drone, both in NED frame
        double dx = pt.x - drone_state.position.x();
        double dy = pt.y - drone_state.position.y();
        double dist_sq = dx * dx + dy * dy;

        // Filter drone self-reflection (within vehicle radius)
        // and points beyond max_range
        if (dist_sq > max_range_sq_ || dist_sq < vehicle_radius_sq_)
            continue;

        double dist = std::sqrt(dist_sq);

        // Compute angle relative to drone heading, normalize to [-pi, pi]
        double angle = std::atan2(dy, dx) - drone_state.yaw;
        angle = std::atan2(std::sin(angle), std::cos(angle));

        // Map angle to bin index
        int bin = static_cast<int>((angle + px4_common::math::kPi) / angle_increment_);
        if (bin < 0)
            bin = 0;
        if (bin >= kNumBins)
            bin = kNumBins - 1;

        // Keep minimum distance per bin
        float dist_f = static_cast<float>(dist);
        if (dist_f < scan_ranges_[bin])
            scan_ranges_[bin] = dist_f;
    }

    // Update cached scan if needed
    if (cached_scan_) {
        cached_scan_->ranges = scan_ranges_;
        cached_scan_->range_max = static_cast<float>(max_range_);
    }
}

std::vector<double> VirtualScan::get_obstacle_bearings() const {
    std::vector<double> bearings;
    bearings.reserve(kNumBins);

    for (int i = 0; i < kNumBins; ++i) {
        bearings.push_back(-px4_common::math::kPi + i * angle_increment_);
    }

    return bearings;
}

}  // namespace px4_navigation