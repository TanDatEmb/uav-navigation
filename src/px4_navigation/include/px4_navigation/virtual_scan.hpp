#ifndef PX4_NAVIGATION_VIRTUAL_SCAN_HPP_
#define PX4_NAVIGATION_VIRTUAL_SCAN_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <px4_nav_common/types.hpp>
#include <px4_ros2_utils/common/constants.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace px4_navigation {

/**
 * @brief Virtual scan generator that converts 3D point clouds to 2D laser scans.
 *
 * This class generates synthetic laser scans from 3D point clouds by projecting
 * points onto a 2D plane and binning them into angular sectors. The scan is
 * generated in the drone's local NED frame.
 */
class VirtualScan {
   public:
    /// Number of angular bins (360 degrees at 1 degree resolution)
    static constexpr int kNumBins = 360;

    /// Default maximum scan range in metres
    static constexpr double kDefaultMaxRange = 15.0;

    /// Default vehicle safety radius in metres
    static constexpr double kDefaultVehicleRadius = 0.5;

    /// Maximum scan range in metres
    double MaxRange() const {
        return max_range_;
    }

    /// Vehicle safety radius in metres
    double VehicleRadius() const {
        return vehicle_radius_;
    }

    /**
     * @brief Construct a new VirtualScan object.
     */
    VirtualScan();

    /**
     * @brief Reset the virtual scan with new parameters.
     *
     * @param angle_resolution Angle resolution in radians (default: 1 degree)
     * @param max_range Maximum range in meters
     * @param vehicle_radius Vehicle safety radius in meters
     */
    void Reset(double angle_resolution = px4_ros2_utils::constants::PI / 180.0,  // 1 degree
               double max_range = kDefaultMaxRange, double vehicle_radius = kDefaultVehicleRadius);

    /**
     * @brief Update the virtual scan with new obstacle points.
     *
     * @param occupied_points Vector of obstacle points in NED frame
     * @param drone_state Current drone state in NED frame
     * @param height_above Include points up to X meters above drone
     * @param height_below Include points up to X meters below drone
     */
    void Update(const std::vector<px4_nav_common::PointLivox>& occupied_points,
                const px4_nav_common::DroneStateNed& drone_state, double height_above = 2.0,
                double height_below = 1.0);

    /**
     * @brief Get obstacle distances from the most recent scan.
     *
     * @return const std::vector<float>& Vector of distances in meters
     */
    const std::vector<float>& ObstacleDistances() const {
        return scan_ranges_;
    }

    /**
     * @brief Get obstacle bearings from the most recent scan.
     *
     * @return std::vector<double> Vector of bearings in radians [-π, π]
     */
    std::vector<double> ObstacleBearings() const;

    /**
     * @brief Get the angle increment between bins.
     *
     * @return double Angle increment in radians
     */
    double AngleIncrement() const {
        return angle_increment_;
    }

   private:
    /// Vector of scan ranges (distances)
    std::vector<float> scan_ranges_;

    /// Angle increment between bins in radians
    double angle_increment_;

    /// Maximum scan range in meters
    double max_range_;

    /// Vehicle safety radius in meters
    double vehicle_radius_;

    /// Vehicle safety radius squared
    double vehicle_radius_sq_;

    /// Maximum range squared
    double max_range_sq_;
};

}  // namespace px4_navigation

#endif  // PX4_NAVIGATION_VIRTUAL_SCAN_HPP_
