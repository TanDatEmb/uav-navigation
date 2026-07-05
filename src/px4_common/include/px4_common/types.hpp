#ifndef PX4_COMMON_TYPES_HPP_
#define PX4_COMMON_TYPES_HPP_

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace px4_common {

/**
 * @brief Raw LiDAR point with intensity.
 *
 * Stored in the sensor frame before extrinsic correction. Units are metres.
 */
struct PointLivox {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    float intensity{0.0f};
};

/**
 * @brief Hash functor for Eigen::Vector3i voxel indices.
 *
 * Combines the three integer components into a single size_t value using a
 * boost::hash_combine style mix. Suitable for std::unordered_map keyed by
 * voxel indices.
 */
struct VoxelHash {
    size_t operator()(const Eigen::Vector3i &key) const noexcept {
        static constexpr size_t kHashMix = 0x9e3779b9;
        size_t hash = 0;
        hash ^= std::hash<int>{}(key(0)) + kHashMix + (hash << 6) + (hash >> 2);
        hash ^= std::hash<int>{}(key(1)) + kHashMix + (hash << 6) + (hash >> 2);
        hash ^= std::hash<int>{}(key(2)) + kHashMix + (hash << 6) + (hash >> 2);
        return hash;
    }
};

/**
 * @brief Drone local state expressed in the PX4 NED frame.
 *
 * Position origin is the EKF2 local origin. Z is positive-down (NED).
 * Yaw is positive clockwise around the Z (down) axis, in radians.
 */
struct DroneStateNed {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    bool valid{false};
};

/**
 * @brief Single mission waypoint in the PX4 NED frame.
 *
 * The NED position is the primary field used by planning and control.
 * Latitude, longitude and altitude are stored as auxiliary data for
 * mission awareness and telemetry only.
 */
struct WaypointNed {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double lat{0.0};
    double lon{0.0};
    double alt{0.0};
    bool valid{false};
};

/**
 * @brief Mission path composed of previous, current and next waypoints.
 *
 * Provides geometric helpers for mission following. All positions are in the
 * PX4 NED frame.
 */
class MissionPathNed {
   public:
    WaypointNed previous;
    WaypointNed current;
    WaypointNed next;
    bool has_valid_path{false};

    /**
     * @brief Normalized direction vector from previous to current waypoint.
     * @param[out] dx Normalized X component in the NED frame.
     * @param[out] dy Normalized Y component in the NED frame.
     */
    void GetDirection(double &dx, double &dy) const noexcept {
        dx = 0.0;
        dy = 0.0;
        if (!has_valid_path) {
            return;
        }

        Eigen::Vector3d delta = Eigen::Vector3d::Zero();
        if (current.valid && previous.valid) {
            delta = current.position - previous.position;
        } else if (current.valid && next.valid) {
            delta = next.position - current.position;
        }

        const double norm = delta.head<2>().norm();
        static constexpr double kMinSegmentLength = 0.01;
        if (norm > kMinSegmentLength) {
            dx = delta.x() / norm;
            dy = delta.y() / norm;
        }
    }

    /**
     * @brief Perpendicular distance from a drone position to the mission line.
     * @param drone_x Drone X position in the NED frame.
     * @param drone_y Drone Y position in the NED frame.
     * @return Cross-track error in metres, or 0.0 if the path is invalid.
     */
    double CrossTrackError(double drone_x, double drone_y) const noexcept {
        if (!has_valid_path || !current.valid || !previous.valid) {
            return 0.0;
        }

        const Eigen::Vector2d previous_xy = previous.position.head<2>();
        const Eigen::Vector2d current_xy = current.position.head<2>();
        const Eigen::Vector2d drone_xy(drone_x, drone_y);

        const Eigen::Vector2d segment = current_xy - previous_xy;
        const double len_sq = segment.squaredNorm();

        static constexpr double kMinSegmentLengthSq = 0.01 * 0.01;
        if (len_sq < kMinSegmentLengthSq) {
            return (drone_xy - current_xy).norm();
        }

        const double t = std::max(0.0, (drone_xy - previous_xy).dot(segment) / len_sq);
        const Eigen::Vector2d projection = previous_xy + t * segment;
        return (drone_xy - projection).norm();
    }
};

}  // namespace px4_common

#endif  // PX4_COMMON_TYPES_HPP_
