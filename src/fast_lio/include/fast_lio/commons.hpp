#ifndef FAST_LIO_COMMONS_HPP_
#define FAST_LIO_COMMONS_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <deque>
#include <vector>

// Sophus optional - use Eigen directly if not available
#ifdef USE_EIGEN_SE3
#include "fast_lio/eigen_se3.hpp"
namespace fast_lio {
using SO3d = ::fast_lio::SO3d;
using SE3d = ::fast_lio::SE3d;
}  // namespace fast_lio
#else
#include <sophus/se3.hpp>
namespace fast_lio {
using SO3d = Sophus::SO3d;
using SE3d = Sophus::SE3d;
}  // namespace fast_lio
#endif

namespace fast_lio {

// ============================================================
// Type Aliases
// ============================================================
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;

// Legacy 21-DOF (for reference)
using V21D = Eigen::Matrix<double, 21, 1>;

// NEW: 15-DOF for UAV (Math Expert Review)
using V15D = Eigen::Matrix<double, 15, 1>;

// ============================================================
// Point Types
// ============================================================

// Point with timestamp for deskew
struct EIGEN_ALIGN16 PointXYZIT {
    PCL_ADD_POINT4D;
    float intensity;
    float curvature;  // Time offset in ms (for deskew)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Main point type (PCL)
using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

// ============================================================
// State Representations
// ============================================================

/**
 * @brief 15-DOF FAST-LIO state for UAV localization.
 *
 * State: [R, p, v, b_a, b_ω] ∈ SO(3) × ℝ¹²
 * - R ∈ SO(3): Body-to-world rotation
 * - p ∈ ℝ³: Position in world
 * - v ∈ ℝ³: Velocity in world
 * - b_a ∈ ℝ³: Accel bias
 * - b_ω ∈ ℝ³: Gyro bias
 *
 * Gravity is constant in the z-up LIO world frame (not estimated).
 * LiDAR-IMU extrinsic is calibrated offline (not in state).
 *
 * Uses a right perturbation on SO(3), matching the point-to-plane Jacobian.
 */
struct State15 {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Core state
    SO3d R_wb;            // Body-to-world rotation (kept name for API compatibility)
    Eigen::Vector3d p_w;  // Position in world
    Eigen::Vector3d v_w;  // Velocity in world
    Eigen::Vector3d b_a;  // Accel bias
    Eigen::Vector3d b_w;  // Gyro bias

    // LiDAR extrinsic (IMU to LiDAR, constant)
    SE3d T_imu_lidar;

    State15() {
        R_wb = SO3d();
        p_w = Eigen::Vector3d::Zero();
        v_w = Eigen::Vector3d::Zero();
        b_a = Eigen::Vector3d::Zero();
        b_w = Eigen::Vector3d::Zero();
        T_imu_lidar = SE3d();
    }

    /**
     * @brief Update with a right-perturbation error state
     *
     * delta_x = [δθ, δp, δv, δb_a, δb_ω]
     * R ← R * exp(δθ)
     */
    void update(const V15D& delta_x) {
        R_wb = R_wb * SO3d::exp(delta_x.segment<3>(0));
        p_w += delta_x.segment<3>(3);
        v_w += delta_x.segment<3>(6);
        b_a += delta_x.segment<3>(9);
        b_w += delta_x.segment<3>(12);
    }

    /**
     * @brief Get full pose (for external vision)
     *
     * T_world_imu = [R, p]
     */
    SE3d getPose() const {
        return SE3d(R_wb, p_w);
    }

    /**
     * @brief Get LiDAR pose (for point cloud registration)
     *
     * T_world_lidar = T_world_imu * T_imu_lidar
     */
    SE3d getLiDARPose() const {
        SE3d T_world_imu(R_wb, p_w);
        return T_world_imu * T_imu_lidar;
    }
};

// ============================================================
// Shared State (for IESKF)
// ============================================================

/**
 * @brief Linearized LiDAR measurement data for the 15-DOF IESKF.
 *
 * Contains stacked Jacobians and residuals for IESKF update.
 * Dynamic sizing for variable number of measurements.
 */
struct SharedState15 {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool valid = false;
    Eigen::MatrixXd H;  // N measurements × 15 state
    Eigen::VectorXd b;  // N × 1 residuals
    int num_measurements = 0;

    SharedState15() {
        H.resize(1000, 15);
        b.resize(1000);
        H.setZero();
        b.setZero();
    }

    void reset(int max_measurements = 1000) {
        H.setZero(max_measurements, 15);
        b.setZero(max_measurements);
        valid = false;
        num_measurements = 0;
    }
};

// ============================================================
// IMU Data
// ============================================================
struct IMUData {
    V3D acc;      // Acceleration in IMU frame [m/s²]
    V3D gyro;     // Angular velocity in IMU frame [rad/s]
    double time;  // Timestamp in seconds

    IMUData() : acc(V3D::Zero()), gyro(V3D::Zero()), time(0.0) {}
    IMUData(const V3D& a, const V3D& g, double t) : acc(a), gyro(g), time(t) {}
};

// ============================================================
// Synchronized Package (LiDAR + IMU)
// ============================================================
struct SyncPackage {
    CloudType::Ptr cloud;
    std::deque<IMUData> imus;
    double cloud_start_time;
    double cloud_end_time;

    SyncPackage() : cloud(new CloudType()), cloud_start_time(0.0), cloud_end_time(0.0) {}
};

// ============================================================
// Configuration
// ============================================================
struct Config {
    // LiDAR filter
    double lidar_min_range = 0.5;
    double lidar_max_range = 100.0;
    double scan_resolution = 0.1;
    double map_resolution = 0.2;

    // Local map
    double cube_len = 50.0;
    double det_range = 100.0;
    double move_thresh = 0.5;

    // IMU noise (MID-360 ICM-40609 typical)
    double na = 0.01;      // Accel noise [m/s²/√Hz]
    double ng = 0.001;     // Gyro noise [rad/s/√Hz]
    double nba = 0.0001;   // Accel bias random walk
    double nbg = 0.00001;  // Gyro bias random walk

    // Algorithm
    int imu_init_num = 50;
    double imu_init_accel_std_max = 0.5;
    double imu_init_gyro_rms_max = 0.1;
    double imu_init_gravity_tolerance = 3.0;
    int near_search_num = 5;
    int ieskf_max_iter = 3;
    bool gravity_align = true;

    // Extrinsic: LiDAR in IMU frame
    V3D t_il = V3D::Zero();
    M3D r_il = M3D::Identity();

    // Covariance
    double lidar_cov_inv = 200.0;
};

// ============================================================
// Box Point Type (for local map)
// ============================================================
struct BoxPointType {
    float vertex_min[3];
    float vertex_max[3];
};

// ============================================================
// Builder Status
// ============================================================
enum class BuilderStatus { INITIALIZING, MAPPING, ERROR };

// ============================================================
// Utility Functions
// ============================================================
inline double sq_dist(const PointType& p1, const PointType& p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
           (p1.z - p2.z) * (p1.z - p2.z);
}

/**
 * @brief Skew-symmetric matrix from vector
 * [v]× = [ 0   -v_z  v_y ]
 *        [ v_z  0   -v_x ]
 *        [-v_y  v_x  0   ]
 */
inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d M;
    M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return M;
}

/**
 * @brief Gravity vector in NED frame
 * NED: North-East-Down, so gravity is [0, 0, +9.81]
 */
inline Eigen::Vector3d GRAVITY_NED() {
    return Eigen::Vector3d(0.0, 0.0, 9.81);
}

/**
 * @brief Gravity vector in ENU frame
 * ENU: East-North-Up, so gravity is [0, 0, -9.81]
 */
inline Eigen::Vector3d GRAVITY_ENU() {
    return Eigen::Vector3d(0.0, 0.0, -9.81);
}

}  // namespace fast_lio

#endif  // FAST_LIO_COMMONS_HPP_
