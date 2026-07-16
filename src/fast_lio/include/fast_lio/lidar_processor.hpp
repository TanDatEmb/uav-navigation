#ifndef FAST_LIO_LIDAR_PROCESSOR_HPP_
#define FAST_LIO_LIDAR_PROCESSOR_HPP_

#include "fast_lio/commons.hpp"
#include "fast_lio/ieskf.hpp"
#include "fast_lio/spatial_index.hpp"

#include <pcl/filters/voxel_grid.h>

#include <memory>

namespace fast_lio {

// Forward declarations from IESKF
struct State15;
struct SharedState15;

/**
 * @brief Local map management for LiDAR processing.
 */
struct LocalMap {
    bool initialized = false;
    BoxPointType corner;
    std::vector<BoxPointType> boxes_to_remove;

    void update(const Eigen::Vector3d& position, double cube_len, double move_thresh,
                double det_range);
};

/**
 * @brief Status of LiDAR processing operation.
 */
enum class LidarUpdateStatus {
    kUnknown = 0,                     ///\u003c Initial/unset status
    kSuccess = 1,                     ///\u003c IESKF update succeeded
    kMapInitialized = 2,              ///\u003c First scan - map initialized only
    kEmptyInput = 3,                  ///\u003c Empty input cloud
    kInsufficientDownsampledPoints = 4, ///\u003c Downsampled to too few points
    kNoMeasurements = 5,              ///\u003c No correspondence found
    kInsufficientMeasurements = 6,    ///\u003c Below minimum correspondence threshold
    kIeskfFailure = 7,                ///\u003c IESKF update failed
    kInvalidState = 8                 ///\u003c Invalid state detected
};

/**
 * @brief Residual statistics alias (defined in commons.hpp).
 */
using ResidualStatistics = fast_lio::ResidualStatistics;

/**
 * @brief Result of LiDAR processing operation with full diagnostics.
 */
struct LidarUpdateResult {
    LidarUpdateStatus status{LidarUpdateStatus::kUnknown};

    // Result flags
    bool update_applied{false};       ///\u003c IESKF state was corrected
    bool map_inserted{false};         ///\u003c Scan was inserted into map
    bool converged{false};            ///\u003c IESKF iterations converged

    // Point counts
    std::size_t input_points{0};              ///\u003c Raw input points
    std::size_t downsampled_points{0};        ///\u003c After voxel grid
    std::size_t queried_points{0};            ///\u003c Points sent to KNN
    std::size_t plane_candidates{0};          ///\u003c Points with valid plane
    std::size_t accepted_correspondences{0};  ///\u003c Passed isFinite() and residual gate

    // Iteration count
    std::size_t ieskf_iterations{0};

    // Rejection diagnostics
    RejectionStatistics rejection_stats;

    // Residual statistics (from final iteration if iterated)
    ResidualStatistics residual;

    // Timing (milliseconds)
    double downsample_time_ms{0.0};
    double correspondence_time_ms{0.0};
    double update_time_ms{0.0};
    double map_insertion_time_ms{0.0};

    // Nested IESKF result
    IeskfUpdateResult ieskf;

    [[nodiscard]] bool success() const {
        return status == LidarUpdateStatus::kSuccess ||
               status == LidarUpdateStatus::kMapInitialized;
    }
};

/**
 * @brief LiDAR feature extraction and point-to-plane ICP.
 *
 * Uses a 15-DOF state with right perturbation on SO(3).
 * Point-to-plane Jacobian: H = [-n^T * R * [p_body]×, n^T, 0, 0, 0]
 *
 * Where:
 *   - n: plane normal from map
 *   - R: rotation from IMU state
 *   - p_body: point in IMU frame
 *
 * Design reference: docs/ieskf_design.md
 */
class LidarProcessor {
   public:
    /**
     * @brief Constructor with map tree interface.
     *
     * @param config Algorithm configuration
     * @param kf Shared IESKF instance (15-DOF)
     * @param tree Map tree interface (ikd-Tree or PCL fallback)
     */
    LidarProcessor(const Config& config, std::shared_ptr<IESKF> kf,
                   std::shared_ptr<MapTreeInterface> tree);

    /**
     * @brief Process LiDAR scan with IESKF update.
     *
     * Pipeline:
     * 1. Downsample input cloud
     * 2. Find correspondences (point-to-plane)
     * 3. Setup loss function for IESKF
     * 4. IESKF iterated update
     * 5. Incremental map update
     *
     * @param package Synchronized LiDAR + IMU package
     * @return LidarUpdateResult with full diagnostics
     */
    LidarUpdateResult process(SyncPackage& package);

    /**
     * @brief Get current LiDAR pose in world frame.
     *
     * T_world_lidar = T_world_imu * T_I_L
     *
     * @return SE3 transform from LiDAR to world
     */
    SE3d getLiDARPose() const;

    /**
     * @brief Transform cloud using SE3 transform.
     */
    CloudType::Ptr transformCloud(const CloudType::Ptr& cloud, const SE3d& transform) const;

    /**
     * @brief Get downsampled cloud in LiDAR frame.
     */
    const CloudType::Ptr& getDownsampledCloud() const {
        return cloud_down_lidar_;
    }

    /**
     * @brief Get transformed cloud in world frame.
     */
    const CloudType::Ptr& getWorldCloud() const {
        return cloud_down_world_;
    }

   private:
    Config config_;
    std::shared_ptr<IESKF> kf_;
    std::shared_ptr<MapTreeInterface> map_tree_;

    // Processing buffers
    CloudType::Ptr cloud_down_lidar_;
    CloudType::Ptr cloud_down_world_;
    CloudType::Ptr cloud_effect_lidar_;
    CloudType::Ptr cloud_effect_world_;

    // Last shared state for diagnostics
    SharedState15 last_shared_state_;

    // Voxel grid filter for downsampling
    pcl::VoxelGrid<PointType> voxel_filter_;

    // Local map management
    LocalMap local_map_;

    // Initialize map with first scan
    void initMap(const CloudType::Ptr& cloud);

    // Update local map bounding box
    void updateLocalMap();

    // Incremental map update after IESKF convergence
    void incrementMap();

    /**
     * @brief Loss function for IESKF point-to-plane constraints.
     *
     * Computes Jacobians and residuals for each valid correspondence.
     *
     * Jacobian (1×15): H = [H_θ, H_p, 0, 0, 0]
     *   - H_θ = -n^T * R * [p_body]×  (1×3 rotation)
     *   - H_p = n^T                    (1×3 position)
     *   - 0 for velocity, accel bias, gyro bias
     *
     * Residual: r = n^T * (p_world - q_plane)
     *
     * @param state Current state estimate (15-DOF)
     * @param shared Output: stacked Jacobians and residuals
     */
    void computePointToPlaneConstraint(const State15& state, SharedState15& shared);

    /**
     * @brief Find plane correspondence for a point.
     *
     * @param point_lidar Point in LiDAR frame
     * @param point_world Point transformed to world frame
     * @param[out] normal Plane normal from map
     * @param[out] plane_point Point on plane
     * @return True if valid correspondence found
     */
    bool findPlaneCorrespondence(const Eigen::Vector3d& point_lidar,
                                 const Eigen::Vector3d& point_world, Eigen::Vector3d& normal,
                                 Eigen::Vector3d& plane_point);

    /**
     * @brief Estimate plane from k nearest neighbors.
     *
     * @param points Neighbor points
     * @param[out] normal Plane normal
     * @param[out] plane_point Centroid of neighbors
     * @return True if valid plane estimated
     */
    bool estimatePlane(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& normal,
                       Eigen::Vector3d& plane_point);
};

}  // namespace fast_lio

#endif  // FAST_LIO_LIDAR_PROCESSOR_HPP_
