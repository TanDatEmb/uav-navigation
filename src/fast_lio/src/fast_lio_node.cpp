#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "fast_lio/commons.hpp"
#include "fast_lio/estimator/estimator.hpp"
#include "fast_lio/estimator/ikfom_estimator.hpp"
#include "fast_lio/ieskf.hpp"
#include "fast_lio/input/lidar_input_adapter.hpp"
#include "fast_lio/input/mid360_custom_adapter.hpp"
#include "fast_lio/input/pointcloud2_adapter.hpp"
#include "fast_lio/lidar_scan.hpp"
#include "fast_lio/map_builder.hpp"
#include "fast_lio/measurement_synchronizer.hpp"
#include "fast_lio/pointcloud2_decoder.hpp"
#include "fast_lio/utils.hpp"

using namespace std::chrono_literals;

namespace fast_lio {

/**
 * @brief FAST-LIO2 Node Configuration
 *
 * Frame conventions (docs/frame_contract.md):
 *   - body_frame: "mid360_imu" (IMU frame, FLU)
 *   - world_frame: "lio_world" (FAST-LIO world, gravity-aligned)
 */
struct NodeConfig {
    std::string imu_topic = "/livox/mid360/imu";
    std::string lidar_topic = "/livox/mid360/points";
    std::string body_frame = "mid360_imu";
    std::string world_frame = "lio_world";
    bool print_time_cost = false;
    std::size_t path_max_poses = 2000;
};

/**
 * @brief FAST-LIO2 ROS2 Node (15-DOF)
 *
 * Integrates IMU propagation and LiDAR scan matching with an Estimator backend.
 * Outputs odometry, registered clouds, and TF transforms.
 */
class FastLIONode : public rclcpp::Node {
   public:
    FastLIONode() : Node("fast_lio") {
        RCLCPP_INFO(this->get_logger(), "FAST-LIO2 (15-DOF) Node Starting...");

        loadParameters();

        // Initialize the LiDAR input adapter. The adapter hides sensor-specific
        // decoding (PointCloud2 snapshot, MID-360 PointCloud2, Livox CustomMsg).
        m_lidar_adapter = createLidarAdapter();

        // Initialize the MeasurementSynchronizer
        m_synchronizer = std::make_unique<MeasurementSynchronizer>(m_sync_config);

        // Subscriptions
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_node_config.imu_topic, 10,
            std::bind(&FastLIONode::imuCallback, this, std::placeholders::_1));

        // LiDAR subscription: profile selects PointCloud2 or Livox CustomMsg.
        if (m_adapter_type == "mid360_custom") {
#ifdef LIVOX_ROS2_FOUND
            m_custom_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                m_node_config.lidar_topic, 10,
                std::bind(&FastLIONode::customLidarCallback, this, std::placeholders::_1));
#else
            throw std::runtime_error(
                "lidar_input.adapter=mid360_custom requires livox_ros_driver2. "
                "Install the driver and rebuild.");
#endif
        } else {
            m_lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                m_node_config.lidar_topic, 10,
                std::bind(&FastLIONode::lidarCallback, this, std::placeholders::_1));
        }

        // Publishers
        m_world_cloud_pub =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", 10);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Initialize path
        m_path.poses.clear();
        m_path.poses.reserve(m_node_config.path_max_poses);
        m_path.header.frame_id = m_node_config.world_frame;

        // Initialize KF and MapBuilder with 15-DOF state
        if (m_builder_config.estimator_backend == "ikfom") {
            m_kf = std::make_shared<IkfomEstimator>();
        } else {
            m_kf = std::make_shared<IESKF>();
        }
        RCLCPP_INFO(this->get_logger(), "Estimator backend: %s",
                    m_builder_config.estimator_backend.c_str());
        m_builder = std::make_unique<MapBuilder>(m_builder_config, m_kf);

        // Timer for main loop (50Hz)
        m_timer = this->create_wall_timer(20ms, std::bind(&FastLIONode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "FAST-LIO2 Node Started");
        RCLCPP_INFO(this->get_logger(), "LiDAR adapter: %s",
                    m_lidar_adapter ? m_lidar_adapter->name().c_str() : "none");
        RCLCPP_INFO(this->get_logger(), "Body frame: %s, World frame: %s",
                    m_node_config.body_frame.c_str(), m_node_config.world_frame.c_str());
    }

   private:
    void loadParameters() {
        // ROS 2 parameters are the single configuration source. The launch file
        // loads fast_lio.params.yaml, and later launch/CLI overrides retain the
        // standard ROS 2 precedence instead of being overwritten by a second YAML parser.
        m_node_config.imu_topic =
            this->declare_parameter<std::string>("imu_topic", m_node_config.imu_topic);
        m_node_config.lidar_topic =
            this->declare_parameter<std::string>("lidar_topic", m_node_config.lidar_topic);
        m_node_config.body_frame =
            this->declare_parameter<std::string>("body_frame", m_node_config.body_frame);
        m_node_config.world_frame =
            this->declare_parameter<std::string>("world_frame", m_node_config.world_frame);
        // --- LiDAR input adapter selection ---
        // Adapter decouples the node from sensor-specific message formats.
        // Supported: "sim_snapshot", "mid360_pointcloud2", "mid360_custom".
        m_adapter_type =
            this->declare_parameter<std::string>("lidar_input.adapter", "");
        if (m_adapter_type.empty()) {
            // Backward compatibility: map legacy lidar_input.profile to adapter.
            const std::string input_profile_str =
                this->declare_parameter<std::string>("lidar_input.profile", "sim_xyzi_snapshot");
            if (input_profile_str == "mid360_pointcloud2") {
                m_adapter_type = "mid360_pointcloud2";
            } else {
                m_adapter_type = "sim_snapshot";
            }
        } else {
            // Declare legacy profile so existing parameter files do not fail,
            // but do not use its value when adapter is explicit.
            this->declare_parameter<std::string>("lidar_input.profile", "sim_xyzi_snapshot");
        }

        // Validation of m_adapter_type happens after the 'fail' helper is defined.

        m_node_config.print_time_cost =
            this->declare_parameter<bool>("print_time_cost", m_node_config.print_time_cost);

        // --- Synchronizer configuration ---
        const int sync_max_imu = this->declare_parameter<int>(
            "synchronizer.max_imu_samples", 4000);
        const int sync_max_lidar = this->declare_parameter<int>(
            "synchronizer.max_lidar_scans", 5);
        m_sync_config.max_imu_gap_s =
            this->declare_parameter<double>("synchronizer.max_imu_gap_s", 0.02);
        m_sync_config.require_imu_before_scan_start =
            this->declare_parameter<bool>("synchronizer.require_imu_before_scan_start", true);
        m_sync_config.require_imu_after_scan_end =
            this->declare_parameter<bool>("synchronizer.require_imu_after_scan_end", true);

        if (sync_max_imu > 0) m_sync_config.max_imu_samples = static_cast<std::size_t>(sync_max_imu);
        if (sync_max_lidar > 0) m_sync_config.max_lidar_scans = static_cast<std::size_t>(sync_max_lidar);

        const int path_max_poses = this->declare_parameter<int>(
            "path_max_poses", static_cast<int>(m_node_config.path_max_poses));

        m_builder_config.lidar_min_range =
            this->declare_parameter<double>("lidar_min_range", m_builder_config.lidar_min_range);
        m_builder_config.lidar_max_range =
            this->declare_parameter<double>("lidar_max_range", m_builder_config.lidar_max_range);
        m_builder_config.scan_resolution =
            this->declare_parameter<double>("scan_resolution", m_builder_config.scan_resolution);
        m_builder_config.map_resolution =
            this->declare_parameter<double>("map_resolution", m_builder_config.map_resolution);
        m_builder_config.cube_len =
            this->declare_parameter<double>("cube_len", m_builder_config.cube_len);
        m_builder_config.det_range =
            this->declare_parameter<double>("det_range", m_builder_config.det_range);
        m_builder_config.move_thresh =
            this->declare_parameter<double>("move_thresh", m_builder_config.move_thresh);
        m_builder_config.na = this->declare_parameter<double>("na", m_builder_config.na);
        m_builder_config.ng = this->declare_parameter<double>("ng", m_builder_config.ng);
        m_builder_config.nba = this->declare_parameter<double>("nba", m_builder_config.nba);
        m_builder_config.nbg = this->declare_parameter<double>("nbg", m_builder_config.nbg);
        m_builder_config.imu_init_num =
            this->declare_parameter<int>("imu_init_num", m_builder_config.imu_init_num);
        m_builder_config.imu_init_accel_std_max = this->declare_parameter<double>(
            "imu_init_accel_std_max", m_builder_config.imu_init_accel_std_max);
        m_builder_config.imu_init_gyro_rms_max = this->declare_parameter<double>(
            "imu_init_gyro_rms_max", m_builder_config.imu_init_gyro_rms_max);
        m_builder_config.imu_init_gravity_tolerance = this->declare_parameter<double>(
            "imu_init_gravity_tolerance", m_builder_config.imu_init_gravity_tolerance);
        m_builder_config.knn_search_count =
            this->declare_parameter<int>("knn_search_count", m_builder_config.knn_search_count);
        m_builder_config.ieskf_max_iter =
            this->declare_parameter<int>("ieskf_max_iter", m_builder_config.ieskf_max_iter);
        m_builder_config.estimator_backend =
            this->declare_parameter<std::string>("estimator_backend", m_builder_config.estimator_backend);
        m_builder_config.gravity_align =
            this->declare_parameter<bool>("gravity_align", m_builder_config.gravity_align);
        m_builder_config.lidar_cov_inv =
            this->declare_parameter<double>("lidar_cov_inv", m_builder_config.lidar_cov_inv);

        const auto extrinsic_t = this->declare_parameter<std::vector<double>>(
            "extrinsic_t",
            {m_builder_config.t_I_L.x(), m_builder_config.t_I_L.y(), m_builder_config.t_I_L.z()});
        const auto extrinsic_r = this->declare_parameter<std::vector<double>>(
            "extrinsic_r",
            {m_builder_config.R_I_L(0, 0), m_builder_config.R_I_L(0, 1), m_builder_config.R_I_L(0, 2),
             m_builder_config.R_I_L(1, 0), m_builder_config.R_I_L(1, 1), m_builder_config.R_I_L(1, 2),
             m_builder_config.R_I_L(2, 0), m_builder_config.R_I_L(2, 1),
             m_builder_config.R_I_L(2, 2)});

        const auto fail = [this](const std::string& reason) {
            RCLCPP_FATAL(this->get_logger(), "Invalid FAST-LIO parameter: %s", reason.c_str());
            throw std::invalid_argument(reason);
        };

        if (m_adapter_type != "sim_snapshot" &&
            m_adapter_type != "mid360_pointcloud2" &&
            m_adapter_type != "mid360_custom") {
            fail("lidar_input.adapter must be one of: sim_snapshot, mid360_pointcloud2, mid360_custom");
        }

        if (m_node_config.imu_topic.empty() || m_node_config.lidar_topic.empty()) {
            fail("imu_topic and lidar_topic must not be empty");
        }
        if (m_node_config.body_frame.empty() || m_node_config.world_frame.empty()) {
            fail("body_frame and world_frame must not be empty");
        }
        if (path_max_poses <= 0) {
            fail("path_max_poses must be positive");
        }
        if (m_builder_config.lidar_min_range < 0.0 ||
            m_builder_config.lidar_max_range <= m_builder_config.lidar_min_range) {
            fail("lidar range must satisfy 0 <= lidar_min_range < lidar_max_range");
        }
        if (m_builder_config.scan_resolution < 0.0 || m_builder_config.map_resolution <= 0.0) {
            fail("scan_resolution must be non-negative and map_resolution must be positive");
        }
        if (m_builder_config.cube_len <= 0.0 || m_builder_config.det_range <= 0.0 ||
            m_builder_config.move_thresh <= 0.0 || m_builder_config.move_thresh > 1.0) {
            fail("cube_len/det_range must be positive and move_thresh must be in (0, 1]");
        }
        if (m_builder_config.na < 0.0 || m_builder_config.ng < 0.0 || m_builder_config.nba < 0.0 ||
            m_builder_config.nbg < 0.0) {
            fail("IMU noise values must be non-negative");
        }
        if (m_builder_config.imu_init_num <= 0 || m_builder_config.imu_init_accel_std_max < 0.0 ||
            m_builder_config.imu_init_gyro_rms_max < 0.0 ||
            m_builder_config.imu_init_gravity_tolerance < 0.0) {
            fail("IMU initialization limits are invalid");
        }
        if (m_builder_config.knn_search_count < 5 || m_builder_config.ieskf_max_iter <= 0 ||
            m_builder_config.lidar_cov_inv <= 0.0) {
            fail("knn_search_count must be >= 5; ieskf_max_iter and lidar_cov_inv must be positive");
        }
        if (m_builder_config.estimator_backend != "ieskf" &&
            m_builder_config.estimator_backend != "ikfom") {
            fail("estimator_backend must be one of: ieskf, ikfom");
        }
        if (extrinsic_t.size() != 3 || extrinsic_r.size() != 9) {
            fail("extrinsic_t must contain 3 values and extrinsic_r must contain 9 values");
        }

        m_node_config.path_max_poses = static_cast<std::size_t>(path_max_poses);
        m_builder_config.t_I_L = Eigen::Vector3d(extrinsic_t[0], extrinsic_t[1], extrinsic_t[2]);
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                m_builder_config.R_I_L(row, col) = extrinsic_r[row * 3 + col];
            }
        }
        const Eigen::Matrix3d rotation_error =
            m_builder_config.R_I_L.transpose() * m_builder_config.R_I_L - Eigen::Matrix3d::Identity();
        if (!m_builder_config.R_I_L.allFinite() || rotation_error.norm() > 1e-3 ||
            std::abs(m_builder_config.R_I_L.determinant() - 1.0) > 1e-3) {
            fail("extrinsic_r must be a finite right-handed rotation matrix");
        }

        RCLCPP_INFO(this->get_logger(),
                    "ROS parameters loaded: scan=%.2fm map=%.2fm KNN=%zu IESKF=%zu range=[%.1f, "
                    "%.1f]m",
                    m_builder_config.scan_resolution, m_builder_config.map_resolution,
                    m_builder_config.knn_search_count,
                    static_cast<std::size_t>(m_builder_config.ieskf_max_iter),
                    m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        RCLCPP_INFO(this->get_logger(), "LiDAR-IMU translation: [%.3f, %.3f, %.3f]",
                    m_builder_config.t_I_L.x(), m_builder_config.t_I_L.y(),
                    m_builder_config.t_I_L.z());
    }

    std::unique_ptr<LidarInputAdapter> createLidarAdapter() const {
        if (m_adapter_type == "sim_snapshot") {
            return makeSimSnapshotAdapter();
        }
        if (m_adapter_type == "mid360_pointcloud2") {
            return makeMid360PointCloud2Adapter();
        }
        if (m_adapter_type == "mid360_custom") {
            return makeMid360CustomAdapter();
        }
        // Should never reach here because loadParameters validates the type.
        throw std::invalid_argument("Unknown LiDAR adapter type: " + m_adapter_type);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        IMUData imu;
        imu.acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                  msg->linear_acceleration.z);
        imu.gyro = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                                   msg->angular_velocity.z);
        imu.time = timestamp;
        if (!imu.acc.allFinite() || !imu.gyro.allFinite() || !std::isfinite(imu.time)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Ignoring non-finite IMU sample");
            return;
        }

        m_synchronizer->pushImu(imu);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "IMU accepted: t=%.6f gyro=[%.4f %.4f %.4f] acc=[%.4f %.4f %.4f]",
                             imu.time, imu.gyro.x(), imu.gyro.y(), imu.gyro.z(),
                             imu.acc.x(), imu.acc.y(), imu.acc.z());

        processReadyPackages();
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        DecodeResult result = m_lidar_adapter->decode(*msg);
        handleDecodeResult(result);
    }

#ifdef LIVOX_ROS2_FOUND
    void customLidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
        DecodeResult result = m_lidar_adapter->decode(*msg);
        handleDecodeResult(result);
    }
#endif

    void handleDecodeResult(DecodeResult& result) {
        if (!result.ok()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "LiDAR decode failed: %s", result.errorMessage().c_str());
            return;
        }

        const auto& scan = result.scan;
        const double scan_start_s = static_cast<double>(scan.scan_start_time_ns) * 1e-9;
        const double scan_end_s = static_cast<double>(scan.scan_end_time_ns) * 1e-9;
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "LiDAR accepted: points=%zu timed=%d scan=[%.6f, %.6f] duration=%.6f frame='%s'",
            scan.cloud ? scan.cloud->size() : 0U, static_cast<int>(scan.has_per_point_time),
            scan_start_s, scan_end_s, scan_end_s - scan_start_s, scan.lidar_frame.c_str());

        m_synchronizer->pushLidar(result.scan);
        processReadyPackages();
    }

    void processReadyPackages() {
        while (true) {
            SyncResult sync_result = m_synchronizer->tryPop();
            const auto diag = m_synchronizer->diagnostics();

            if (!sync_result.ready()) {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Sync status=%d imu_buf=%zu lidar_buf=%zu "
                    "imu_recv=%zu lidar_recv=%zu pkgs=%zu "
                    "drops_start=%zu drops_gap=%zu regressions=%zu",
                    static_cast<int>(sync_result.status), diag.imu_buffer_size,
                    diag.lidar_buffer_size, diag.imu_received, diag.lidar_received,
                    diag.packages_emitted, diag.dropped_no_start_coverage,
                    diag.dropped_imu_gap, diag.imu_out_of_order);

                if (sync_result.status == SyncStatus::kDropNoStartCoverage) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "Dropped scan: no IMU coverage at scan start");
                } else if (sync_result.status == SyncStatus::kDropImuGap) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "Dropped scan: IMU gap too large");
                }
                break;
            }

            const auto& package = sync_result.package;
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Sync READY: scan=[%.6f, %.6f] imus=%zu imu=[%.6f, %.6f]",
                package.cloud_start_time, package.cloud_end_time,
                package.imus.size(), package.imus.front().time,
                package.imus.back().time);

            m_package = std::move(sync_result.package);
            processPackage();

            // Log IMU initialization progress before publishing
            const auto init_diag = m_builder->imuInitializationDiagnostics();
            if (!init_diag.initialized) {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "IMU init: samples=%zu/%zu acc_mean=[%.4f %.4f %.4f] "
                    "acc_norm=%.4f accel_std=%.4f gyro_rms=%.6f gravity_err=%.4f stationary=%d",
                    init_diag.collected_samples, init_diag.required_samples,
                    init_diag.mean_acceleration.x(), init_diag.mean_acceleration.y(),
                    init_diag.mean_acceleration.z(), init_diag.acceleration_norm,
                    init_diag.accel_std, init_diag.gyro_rms, init_diag.gravity_error,
                    static_cast<int>(init_diag.stationary));
            }

            publishResults();
        }
    }

    void timerCallback() {
        processReadyPackages();
    }

    void processPackage() {
        auto t1 = std::chrono::high_resolution_clock::now();

        LidarUpdateResult result = m_builder->process(m_package);

        auto t2 = std::chrono::high_resolution_clock::now();

        if (m_node_config.print_time_cost) {
            auto time_used =
                std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_INFO(this->get_logger(), "Processing time: %.2f ms", time_used);
        }

        // Throttled update result logging
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "LidarUpdate: status=%d input=%zu down=%zu queried=%zu planes=%zu "
            "accepted=%zu update=%d inserted=%d converged=%d "
            "res=[mean=%.4f abs=%.4f rms=%.4f max=%.4f]",
            static_cast<int>(result.status), result.input_points, result.downsampled_points,
            result.queried_points, result.plane_candidates, result.accepted_correspondences,
            static_cast<int>(result.update_applied), static_cast<int>(result.map_inserted),
            static_cast<int>(result.converged), result.residual.mean_signed,
            result.residual.mean_absolute, result.residual.rms, result.residual.max_absolute);

        if (m_builder->status() != BuilderStatus::MAPPING) {
            return;
        }

        // Throttled state logging for debugging coordinate drift
        const auto& state = m_builder->state();
        const Eigen::AngleAxisd aa(state.R_wb.matrix());
        const Eigen::Vector3d euler = aa.angle() * aa.axis() * 180.0 / M_PI;
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "LIO state: p=[%.3f %.3f %.3f] v=[%.3f %.3f %.3f] "
            "euler=[%.3f %.3f %.3f] ba=[%.4f %.4f %.4f] bw=[%.5f %.5f %.5f]",
            state.p_w.x(), state.p_w.y(), state.p_w.z(), state.v_w.x(), state.v_w.y(),
            state.v_w.z(), euler.x(), euler.y(), euler.z(), state.b_a.x(), state.b_a.y(),
            state.b_a.z(), state.b_w.x(), state.b_w.y(), state.b_w.z());
    }

    void publishResults() {
        // Guard: do not publish odometry/path/cloud before estimator is tracking
        const auto builder_status = m_builder->status();
        if (builder_status != BuilderStatus::MAPPING) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Skipping publish: builder status=%d (not tracking yet)",
                                 static_cast<int>(builder_status));
            return;
        }

        double publish_time = m_package.cloud_end_time;

        // Get the current IMU pose from the 15-DOF state.
        SE3d T_world_imu = m_builder->getIMUPose();

        // Publish TF: lio_world -> mid360_imu
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = utils::getTime(publish_time);
        transform.header.frame_id = m_node_config.world_frame;
        transform.child_frame_id = m_node_config.body_frame;
        transform.transform = utils::SE3ToTransform(T_world_imu);
        m_tf_broadcaster->sendTransform(transform);

        // Publish odometry (IMU pose)
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = utils::getTime(publish_time);
        odom.header.frame_id = m_node_config.world_frame;
        odom.child_frame_id = m_node_config.body_frame;
        utils::SE3ToOdometry(T_world_imu, m_node_config.world_frame, m_node_config.body_frame,
                             odom);
        utils::setOdometryPoseCovariance(m_kf->getCovariance(), odom);
        m_odom_pub->publish(odom);

        // Update and publish path
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = utils::getTime(publish_time);
        pose.header.frame_id = m_node_config.world_frame;
        pose.pose = odom.pose.pose;
        utils::appendBoundedPose(m_path, pose, m_node_config.path_max_poses);
        m_path_pub->publish(m_path);

        // Publish registered cloud
        auto world_cloud = m_builder->getLidarProcessor()->getWorldCloud();
        if (world_cloud && !world_cloud->empty()) {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*world_cloud, cloud_msg);
            cloud_msg.header.stamp = utils::getTime(publish_time);
            cloud_msg.header.frame_id = m_node_config.world_frame;
            m_world_cloud_pub->publish(cloud_msg);
        }
    }

    // Members
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_sub;
#ifdef LIVOX_ROS2_FOUND
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_custom_lidar_sub;
#endif

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::TimerBase::SharedPtr m_timer;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    SynchronizerConfig m_sync_config;
    nav_msgs::msg::Path m_path;
    std::string m_adapter_type;

    std::shared_ptr<Estimator> m_kf;
    std::unique_ptr<MapBuilder> m_builder;
    std::unique_ptr<LidarInputAdapter> m_lidar_adapter;
    std::unique_ptr<MeasurementSynchronizer> m_synchronizer;
};

}  // namespace fast_lio

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fast_lio::FastLIONode>());
    rclcpp::shutdown();
    return 0;
}
