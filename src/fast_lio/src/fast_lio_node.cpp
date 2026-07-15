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
#include <mutex>
#include <stdexcept>

#include "fast_lio/commons.hpp"
#include "fast_lio/map_builder.hpp"
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
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "mid360_imu";
    std::string world_frame = "lio_world";
    bool print_time_cost = false;
    bool sim_mode = true;  // Disable deskew in sim mode
    std::size_t path_max_poses = 2000;
    std::size_t imu_buffer_max_samples = 2000;
    std::size_t lidar_buffer_max_scans = 3;
};

/**
 * @brief State data for synchronization
 */
struct StateData {
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, CloudType::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

/**
 * @brief FAST-LIO2 ROS2 Node (15-DOF)
 *
 * Integrates IMU propagation and LiDAR scan matching with IESKF.
 * Outputs odometry, registered clouds, and TF transforms.
 */
class FastLIONode : public rclcpp::Node {
   public:
    FastLIONode() : Node("fast_lio") {
        RCLCPP_INFO(this->get_logger(), "FAST-LIO2 (15-DOF) Node Starting...");

        loadParameters();

        // Subscriptions
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_node_config.imu_topic, 10,
            std::bind(&FastLIONode::imuCallback, this, std::placeholders::_1));

        // LiDAR subscription - PointCloud2 (works with Livox sim adapter)
        m_lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_node_config.lidar_topic, 10,
            std::bind(&FastLIONode::lidarCallback, this, std::placeholders::_1));

        // Publishers
        m_world_cloud_pub =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", 10);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Initialize path
        m_state_data.path.poses.clear();
        m_state_data.path.poses.reserve(m_node_config.path_max_poses);
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        // Initialize KF and MapBuilder with 15-DOF state
        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_unique<MapBuilder>(m_builder_config, m_kf);

        // Timer for main loop (50Hz)
        m_timer = this->create_wall_timer(20ms, std::bind(&FastLIONode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "FAST-LIO2 Node Started");
        RCLCPP_INFO(this->get_logger(), "Sim mode: %s",
                    m_node_config.sim_mode ? "true (deskew disabled)" : "false");
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
        m_node_config.sim_mode = this->declare_parameter<bool>("sim_mode", m_node_config.sim_mode);
        m_node_config.print_time_cost =
            this->declare_parameter<bool>("print_time_cost", m_node_config.print_time_cost);

        const int path_max_poses = this->declare_parameter<int>(
            "path_max_poses", static_cast<int>(m_node_config.path_max_poses));
        const int imu_buffer_max_samples = this->declare_parameter<int>(
            "imu_buffer_max_samples", static_cast<int>(m_node_config.imu_buffer_max_samples));
        const int lidar_buffer_max_scans = this->declare_parameter<int>(
            "lidar_buffer_max_scans", static_cast<int>(m_node_config.lidar_buffer_max_scans));

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
        m_builder_config.near_search_num =
            this->declare_parameter<int>("near_search_num", m_builder_config.near_search_num);
        m_builder_config.ieskf_max_iter =
            this->declare_parameter<int>("ieskf_max_iter", m_builder_config.ieskf_max_iter);
        m_builder_config.gravity_align =
            this->declare_parameter<bool>("gravity_align", m_builder_config.gravity_align);
        m_builder_config.lidar_cov_inv =
            this->declare_parameter<double>("lidar_cov_inv", m_builder_config.lidar_cov_inv);

        const auto extrinsic_t = this->declare_parameter<std::vector<double>>(
            "extrinsic_t",
            {m_builder_config.t_il.x(), m_builder_config.t_il.y(), m_builder_config.t_il.z()});
        const auto extrinsic_r = this->declare_parameter<std::vector<double>>(
            "extrinsic_r",
            {m_builder_config.r_il(0, 0), m_builder_config.r_il(0, 1), m_builder_config.r_il(0, 2),
             m_builder_config.r_il(1, 0), m_builder_config.r_il(1, 1), m_builder_config.r_il(1, 2),
             m_builder_config.r_il(2, 0), m_builder_config.r_il(2, 1),
             m_builder_config.r_il(2, 2)});

        const auto fail = [this](const std::string& reason) {
            RCLCPP_FATAL(this->get_logger(), "Invalid FAST-LIO parameter: %s", reason.c_str());
            throw std::invalid_argument(reason);
        };

        if (m_node_config.imu_topic.empty() || m_node_config.lidar_topic.empty()) {
            fail("imu_topic and lidar_topic must not be empty");
        }
        if (m_node_config.body_frame.empty() || m_node_config.world_frame.empty()) {
            fail("body_frame and world_frame must not be empty");
        }
        if (path_max_poses <= 0 || imu_buffer_max_samples <= 0 || lidar_buffer_max_scans <= 0) {
            fail("path and buffer limits must be positive");
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
        if (m_builder_config.near_search_num < 5 || m_builder_config.ieskf_max_iter <= 0 ||
            m_builder_config.lidar_cov_inv <= 0.0) {
            fail("near_search_num must be >= 5; ieskf_max_iter and lidar_cov_inv must be positive");
        }
        if (extrinsic_t.size() != 3 || extrinsic_r.size() != 9) {
            fail("extrinsic_t must contain 3 values and extrinsic_r must contain 9 values");
        }

        m_node_config.path_max_poses = static_cast<std::size_t>(path_max_poses);
        m_node_config.imu_buffer_max_samples = static_cast<std::size_t>(imu_buffer_max_samples);
        m_node_config.lidar_buffer_max_scans = static_cast<std::size_t>(lidar_buffer_max_scans);
        m_builder_config.t_il = Eigen::Vector3d(extrinsic_t[0], extrinsic_t[1], extrinsic_t[2]);
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                m_builder_config.r_il(row, col) = extrinsic_r[row * 3 + col];
            }
        }
        const Eigen::Matrix3d rotation_error =
            m_builder_config.r_il.transpose() * m_builder_config.r_il - Eigen::Matrix3d::Identity();
        if (!m_builder_config.r_il.allFinite() || rotation_error.norm() > 1e-3 ||
            std::abs(m_builder_config.r_il.determinant() - 1.0) > 1e-3) {
            fail("extrinsic_r must be a finite right-handed rotation matrix");
        }

        RCLCPP_INFO(this->get_logger(),
                    "ROS parameters loaded: scan=%.2fm map=%.2fm KNN=%d IESKF=%d range=[%.1f, "
                    "%.1f]m",
                    m_builder_config.scan_resolution, m_builder_config.map_resolution,
                    m_builder_config.near_search_num, m_builder_config.ieskf_max_iter,
                    m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        RCLCPP_INFO(this->get_logger(), "LiDAR-IMU translation: [%.3f, %.3f, %.3f]",
                    m_builder_config.t_il.x(), m_builder_config.t_il.y(),
                    m_builder_config.t_il.z());
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);

        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if (timestamp < m_state_data.last_imu_time) {
            RCLCPP_WARN(this->get_logger(), "IMU message out of order, clearing buffer");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }

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

        m_state_data.imu_buffer.emplace_back(imu);
        const std::size_t dropped =
            utils::trimDequeFront(m_state_data.imu_buffer, m_node_config.imu_buffer_max_samples);
        if (dropped > 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "IMU backlog full; dropped %zu oldest samples", dropped);
        }
        m_state_data.last_imu_time = timestamp;
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Gazebo publishes XYZI only, while FAST-LIO's internal point type also
        // carries normals and per-point time in `curvature`. Converting directly
        // to PointXYZINormal emits missing-field warnings and can leave auxiliary
        // fields invalid, so normalize the wire format explicitly.
        pcl::PointCloud<pcl::PointXYZI> xyzi_cloud;
        pcl::fromROSMsg(*msg, xyzi_cloud);

        CloudType::Ptr cloud(new CloudType);
        cloud->header = xyzi_cloud.header;
        cloud->width = xyzi_cloud.width;
        cloud->height = xyzi_cloud.height;
        cloud->is_dense = xyzi_cloud.is_dense;
        cloud->points.resize(xyzi_cloud.points.size());
        for (std::size_t i = 0; i < xyzi_cloud.points.size(); ++i) {
            const auto& src = xyzi_cloud.points[i];
            auto& dst = cloud->points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
            dst.normal_x = 0.0f;
            dst.normal_y = 0.0f;
            dst.normal_z = 0.0f;
            dst.curvature = 0.0f;
        }
        if (cloud->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Ignoring empty LiDAR cloud");
            return;
        }

        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);

        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if (timestamp < m_state_data.last_lidar_time) {
            RCLCPP_WARN(this->get_logger(), "LiDAR message out of order, clearing buffer");
            std::deque<std::pair<double, CloudType::Ptr>>().swap(m_state_data.lidar_buffer);
            m_state_data.lidar_pushed = false;
        }

        // In sim mode, disable deskew by setting all curvature to 0
        if (m_node_config.sim_mode) {
            for (auto& pt : cloud->points) {
                pt.curvature = 0.0f;
            }
        }

        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        const std::size_t dropped =
            utils::trimDequeFront(m_state_data.lidar_buffer, m_node_config.lidar_buffer_max_scans);
        if (dropped > 0) {
            // The package cached by syncPackage() referenced the old front.
            // Abandon it so the next timer iteration binds the newest front.
            m_state_data.lidar_pushed = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "LiDAR backlog full; dropped %zu oldest scans", dropped);
        }
        m_state_data.last_lidar_time = timestamp;
    }

    bool syncPackage() {
        std::lock_guard<std::mutex> lock_imu(m_state_data.imu_mutex);
        std::lock_guard<std::mutex> lock_lidar(m_state_data.lidar_mutex);

        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty()) {
            return false;
        }

        if (!m_state_data.lidar_pushed) {
            m_package.cloud = m_state_data.lidar_buffer.front().second;

            // Sort points by curvature (time) if not in sim mode
            if (!m_node_config.sim_mode) {
                std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(),
                          [](const PointType& p1, const PointType& p2) {
                              return p1.curvature < p2.curvature;
                          });
            }

            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time =
                m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }

        if (m_state_data.last_imu_time < m_package.cloud_end_time) {
            return false;
        }

        // Collect IMU data for this scan
        m_package.imus.clear();
        while (!m_state_data.imu_buffer.empty() &&
               m_state_data.imu_buffer.front().time < m_package.cloud_end_time) {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }

        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;

        return true;
    }

    void timerCallback() {
        if (!syncPackage())
            return;

        auto t1 = std::chrono::high_resolution_clock::now();

        m_builder->process(m_package);

        auto t2 = std::chrono::high_resolution_clock::now();

        if (m_node_config.print_time_cost) {
            auto time_used =
                std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_INFO(this->get_logger(), "Processing time: %.2f ms", time_used);
        }

        if (m_builder->status() != BuilderStatus::MAPPING) {
            return;
        }

        // Publish results
        publishResults();
    }

    void publishResults() {
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
        utils::appendBoundedPose(m_state_data.path, pose, m_node_config.path_max_poses);
        m_path_pub->publish(m_state_data.path);

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

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::TimerBase::SharedPtr m_timer;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;

    std::shared_ptr<IESKF> m_kf;
    std::unique_ptr<MapBuilder> m_builder;
};

}  // namespace fast_lio

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fast_lio::FastLIONode>());
    rclcpp::shutdown();
    return 0;
}
