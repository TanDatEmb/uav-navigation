#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
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
#include <fstream>
#include <mutex>

#include "fast_lio/commons.hpp"
#include "fast_lio/map_builder.hpp"
#include "fast_lio/utils.hpp"

using namespace std::chrono_literals;

namespace fast_lio {

/**
 * @brief FAST-LIO2 Node Configuration
 *
 * Frame conventions (frame_contract_v1.md):
 *   - body_frame: "mid360_imu" (IMU frame, FLU)
 *   - world_frame: "lio_world" (FAST-LIO world, gravity-aligned)
 *   - lidar_frame: "mid360_lidar" (LiDAR optical center, FLU)
 */
struct NodeConfig {
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "mid360_imu";
    std::string world_frame = "lio_world";
    std::string lidar_frame = "mid360_lidar";
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

        // Load extrinsic
        loadExtrinsic();

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
        this->declare_parameter<std::string>("config_path", "");
        this->declare_parameter<std::string>("extrinsic_path", "");

        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        if (!config_path.empty()) {
            RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_path.c_str());
            try {
                YAML::Node config = YAML::LoadFile(config_path);

                m_node_config.imu_topic = config["imu_topic"].as<std::string>();
                m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
                m_node_config.body_frame = config["body_frame"].as<std::string>();
                m_node_config.world_frame = config["world_frame"].as<std::string>();
                m_node_config.print_time_cost = config["print_time_cost"].as<bool>();
                m_node_config.sim_mode = config["sim_mode"].as<bool>();
                if (config["path_max_poses"]) {
                    const int configured_max = config["path_max_poses"].as<int>();
                    if (configured_max > 0) {
                        m_node_config.path_max_poses = static_cast<std::size_t>(configured_max);
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                                    "path_max_poses must be positive; using default %zu",
                                    m_node_config.path_max_poses);
                    }
                }
                if (config["imu_buffer_max_samples"]) {
                    const int configured_max = config["imu_buffer_max_samples"].as<int>();
                    if (configured_max > 0) {
                        m_node_config.imu_buffer_max_samples =
                            static_cast<std::size_t>(configured_max);
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                                    "imu_buffer_max_samples must be positive; using default %zu",
                                    m_node_config.imu_buffer_max_samples);
                    }
                }
                if (config["lidar_buffer_max_scans"]) {
                    const int configured_max = config["lidar_buffer_max_scans"].as<int>();
                    if (configured_max > 0) {
                        m_node_config.lidar_buffer_max_scans =
                            static_cast<std::size_t>(configured_max);
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                                    "lidar_buffer_max_scans must be positive; using default %zu",
                                    m_node_config.lidar_buffer_max_scans);
                    }
                }

                m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
                m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
                m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
                m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
                m_builder_config.map_resolution = config["map_resolution"].as<double>();
                m_builder_config.cube_len = config["cube_len"].as<double>();
                m_builder_config.det_range = config["det_range"].as<double>();
                m_builder_config.move_thresh = config["move_thresh"].as<double>();
                m_builder_config.na = config["na"].as<double>();
                m_builder_config.ng = config["ng"].as<double>();
                m_builder_config.nba = config["nba"].as<double>();
                m_builder_config.nbg = config["nbg"].as<double>();
                m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
                if (config["imu_init_accel_std_max"]) {
                    m_builder_config.imu_init_accel_std_max =
                        config["imu_init_accel_std_max"].as<double>();
                }
                if (config["imu_init_gyro_rms_max"]) {
                    m_builder_config.imu_init_gyro_rms_max =
                        config["imu_init_gyro_rms_max"].as<double>();
                }
                if (config["imu_init_gravity_tolerance"]) {
                    m_builder_config.imu_init_gravity_tolerance =
                        config["imu_init_gravity_tolerance"].as<double>();
                }
                m_builder_config.near_search_num = config["near_search_num"].as<int>();
                m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
                m_builder_config.gravity_align = config["gravity_align"].as<bool>();
                m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

                // Load extrinsic from same config
                if (config["extrinsic_t"]) {
                    std::vector<double> t_vec = config["extrinsic_t"].as<std::vector<double>>();
                    m_builder_config.t_il << t_vec[0], t_vec[1], t_vec[2];
                }
                if (config["extrinsic_r"]) {
                    std::vector<double> r_vec = config["extrinsic_r"].as<std::vector<double>>();
                    for (int i = 0; i < 3; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            m_builder_config.r_il(i, j) = r_vec[i * 3 + j];
                        }
                    }
                }

                RCLCPP_INFO(this->get_logger(), "Loaded config from: %s", config_path.c_str());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to load config: %s. Using defaults.",
                            e.what());
            }
        }
    }

    void loadExtrinsic() {
        // Extrinsic already loaded in loadParameters
        // Log the values
        RCLCPP_INFO(this->get_logger(), "LiDAR-IMU extrinsic:");
        RCLCPP_INFO(this->get_logger(), "  Translation: [%.3f, %.3f, %.3f]",
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
