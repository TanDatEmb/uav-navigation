#include "fast_lio/utils.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace fast_lio {
namespace utils {

#ifdef LIVOX_ROS2_FOUND
CloudType::Ptr livoxToPCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num,
                          double min_range, double max_range) {
    CloudType::Ptr cloud(new CloudType);

    for (size_t i = 0; i < msg->points.size(); i += filter_num) {
        const auto& pt = msg->points[i];

        double range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (range < min_range || range > max_range)
            continue;

        PointType pcl_pt;
        pcl_pt.x = pt.x;
        pcl_pt.y = pt.y;
        pcl_pt.z = pt.z;
        pcl_pt.intensity = static_cast<float>(pt.reflectivity);
        pcl_pt.curvature = static_cast<float>(pt.offset_time) / 1000.0f;  // ns to ms

        cloud->points.push_back(pcl_pt);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}
#endif

IMUData rosToIMU(const sensor_msgs::msg::Imu::SharedPtr msg) {
    IMUData imu;
    imu.acc =
        V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    imu.gyro = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    imu.time = static_cast<double>(msg->header.stamp.sec) +
               static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    return imu;
}

double getSec(const std_msgs::msg::Header& header) {
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time getTime(double sec) {
    builtin_interfaces::msg::Time time;
    time.sec = static_cast<int32_t>(sec);
    time.nanosec = static_cast<uint32_t>((sec - time.sec) * 1e9);
    return time;
}

geometry_msgs::msg::Transform SE3ToTransform(const SE3d& T) {
    geometry_msgs::msg::Transform transform;

    // Access translation and rotation from SE3d using accessor methods
    transform.translation.x = T.translation()(0);
    transform.translation.y = T.translation()(1);
    transform.translation.z = T.translation()(2);

    Eigen::Quaterniond q(T.rotation().matrix());
    transform.rotation.x = q.x();
    transform.rotation.y = q.y();
    transform.rotation.z = q.z();
    transform.rotation.w = q.w();

    return transform;
}

void SE3ToOdometry(const SE3d& T, const std::string& frame_id, const std::string& child_frame_id,
                   nav_msgs::msg::Odometry& odom) {
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = T.translation()(0);
    odom.pose.pose.position.y = T.translation()(1);
    odom.pose.pose.position.z = T.translation()(2);

    Eigen::Quaterniond q(T.rotation().matrix());
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
}

void setOdometryPoseCovariance(const Eigen::Matrix<double, 15, 15>& state_covariance,
                               nav_msgs::msg::Odometry& odom) {
    for (int row = 0; row < 6; ++row) {
        const int state_row = row < 3 ? row + 3 : row - 3;
        for (int col = 0; col < 6; ++col) {
            const int state_col = col < 3 ? col + 3 : col - 3;
            odom.pose.covariance[static_cast<std::size_t>(row * 6 + col)] =
                state_covariance(state_row, state_col);
        }
    }
}

void appendBoundedPose(nav_msgs::msg::Path& path, const geometry_msgs::msg::PoseStamped& pose,
                       std::size_t max_poses) {
    path.header.stamp = pose.header.stamp;
    if (max_poses == 0) {
        path.poses.clear();
        return;
    }

    path.poses.push_back(pose);
    if (path.poses.size() > max_poses) {
        const std::size_t excess = path.poses.size() - max_poses;
        path.poses.erase(path.poses.begin(),
                         path.poses.begin() + static_cast<std::ptrdiff_t>(excess));
    }
}

}  // namespace utils
}  // namespace fast_lio
