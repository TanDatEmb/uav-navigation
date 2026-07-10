/****************************************************************************
 *
 * Copyright 2026 CTUAV. All rights reserved.
 *
 * PX4 uORB <-> ROS 2 frame transform helpers.
 *
 * This file re-implements the subset of PX4/MAVROS frame transforms required
 * for the uav-navigation stack. It delegates the core ENU/NED math to
 * px4_common::math and only adds ROS 2 message/covariance convenience helpers.
 *
 * Coordinate conventions:
 *   NED  : North-East-Down (PX4 default).
 *   ENU  : East-North-Up (ROS default).
 *   body : Forward-Right-Down (aircraft / PX4 body frame).
 *   base_link : Forward-Left-Up (ROS REP-103 body frame).
 ****************************************************************************/

#ifndef PX4_ROS_COM_FRAME_TRANSFORMS_HPP_
#define PX4_ROS_COM_FRAME_TRANSFORMS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cstddef>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <px4_common/math/transforms.hpp>

namespace px4_ros_com::frame_transforms {

//! Type matching rosmsg for 3x3 covariance matrix.
using Covariance3d = sensor_msgs::msg::Imu::_angular_velocity_covariance_type;

//! Type matching rosmsg for 6x6 covariance matrix.
using Covariance6d = geometry_msgs::msg::PoseWithCovariance::_covariance_type;

//! Type matching rosmsg for 9x9 covariance matrix.
using Covariance9d = std::array<double, 81>;

//! Eigen::Map for Covariance3d.
using EigenMapCovariance3d = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>;
using EigenMapConstCovariance3d = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>;

//! Eigen::Map for Covariance6d.
using EigenMapCovariance6d = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>;
using EigenMapConstCovariance6d = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>;

//! Eigen::Map for Covariance9d.
using EigenMapCovariance9d = Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor>>;
using EigenMapConstCovariance9d = Eigen::Map<const Eigen::Matrix<double, 9, 9, Eigen::RowMajor>>;

/**
 * @brief Orientation transform options when applying rotations to data.
 */
enum class StaticTf {
    kNedToEnu,            //!< Change from expressed WRT NED frame to WRT ENU frame.
    kEnuToNed,            //!< Change from expressed WRT ENU frame to WRT NED frame.
    kAircraftToBaselink,  //!< Change from expressed WRT aircraft frame to WRT baselink frame.
    kBaselinkToAircraft   //!< Change from expressed WRT baselink to WRT aircraft.
};

namespace utils {

namespace quaternion {

/**
 * @brief Convert euler angles to quaternion.
 * @param euler Euler angles [roll, pitch, yaw] in radians.
 * @return Unit quaternion.
 */
inline Eigen::Quaterniond QuaternionFromEuler(const Eigen::Vector3d& euler) {
    return px4_common::math::EulerToQuaternion(euler);
}

/**
 * @brief Convert euler angles to quaternion.
 * @param roll Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @param yaw Yaw angle in radians.
 * @return Unit quaternion.
 */
inline Eigen::Quaterniond QuaternionFromEuler(double roll, double pitch, double yaw) {
    return px4_common::math::EulerToQuaternion(roll, pitch, yaw);
}

/**
 * @brief Convert quaternion to euler angles.
 * @param q Unit quaternion.
 * @return Euler angles [roll, pitch, yaw] in radians.
 */
inline Eigen::Vector3d QuaternionToEuler(const Eigen::Quaterniond& q) {
    return px4_common::math::QuaternionToEuler(q);
}

/**
 * @brief Convert quaternion to euler angles.
 * @param q Unit quaternion.
 * @param[out] roll Roll angle in radians.
 * @param[out] pitch Pitch angle in radians.
 * @param[out] yaw Yaw angle in radians.
 */
inline void QuaternionToEuler(const Eigen::Quaterniond& q, double& roll, double& pitch,
                              double& yaw) {
    px4_common::math::QuaternionToEuler(q, roll, pitch, yaw);
}

/**
 * @brief Store Eigen quaternion to float[4] in PX4 wxyz order.
 * @param q Eigen quaternion in xyzw internal order.
 * @param[out] qarray PX4 quaternion in wxyz order.
 */
inline void EigenQuatToArray(const Eigen::Quaterniond& q, std::array<float, 4>& qarray) {
    qarray[0] = static_cast<float>(q.w());
    qarray[1] = static_cast<float>(q.x());
    qarray[2] = static_cast<float>(q.y());
    qarray[3] = static_cast<float>(q.z());
}

/**
 * @brief Convert float[4] quaternion (PX4 wxyz order) to Eigen quaternion.
 * @param qarray PX4 quaternion in wxyz order.
 * @return Eigen quaternion in xyzw internal order.
 */
inline Eigen::Quaterniond ArrayToEigenQuat(const std::array<float, 4>& qarray) {
    return Eigen::Quaterniond(static_cast<double>(qarray[0]), static_cast<double>(qarray[1]),
                              static_cast<double>(qarray[2]), static_cast<double>(qarray[3]));
}

/**
 * @brief Get yaw angle from quaternion.
 * @param q Unit quaternion.
 * @return Yaw angle in radians.
 */
inline double QuaternionGetYaw(const Eigen::Quaterniond& q) {
    return px4_common::math::QuaternionGetYaw(q);
}

}  // namespace quaternion

namespace types {

/**
 * @brief Convert a covariance matrix to float[n] array.
 * @tparam T Eigen matrix type.
 * @tparam SIZE Array size.
 * @param cov Covariance matrix.
 * @param[out] covmsg Output array.
 */
template <class T, std::size_t SIZE>
void CovarianceToArray(const T& cov, std::array<float, SIZE>& covmsg) {
    static_assert(T::RowsAtCompileTime == 3 && T::ColsAtCompileTime == 3,
                  "CovarianceToArray requires a 3x3 matrix");
    for (std::size_t i = 0; i < 9; ++i) {
        covmsg[i] = static_cast<float>(cov.data()[i]);
    }
}

/**
 * @brief Convert upper right triangular of a covariance matrix to float[n] array.
 * @tparam T Eigen matrix type.
 * @tparam ARR_SIZE Array size.
 * @param covmap Covariance map.
 * @param[out] covmsg Output array.
 */
template <class T, std::size_t ARR_SIZE>
void CovarianceUrtToArray(const T& covmap, std::array<float, ARR_SIZE>& covmsg) {
    static_assert(T::RowsAtCompileTime == 3 && T::ColsAtCompileTime == 3,
                  "CovarianceUrtToArray requires a 3x3 matrix");
    covmsg[0] = static_cast<float>(covmap(0, 0));
    covmsg[1] = static_cast<float>(covmap(0, 1));
    covmsg[2] = static_cast<float>(covmap(0, 2));
    covmsg[3] = static_cast<float>(covmap(1, 1));
    covmsg[4] = static_cast<float>(covmap(1, 2));
    covmsg[5] = static_cast<float>(covmap(2, 2));
}

/**
 * @brief Convert float[n] array (upper right triangular of covariance) to full matrix.
 * @tparam T Eigen matrix type.
 * @tparam ARR_SIZE Array size.
 * @param covmsg Input array.
 * @param[out] covmat Output covariance matrix.
 */
template <class T, std::size_t ARR_SIZE>
void ArrayUrtToCovarianceMatrix(const std::array<float, ARR_SIZE>& covmsg, T& covmat) {
    static_assert(T::RowsAtCompileTime == 3 && T::ColsAtCompileTime == 3,
                  "ArrayUrtToCovarianceMatrix requires a 3x3 matrix");
    covmat.setZero();
    covmat(0, 0) = static_cast<double>(covmsg[0]);
    covmat(0, 1) = static_cast<double>(covmsg[1]);
    covmat(0, 2) = static_cast<double>(covmsg[2]);
    covmat(1, 0) = static_cast<double>(covmsg[1]);
    covmat(1, 1) = static_cast<double>(covmsg[3]);
    covmat(1, 2) = static_cast<double>(covmsg[4]);
    covmat(2, 0) = static_cast<double>(covmsg[2]);
    covmat(2, 1) = static_cast<double>(covmsg[4]);
    covmat(2, 2) = static_cast<double>(covmsg[5]);
}

}  // namespace types
}  // namespace utils

/**
 * @brief Static quaternion for NED <-> ENU frame transform.
 *
 * NED to ENU: +PI/2 rotation about Z (Down) followed by +PI rotation around X.
 */
inline Eigen::Quaterniond NedEnuQuaternion() {
    static constexpr double kScalar = 0.0;
    static constexpr double kX = 0.7071067811865475;
    static constexpr double kY = 0.7071067811865475;
    static constexpr double kZ = 0.0;
    return Eigen::Quaterniond(kScalar, kX, kY, kZ).normalized();
}

/**
 * @brief Static quaternion for aircraft (FRD) <-> base_link (FLU) transform.
 * +PI rotation around X transforms from FRD to FLU.
 *
 * Delegates to px4_common::math to keep the core math in one place.
 */
inline Eigen::Quaterniond AircraftBaselinkQuaternion() {
    return px4_common::math::QuaternionAircraftToBaselink(Eigen::Quaterniond::Identity());
}

/**
 * @brief Convert a 3D point from ENU to NED.
 */
inline Eigen::Vector3d EnuToNed(const Eigen::Vector3d& enu) {
    return px4_common::math::EnuToNed(enu);
}

/**
 * @brief Convert a 3D point from NED to ENU.
 */
inline Eigen::Vector3d NedToEnu(const Eigen::Vector3d& ned) {
    return px4_common::math::NedToEnu(ned);
}

/**
 * @brief Convert a quaternion from ENU frame representation to NED.
 */
inline Eigen::Quaterniond QuaternionEnuToNed(const Eigen::Quaterniond& q_enu) {
    return px4_common::math::QuaternionEnuToNed(q_enu);
}

/**
 * @brief Convert a quaternion from NED frame representation to ENU.
 */
inline Eigen::Quaterniond QuaternionNedToEnu(const Eigen::Quaterniond& q_ned) {
    return px4_common::math::QuaternionNedToEnu(q_ned);
}

/**
 * @brief Convert geometry_msgs Vector3 from ENU to NED.
 */
inline geometry_msgs::msg::Vector3 EnuToNed(const geometry_msgs::msg::Vector3& enu) {
    const Eigen::Vector3d ned = EnuToNed(Eigen::Vector3d(enu.x, enu.y, enu.z));
    geometry_msgs::msg::Vector3 msg;
    msg.x = ned.x();
    msg.y = ned.y();
    msg.z = ned.z();
    return msg;
}

/**
 * @brief Convert geometry_msgs Vector3 from NED to ENU.
 */
inline geometry_msgs::msg::Vector3 NedToEnu(const geometry_msgs::msg::Vector3& ned) {
    const Eigen::Vector3d enu = NedToEnu(Eigen::Vector3d(ned.x, ned.y, ned.z));
    geometry_msgs::msg::Vector3 msg;
    msg.x = enu.x();
    msg.y = enu.y();
    msg.z = enu.z();
    return msg;
}

/**
 * @brief Convert a quaternion from aircraft (FRD) to base_link (FLU).
 *
 * Passive frame rotation: the physical orientation is unchanged, only its
 * coordinate expression switches from the PX4 aircraft frame to the ROS
 * base_link frame.
 *
 * Delegates to px4_common::math to keep the core math in one place.
 */
inline Eigen::Quaterniond QuaternionAircraftToBaselink(const Eigen::Quaterniond& q_aircraft) {
    return px4_common::math::QuaternionAircraftToBaselink(q_aircraft);
}

/**
 * @brief Convert a quaternion from base_link (FLU) to aircraft (FRD).
 */
inline Eigen::Quaterniond QuaternionBaselinkToAircraft(const Eigen::Quaterniond& q_baselink) {
    return px4_common::math::QuaternionBaselinkToAircraft(q_baselink);
}

/**
 * @brief Convert a 3D vector from aircraft (FRD) to base_link (FLU).
 */
inline Eigen::Vector3d AircraftToBaselink(const Eigen::Vector3d& v_aircraft) {
    return px4_common::math::AircraftToBaselink(v_aircraft);
}

/**
 * @brief Convert a 3D vector from base_link (FLU) to aircraft (FRD).
 */
inline Eigen::Vector3d BaselinkToAircraft(const Eigen::Vector3d& v_baselink) {
    return px4_common::math::BaselinkToAircraft(v_baselink);
}

/**
 * @brief Convert orientation from PX4 convention to ROS convention.
 *
 * PX4 convention: orientation of aircraft (FRD) frame w.r.t. NED.
 * ROS convention: orientation of base_link (FLU) frame w.r.t. ENU.
 *
 * Composition: aircraft→NED → NED→ENU → base_link→aircraft (inverse).
 */
inline Eigen::Quaterniond Px4ToRosOrientation(const Eigen::Quaterniond& q_px4) {
    return QuaternionBaselinkToAircraft(QuaternionNedToEnu(q_px4));
}

/**
 * @brief Convert orientation from ROS convention to PX4 convention.
 *
 * ROS convention: orientation of base_link (FLU) frame w.r.t. ENU.
 * PX4 convention: orientation of aircraft (FRD) frame w.r.t. NED.
 */
inline Eigen::Quaterniond RosToPx4Orientation(const Eigen::Quaterniond& q_ros) {
    return QuaternionEnuToNed(QuaternionAircraftToBaselink(q_ros));
}

}  // namespace px4_ros_com::frame_transforms

#endif  // PX4_ROS_COM_FRAME_TRANSFORMS_HPP_
