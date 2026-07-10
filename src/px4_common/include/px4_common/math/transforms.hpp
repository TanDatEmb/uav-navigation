#ifndef PX4_COMMON_MATH_TRANSFORMS_HPP_
#define PX4_COMMON_MATH_TRANSFORMS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace px4_common::math {

inline constexpr double kPi = 3.14159265358979323846;

/**
 * @brief Convert a 3D point from ENU (ROS default) to NED (PX4 default).
 * @param enu Point expressed in East-North-Up.
 * @return Point expressed in North-East-Down.
 */
inline Eigen::Vector3d EnuToNed(const Eigen::Vector3d &enu) noexcept {
    return Eigen::Vector3d(enu.y(), enu.x(), -enu.z());
}

/**
 * @brief Convert a 3D point from NED (PX4 default) to ENU (ROS default).
 * @param ned Point expressed in North-East-Down.
 * @return Point expressed in East-North-Up.
 */
inline Eigen::Vector3d NedToEnu(const Eigen::Vector3d &ned) noexcept {
    return Eigen::Vector3d(ned.y(), ned.x(), -ned.z());
}

/**
 * @brief Rotation matrix corresponding to ENU→NED frame rotation.
 *
 * Applying this matrix to a vector expressed in ENU yields the same vector
 * expressed in NED.
 * @return 3×3 orthonormal rotation matrix.
 */
inline Eigen::Matrix3d EnuToNedRotation() noexcept {
    // clang-format off
    return (Eigen::Matrix3d() <<
        0.0, 1.0,  0.0,
        1.0, 0.0,  0.0,
        0.0, 0.0, -1.0).finished();
    // clang-format on
}

/**
 * @brief Rotation matrix corresponding to NED→ENU frame rotation.
 * @return 3×3 orthonormal rotation matrix.
 */
inline Eigen::Matrix3d NedToEnuRotation() noexcept {
    return EnuToNedRotation().transpose();
}

/**
 * @brief Convert Euler angles (roll, pitch, yaw) to a quaternion.
 *
 * Angles are interpreted in the active frame of the calling convention.
 * The returned quaternion rotates from the local frame to the frame
 * described by the Euler angles.
 *
 * @param euler Euler angles in radians: [roll, pitch, yaw].
 * @return Unit quaternion representing the same orientation.
 */
inline Eigen::Quaterniond EulerToQuaternion(const Eigen::Vector3d &euler) {
    return Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX());
}

/**
 * @brief Convert Euler angles (roll, pitch, yaw) to a quaternion.
 * @param roll Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @param yaw Yaw angle in radians.
 * @return Unit quaternion representing the same orientation.
 */
inline Eigen::Quaterniond EulerToQuaternion(double roll, double pitch, double yaw) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

/**
 * @brief Convert a quaternion to Euler angles (roll, pitch, yaw).
 * @param q Unit quaternion.
 * @return Euler angles in radians: [roll, pitch, yaw].
 */
inline Eigen::Vector3d QuaternionToEuler(const Eigen::Quaterniond &q) {
    const Eigen::Matrix3d r = q.toRotationMatrix();
    Eigen::Vector3d euler;
    euler.x() = std::atan2(r(2, 1), r(2, 2));
    euler.y() = std::atan2(-r(2, 0), std::sqrt(r(2, 1) * r(2, 1) + r(2, 2) * r(2, 2)));
    euler.z() = std::atan2(r(1, 0), r(0, 0));
    return euler;
}

/**
 * @brief Convert a quaternion to Euler angles (roll, pitch, yaw).
 * @param q Unit quaternion.
 * @param[out] roll Roll angle in radians.
 * @param[out] pitch Pitch angle in radians.
 * @param[out] yaw Yaw angle in radians.
 */
inline void QuaternionToEuler(const Eigen::Quaterniond &q, double &roll, double &pitch,
                              double &yaw) {
    const Eigen::Vector3d euler = QuaternionToEuler(q);
    roll = euler.x();
    pitch = euler.y();
    yaw = euler.z();
}

/**
 * @brief Extract the yaw angle from a quaternion.
 *
 * Yaw is the rotation around the Z axis. The sign convention follows Eigen
 * eulerAngles: positive yaw follows the right-hand rule around the Z axis.
 *
 * @param q Unit quaternion.
 * @return Yaw angle in radians.
 */
inline double QuaternionGetYaw(const Eigen::Quaterniond &q) {
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/**
 * @brief Rotate a quaternion from ENU to NED frame representation.
 *
 * This is a passive rotation of the reference frame: the physical orientation
 * does not change, only its coordinate expression switches from ENU to NED.
 *
 * @param q_enu Orientation expressed with respect to the ENU frame.
 * @return The same physical orientation expressed with respect to the NED frame.
 */
inline Eigen::Quaterniond QuaternionEnuToNed(const Eigen::Quaterniond &q_enu) noexcept {
    static constexpr double kQuaternionRotationScalar = 0.0;
    static constexpr double kQuaternionRotationX = 0.7071067811865475;
    static constexpr double kQuaternionRotationY = 0.7071067811865475;
    static constexpr double kQuaternionRotationZ = 0.0;
    const Eigen::Quaterniond q_rotation(kQuaternionRotationScalar, kQuaternionRotationX,
                                        kQuaternionRotationY, kQuaternionRotationZ);
    return q_rotation * q_enu * q_rotation.conjugate();
}

/**
 * @brief Rotate a quaternion from NED to ENU frame representation.
 * @param q_ned Orientation expressed with respect to the NED frame.
 * @return The same physical orientation expressed with respect to the ENU frame.
 */
inline Eigen::Quaterniond QuaternionNedToEnu(const Eigen::Quaterniond &q_ned) noexcept {
    // ENU↔NED is a 180° rotation about the (1,1,0) axis, which is an involution:
    // R_enu_to_ned = R_ned_to_enu and q_rot is its own inverse. Therefore the
    // forward and inverse frame transforms are the same sandwich operation.
    return QuaternionEnuToNed(q_ned);
}

/**
 * @brief Rotation matrix corresponding to aircraft (FRD) → base_link (FLU).
 *
 * Forward-Right-Down and Forward-Left-Up share the same Forward axis. The
 * transform is a 180° rotation about the common X axis, which flips both
 * the Y and Z coordinates.
 *
 * @return 3×3 orthonormal rotation matrix.
 */
inline Eigen::Matrix3d AircraftToBaselinkRotation() noexcept {
    // clang-format off
    return (Eigen::Matrix3d() <<
        1.0,  0.0,  0.0,
        0.0, -1.0,  0.0,
        0.0,  0.0, -1.0).finished();
    // clang-format on
}

/**
 * @brief Rotation matrix corresponding to base_link (FLU) → aircraft (FRD).
 * @return 3×3 orthonormal rotation matrix.
 */
inline Eigen::Matrix3d BaselinkToAircraftRotation() noexcept {
    return AircraftToBaselinkRotation().transpose();
}

/**
 * @brief Convert a 3D point from aircraft (FRD) to base_link (FLU).
 * @param aircraft Point expressed in Forward-Right-Down.
 * @return Point expressed in Forward-Left-Up.
 */
inline Eigen::Vector3d AircraftToBaselink(const Eigen::Vector3d &aircraft) noexcept {
    return Eigen::Vector3d(aircraft.x(), -aircraft.y(), -aircraft.z());
}

/**
 * @brief Convert a 3D point from base_link (FLU) to aircraft (FRD).
 * @param baselink Point expressed in Forward-Left-Up.
 * @return Point expressed in Forward-Right-Down.
 */
inline Eigen::Vector3d BaselinkToAircraft(const Eigen::Vector3d &baselink) noexcept {
    return Eigen::Vector3d(baselink.x(), -baselink.y(), -baselink.z());
}

/**
 * @brief Rotate a quaternion from aircraft (FRD) to base_link (FLU) frame.
 *
 * This is a passive rotation of the reference frame. The physical orientation
 * does not change; only its coordinate expression switches from FRD to FLU.
 *
 * @param q_aircraft Orientation expressed with respect to the aircraft frame.
 * @return The same physical orientation expressed with respect to base_link.
 */
inline Eigen::Quaterniond QuaternionAircraftToBaselink(
    const Eigen::Quaterniond &q_aircraft) noexcept {
    const Eigen::Quaterniond q_rotation(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
    return q_rotation * q_aircraft * q_rotation.conjugate();
}

/**
 * @brief Rotate a quaternion from base_link (FLU) to aircraft (FRD) frame.
 * @param q_baselink Orientation expressed with respect to the base_link frame.
 * @return The same physical orientation expressed with respect to aircraft.
 */
inline Eigen::Quaterniond QuaternionBaselinkToAircraft(
    const Eigen::Quaterniond &q_baselink) noexcept {
    // The FRD↔FLU transform is a 180° rotation about X, which is an involution:
    // q_rot is its own inverse. Therefore the forward and inverse frame
    // transforms use the same sandwich operation.
    return QuaternionAircraftToBaselink(q_baselink);
}

/**
 * @brief Spherical linear interpolation between two quaternions.
 * @param q0 Start quaternion, must be unit length.
 * @param q1 End quaternion, must be unit length.
 * @param t Interpolation parameter in [0, 1].
 * @return Interpolated unit quaternion.
 */
inline Eigen::Quaterniond Slerp(const Eigen::Quaterniond &q0, const Eigen::Quaterniond &q1,
                                double t) {
    return q0.slerp(t, q1);
}

/**
 * @brief Convert degrees to radians.
 * @param deg Angle in degrees.
 * @return Angle in radians.
 */
inline constexpr double Deg2Rad(double deg) noexcept {
    return deg * (kPi / 180.0);
}

/**
 * @brief Convert radians to degrees.
 * @param rad Angle in radians.
 * @return Angle in degrees.
 */
inline constexpr double Rad2Deg(double rad) noexcept {
    return rad * (180.0 / kPi);
}

}  // namespace px4_common::math

#endif  // PX4_COMMON_MATH_TRANSFORMS_HPP_
