#ifndef PX4_COMMON_MATH_TRANSFORMS_HPP_
#define PX4_COMMON_MATH_TRANSFORMS_HPP_

#include <Eigen/Dense>

namespace px4_common::math {

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

}  // namespace px4_common::math

#endif  // PX4_COMMON_MATH_TRANSFORMS_HPP_
