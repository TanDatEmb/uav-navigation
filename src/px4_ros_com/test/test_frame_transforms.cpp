#include <px4_ros_com/frame_transforms.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>

namespace {

using px4_ros_com::frame_transforms::EnuToNed;
using px4_ros_com::frame_transforms::NedToEnu;
using px4_ros_com::frame_transforms::QuaternionEnuToNed;
using px4_ros_com::frame_transforms::QuaternionNedToEnu;
using px4_ros_com::frame_transforms::utils::quaternion::ArrayToEigenQuat;
using px4_ros_com::frame_transforms::utils::quaternion::EigenQuatToArray;
using px4_ros_com::frame_transforms::utils::quaternion::QuaternionFromEuler;
using px4_ros_com::frame_transforms::utils::quaternion::QuaternionGetYaw;
using px4_ros_com::frame_transforms::utils::quaternion::QuaternionToEuler;

TEST(FrameTransforms, EnuToNedPoint) {
    const Eigen::Vector3d enu(1.0, 2.0, 3.0);
    const Eigen::Vector3d ned = EnuToNed(enu);
    EXPECT_DOUBLE_EQ(ned.x(), enu.y());   // North = East
    EXPECT_DOUBLE_EQ(ned.y(), enu.x());   // East = North
    EXPECT_DOUBLE_EQ(ned.z(), -enu.z());  // Down = -Up
}

TEST(FrameTransforms, NedToEnuRoundTrip) {
    const Eigen::Vector3d ned(4.0, -1.0, 2.5);
    const Eigen::Vector3d enu = NedToEnu(ned);
    const Eigen::Vector3d recovered = EnuToNed(enu);
    EXPECT_NEAR(recovered.x(), ned.x(), 1e-12);
    EXPECT_NEAR(recovered.y(), ned.y(), 1e-12);
    EXPECT_NEAR(recovered.z(), ned.z(), 1e-12);
}

TEST(FrameTransforms, QuaternionEnuToNedRoundTrip) {
    const Eigen::Quaterniond q_enu = QuaternionFromEuler(0.1, 0.2, 0.3);
    const Eigen::Quaterniond q_ned = QuaternionEnuToNed(q_enu);
    const Eigen::Quaterniond recovered = QuaternionNedToEnu(q_ned);
    EXPECT_NEAR(recovered.w(), q_enu.w(), 1e-12);
    EXPECT_NEAR(recovered.x(), q_enu.x(), 1e-12);
    EXPECT_NEAR(recovered.y(), q_enu.y(), 1e-12);
    EXPECT_NEAR(recovered.z(), q_enu.z(), 1e-12);
}

TEST(FrameTransforms, QuaternionGetYawPureYaw) {
    const Eigen::Quaterniond q = QuaternionFromEuler(0.0, 0.0, px4_common::math::kPi / 4.0);
    EXPECT_NEAR(QuaternionGetYaw(q), px4_common::math::kPi / 4.0, 1e-12);
}

TEST(FrameTransforms, QuaternionToEulerRoundTrip) {
    const Eigen::Vector3d euler_in(0.1, 0.2, 0.3);
    const Eigen::Quaterniond q = QuaternionFromEuler(euler_in);
    const Eigen::Vector3d euler_out = QuaternionToEuler(q);
    EXPECT_NEAR(euler_out.x(), euler_in.x(), 1e-12);
    EXPECT_NEAR(euler_out.y(), euler_in.y(), 1e-12);
    EXPECT_NEAR(euler_out.z(), euler_in.z(), 1e-12);
}

TEST(FrameTransforms, EigenQuatToArrayAndBack) {
    const Eigen::Quaterniond q_in(0.7071067811865476, 0.0, 0.7071067811865475, 0.0);
    std::array<float, 4> arr{};
    EigenQuatToArray(q_in, arr);
    const Eigen::Quaterniond q_out = ArrayToEigenQuat(arr);
    EXPECT_NEAR(q_out.w(), q_in.w(), 1e-6);
    EXPECT_NEAR(q_out.x(), q_in.x(), 1e-6);
    EXPECT_NEAR(q_out.y(), q_in.y(), 1e-6);
    EXPECT_NEAR(q_out.z(), q_in.z(), 1e-6);
}

TEST(FrameTransforms, NedEnuQuaternionIsUnitLength) {
    const Eigen::Quaterniond q = px4_ros_com::frame_transforms::NedEnuQuaternion();
    EXPECT_NEAR(q.norm(), 1.0, 1e-12);
}

}  // namespace
