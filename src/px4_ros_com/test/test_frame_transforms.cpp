#include <px4_ros_com/frame_transforms.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>

namespace {

using px4_ros_com::frame_transforms::AircraftToBaselink;
using px4_ros_com::frame_transforms::BaselinkToAircraft;
using px4_ros_com::frame_transforms::EnuToNed;
using px4_ros_com::frame_transforms::NedToEnu;
using px4_ros_com::frame_transforms::Px4ToRosOrientation;
using px4_ros_com::frame_transforms::QuaternionAircraftToBaselink;
using px4_ros_com::frame_transforms::QuaternionBaselinkToAircraft;
using px4_ros_com::frame_transforms::QuaternionEnuToNed;
using px4_ros_com::frame_transforms::QuaternionNedToEnu;
using px4_ros_com::frame_transforms::RosToPx4Orientation;
using px4_ros_com::frame_transforms::utils::quaternion::ArrayToEigenQuat;
using px4_ros_com::frame_transforms::utils::quaternion::EigenQuatToArray;
using px4_ros_com::frame_transforms::utils::quaternion::QuaternionFromEuler;
using px4_ros_com::frame_transforms::utils::quaternion::QuaternionGetYaw;
using px4_ros_com::frame_transforms::utils::quaternion::QuaternionToEuler;
using px4_ros_com::frame_transforms::utils::types::ArrayUrtToCovarianceMatrix;
using px4_ros_com::frame_transforms::utils::types::CovarianceToArray;
using px4_ros_com::frame_transforms::utils::types::CovarianceUrtToArray;

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

TEST(FrameTransforms, QuaternionAircraftToBaselinkRoll) {
    // Roll around the common Forward (X) axis is preserved between FRD and FLU.
    const Eigen::Quaterniond q_aircraft =
        QuaternionFromEuler(px4_common::math::kPi / 2.0, 0.0, 0.0);
    const Eigen::Quaterniond q_baselink = QuaternionAircraftToBaselink(q_aircraft);
    const Eigen::Vector3d euler = QuaternionToEuler(q_baselink);
    EXPECT_NEAR(euler.x(), px4_common::math::kPi / 2.0, 1e-12);
    EXPECT_NEAR(euler.y(), 0.0, 1e-12);
    EXPECT_NEAR(euler.z(), 0.0, 1e-12);
}

TEST(FrameTransforms, QuaternionAircraftToBaselinkPitch) {
    // Pitch in FRD (Forward -> Down) becomes negative pitch in FLU (Forward -> Up).
    const Eigen::Quaterniond q_aircraft =
        QuaternionFromEuler(0.0, px4_common::math::kPi / 4.0, 0.0);
    const Eigen::Quaterniond q_baselink = QuaternionAircraftToBaselink(q_aircraft);
    const Eigen::Vector3d euler = QuaternionToEuler(q_baselink);
    EXPECT_NEAR(euler.x(), 0.0, 1e-12);
    EXPECT_NEAR(euler.y(), -px4_common::math::kPi / 4.0, 1e-12);
    EXPECT_NEAR(euler.z(), 0.0, 1e-12);
}

TEST(FrameTransforms, AircraftToBaselinkVectorYFlip) {
    // FRD right vector (0,1,0) maps to FLU left vector (0,-1,0).
    const Eigen::Vector3d v_aircraft(0.0, 1.0, 0.0);
    const Eigen::Vector3d v_baselink = AircraftToBaselink(v_aircraft);
    EXPECT_NEAR(v_baselink.x(), 0.0, 1e-12);
    EXPECT_NEAR(v_baselink.y(), -1.0, 1e-12);
    EXPECT_NEAR(v_baselink.z(), 0.0, 1e-12);
}

TEST(FrameTransforms, AircraftBaselinkRoundTrip) {
    const Eigen::Quaterniond q_in = QuaternionFromEuler(0.1, 0.2, 0.3);
    const Eigen::Quaterniond q_out =
        QuaternionBaselinkToAircraft(QuaternionAircraftToBaselink(q_in));
    EXPECT_NEAR(std::abs(q_in.dot(q_out)), 1.0, 1e-12);
}

TEST(FrameTransforms, Px4ToRosToPx4RoundTrip) {
    // Orientation of aircraft w.r.t. NED, expressed as PX4 quaternion.
    const Eigen::Quaterniond q_px4 = QuaternionFromEuler(0.1, -0.2, 0.7);
    const Eigen::Quaterniond q_ros = Px4ToRosOrientation(q_px4);
    const Eigen::Quaterniond q_recovered = RosToPx4Orientation(q_ros);
    EXPECT_NEAR(std::abs(q_px4.dot(q_recovered)), 1.0, 1e-12);
}

TEST(FrameTransforms, ArrayToEigenQuatUsesWxyz) {
    // Eigen constructor is (w,x,y,z). Ensure ArrayToEigenQuat treats input as [w,x,y,z].
    const std::array<float, 4> wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
    const Eigen::Quaterniond q = ArrayToEigenQuat(wxyz);
    EXPECT_DOUBLE_EQ(q.w(), 1.0);
    EXPECT_DOUBLE_EQ(q.x(), 0.0);
    EXPECT_DOUBLE_EQ(q.y(), 0.0);
    EXPECT_DOUBLE_EQ(q.z(), 0.0);
}

TEST(FrameTransforms, EigenQuatToArrayUsesWxyz) {
    const Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
    std::array<float, 4> arr{};
    EigenQuatToArray(q, arr);
    EXPECT_FLOAT_EQ(arr[0], 1.0f);
    EXPECT_FLOAT_EQ(arr[1], 0.0f);
    EXPECT_FLOAT_EQ(arr[2], 0.0f);
    EXPECT_FLOAT_EQ(arr[3], 0.0f);
}

TEST(FrameTransforms, Covariance3x3ToArrayRoundTrip) {
    Eigen::Matrix3d cov_in;
    cov_in << 1.0, 0.2, 0.3, 0.2, 2.0, 0.4, 0.3, 0.4, 3.0;
    std::array<float, 9> arr{};
    CovarianceToArray(cov_in, arr);
    for (std::size_t i = 0; i < 9; ++i) {
        EXPECT_FLOAT_EQ(arr[i], static_cast<float>(cov_in.data()[i]));
    }
}

TEST(FrameTransforms, CovarianceUrtRoundTrip) {
    Eigen::Matrix3d cov_in;
    cov_in << 1.0, 0.2, 0.3, 0.2, 2.0, 0.4, 0.3, 0.4, 3.0;
    std::array<float, 6> arr{};
    CovarianceUrtToArray(cov_in, arr);
    Eigen::Matrix3d cov_out;
    ArrayUrtToCovarianceMatrix(arr, cov_out);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(cov_in(i, j), cov_out(i, j), 1e-6);
        }
    }
}

}  // namespace
