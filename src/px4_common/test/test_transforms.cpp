#include <gtest/gtest.h>

#include <px4_common/math/transforms.hpp>

using px4_common::math::Deg2Rad;
using px4_common::math::EnuToNed;
using px4_common::math::EnuToNedRotation;
using px4_common::math::EulerToQuaternion;
using px4_common::math::NedToEnu;
using px4_common::math::NedToEnuRotation;
using px4_common::math::QuaternionEnuToNed;
using px4_common::math::QuaternionGetYaw;
using px4_common::math::QuaternionNedToEnu;
using px4_common::math::QuaternionToEuler;
using px4_common::math::Rad2Deg;
using px4_common::math::Slerp;

TEST(TransformsTest, EnuToNedAndBack) {
    const Eigen::Vector3d enu(1.0, 2.0, 3.0);
    const Eigen::Vector3d ned = EnuToNed(enu);
    const Eigen::Vector3d recovered = NedToEnu(ned);

    EXPECT_DOUBLE_EQ(ned.x(), enu.y());
    EXPECT_DOUBLE_EQ(ned.y(), enu.x());
    EXPECT_DOUBLE_EQ(ned.z(), -enu.z());
    EXPECT_TRUE(recovered.isApprox(enu));
}

TEST(TransformsTest, RotationMatricesAreConsistent) {
    const Eigen::Matrix3d r_enu_to_ned = EnuToNedRotation();
    const Eigen::Matrix3d r_ned_to_enu = NedToEnuRotation();

    const Eigen::Matrix3d identity_from_product = r_enu_to_ned * r_ned_to_enu;
    EXPECT_TRUE(identity_from_product.isApprox(Eigen::Matrix3d::Identity()));

    const Eigen::Matrix3d identity_from_square = r_enu_to_ned * r_enu_to_ned;
    EXPECT_TRUE(identity_from_square.isApprox(Eigen::Matrix3d::Identity()));
}

TEST(TransformsTest, EulerQuaternionRoundTrip) {
    const Eigen::Vector3d euler(Deg2Rad(10.0), Deg2Rad(20.0), Deg2Rad(30.0));
    const Eigen::Quaterniond q = EulerToQuaternion(euler);
    const Eigen::Vector3d recovered = QuaternionToEuler(q);

    EXPECT_NEAR(euler.x(), recovered.x(), 1e-6);
    EXPECT_NEAR(euler.y(), recovered.y(), 1e-6);
    EXPECT_NEAR(euler.z(), recovered.z(), 1e-6);
}

TEST(TransformsTest, QuaternionGetYawZOnly) {
    const double yaw_rad = Deg2Rad(45.0);
    const Eigen::Quaterniond q = EulerToQuaternion(0.0, 0.0, yaw_rad);
    EXPECT_NEAR(QuaternionGetYaw(q), yaw_rad, 1e-9);
}

TEST(TransformsTest, QuaternionEnuToNedIsInvolutory) {
    const Eigen::Quaterniond q_identity = Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond q_ned = QuaternionEnuToNed(q_identity);
    const Eigen::Quaterniond q_recovered = QuaternionNedToEnu(q_ned);

    EXPECT_TRUE(q_recovered.isApprox(q_identity));
}

TEST(TransformsTest, SlerpEndpointsMatch) {
    const Eigen::Quaterniond q0 = EulerToQuaternion(0.0, 0.0, 0.0);
    const Eigen::Quaterniond q1 = EulerToQuaternion(0.0, 0.0, Deg2Rad(90.0));

    const Eigen::Quaterniond q_t0 = Slerp(q0, q1, 0.0);
    const Eigen::Quaterniond q_t1 = Slerp(q0, q1, 1.0);

    EXPECT_TRUE(q_t0.isApprox(q0) || q_t0.coeffs().isApprox(-q0.coeffs()));
    EXPECT_TRUE(q_t1.isApprox(q1) || q_t1.coeffs().isApprox(-q1.coeffs()));
}

TEST(TransformsTest, Deg2RadAndRad2DegAreInverses) {
    constexpr double kTestAngleDeg = 90.0;
    EXPECT_NEAR(Rad2Deg(Deg2Rad(kTestAngleDeg)), kTestAngleDeg, 1e-9);
}

TEST(TransformsTest, QuaternionNedToEnuRoundTripWithNonTrivialOrientation) {
    const Eigen::Vector3d euler_enu(Deg2Rad(15.0), Deg2Rad(-25.0), Deg2Rad(45.0));
    const Eigen::Quaterniond q_enu = EulerToQuaternion(euler_enu);
    const Eigen::Quaterniond q_ned = QuaternionEnuToNed(q_enu);
    const Eigen::Quaterniond q_recovered = QuaternionNedToEnu(q_ned);

    EXPECT_TRUE(q_recovered.isApprox(q_enu) || q_recovered.coeffs().isApprox(-q_enu.coeffs()));
}

TEST(TransformsTest, EulerToQuaternionOverload) {
    const Eigen::Quaterniond q0 = EulerToQuaternion(Deg2Rad(10.0), Deg2Rad(20.0), Deg2Rad(30.0));
    const Eigen::Quaterniond q1 =
        EulerToQuaternion(Eigen::Vector3d(Deg2Rad(10.0), Deg2Rad(20.0), Deg2Rad(30.0)));
    EXPECT_TRUE(q0.isApprox(q1) || q0.coeffs().isApprox(-q1.coeffs()));
}

TEST(TransformsTest, QuaternionToEulerOutputOverload) {
    const Eigen::Quaterniond q = EulerToQuaternion(0.1, 0.2, 0.3);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    QuaternionToEuler(q, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.1, 1e-9);
    EXPECT_NEAR(pitch, 0.2, 1e-9);
    EXPECT_NEAR(yaw, 0.3, 1e-9);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
