#include <gtest/gtest.h>

#include <px4_common/math/transforms.hpp>

using px4_common::math::EnuToNed;
using px4_common::math::EnuToNedRotation;
using px4_common::math::NedToEnu;
using px4_common::math::NedToEnuRotation;

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

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
