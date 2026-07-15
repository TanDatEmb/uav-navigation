#include <gtest/gtest.h>

#include <px4_nav_common/math/grid.hpp>

using px4_nav_common::math::AddressToIndex;
using px4_nav_common::math::IndexToAddress;
using px4_nav_common::math::IndexToWorld;
using px4_nav_common::math::IsIndexInBounds;
using px4_nav_common::math::IsPositionInMap;
using px4_nav_common::math::WorldToIndex;

TEST(GridTest, WorldToIndexAndBack) {
    const Eigen::Vector3d origin(0.0, 0.0, 0.0);
    constexpr double kResolution = 0.2;
    const Eigen::Vector3d position(0.3, 0.5, 0.7);

    const Eigen::Vector3i idx = WorldToIndex(position, origin, kResolution);
    const Eigen::Vector3d centre = IndexToWorld(idx, origin, kResolution);

    EXPECT_EQ(idx.x(), 1);
    EXPECT_EQ(idx.y(), 2);
    EXPECT_EQ(idx.z(), 3);
    EXPECT_NEAR(centre.x(), 0.3, 1e-9);
    EXPECT_NEAR(centre.y(), 0.5, 1e-9);
    EXPECT_NEAR(centre.z(), 0.7, 1e-9);
}

TEST(GridTest, IndexInBounds) {
    const Eigen::Vector3i size(10, 10, 5);
    EXPECT_TRUE(IsIndexInBounds(Eigen::Vector3i(0, 0, 0), size));
    EXPECT_TRUE(IsIndexInBounds(Eigen::Vector3i(9, 9, 4), size));
    EXPECT_FALSE(IsIndexInBounds(Eigen::Vector3i(10, 0, 0), size));
    EXPECT_FALSE(IsIndexInBounds(Eigen::Vector3i(0, -1, 0), size));
}

TEST(GridTest, AddressRoundTrip) {
    const Eigen::Vector3i size(4, 5, 6);
    const Eigen::Vector3i index(1, 2, 3);
    const int address = IndexToAddress(index, size);
    const Eigen::Vector3i recovered = AddressToIndex(address, size);
    EXPECT_EQ(index, recovered);
}

TEST(GridTest, PositionInMap) {
    const Eigen::Vector3d origin(0.0, 0.0, 0.0);
    const Eigen::Vector3i size(10, 10, 5);
    constexpr double kResolution = 1.0;

    EXPECT_TRUE(IsPositionInMap(Eigen::Vector3d(5.0, 5.0, 2.0), origin, size, kResolution));
    EXPECT_FALSE(IsPositionInMap(Eigen::Vector3d(10.5, 5.0, 2.0), origin, size, kResolution));
}
