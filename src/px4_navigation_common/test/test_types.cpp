#include <gtest/gtest.h>

#include <px4_navigation_common/types.hpp>

using px4_navigation_common::DroneStateNed;
using px4_navigation_common::MissionPathNed;
using px4_navigation_common::PointLivox;
using px4_navigation_common::VoxelHash;
using px4_navigation_common::WaypointNed;

TEST(TypesTest, PointLivoxDefaultConstructed) {
    const PointLivox pt;
    EXPECT_DOUBLE_EQ(pt.x, 0.0);
    EXPECT_DOUBLE_EQ(pt.y, 0.0);
    EXPECT_DOUBLE_EQ(pt.z, 0.0);
    EXPECT_FLOAT_EQ(pt.intensity, 0.0f);
}

TEST(TypesTest, VoxelHashIsDeterministic) {
    const VoxelHash hasher;
    const Eigen::Vector3i idx(1, 2, 3);
    EXPECT_EQ(hasher(idx), hasher(idx));
}

TEST(TypesTest, DroneStateNedDefaultsToInvalid) {
    const DroneStateNed state;
    EXPECT_FALSE(state.valid);
    EXPECT_DOUBLE_EQ(state.position.norm(), 0.0);
    EXPECT_DOUBLE_EQ(state.yaw, 0.0);
}

TEST(TypesTest, WaypointNedDefaultsToInvalid) {
    const WaypointNed wp;
    EXPECT_FALSE(wp.valid);
    EXPECT_DOUBLE_EQ(wp.position.norm(), 0.0);
}

TEST(MissionPathNedTest, GetDirectionReturnsZeroWhenInvalid) {
    const MissionPathNed path;
    double dx = 1.0;
    double dy = 1.0;
    path.GetDirection(dx, dy);
    EXPECT_DOUBLE_EQ(dx, 0.0);
    EXPECT_DOUBLE_EQ(dy, 0.0);
}

TEST(MissionPathNedTest, GetDirectionFromPreviousToCurrent) {
    MissionPathNed path;
    path.has_valid_path = true;
    path.previous.valid = true;
    path.previous.position = Eigen::Vector3d(0.0, 0.0, 0.0);
    path.current.valid = true;
    path.current.position = Eigen::Vector3d(3.0, 4.0, 0.0);

    double dx = 0.0;
    double dy = 0.0;
    path.GetDirection(dx, dy);
    EXPECT_DOUBLE_EQ(dx, 0.6);
    EXPECT_DOUBLE_EQ(dy, 0.8);
}

TEST(MissionPathNedTest, CrossTrackErrorOnStraightLine) {
    MissionPathNed path;
    path.has_valid_path = true;
    path.previous.valid = true;
    path.previous.position = Eigen::Vector3d(0.0, 0.0, 0.0);
    path.current.valid = true;
    path.current.position = Eigen::Vector3d(10.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(path.CrossTrackError(5.0, 1.0), 1.0);
}

TEST(MissionPathNedTest, CrossTrackErrorClampsProjectionToStart) {
    MissionPathNed path;
    path.has_valid_path = true;
    path.previous.valid = true;
    path.previous.position = Eigen::Vector3d(0.0, 0.0, 0.0);
    path.current.valid = true;
    path.current.position = Eigen::Vector3d(10.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(path.CrossTrackError(-3.0, 4.0), 5.0);
}
