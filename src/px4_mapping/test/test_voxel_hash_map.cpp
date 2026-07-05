#include <gtest/gtest.h>

#include <Eigen/Core>

#include <px4_common/mapping/voxel_types.hpp>
#include <px4_common/types.hpp>
#include <px4_mapping/voxel_hash_map.hpp>

using px4_common::PointLivox;
using px4_mapping::VoxelHashMap;

class VoxelHashMapTest : public ::testing::Test {
   protected:
    void SetUp() override {
        map_.SetExtrinsicTranslation(Eigen::Vector3d::Zero());
    }

    VoxelHashMap map_;
};

TEST_F(VoxelHashMapTest, InitiallyEmptyAndNotReady) {
    EXPECT_EQ(map_.Size(), 0U);
    EXPECT_EQ(map_.OccupiedCount(), 0U);
    EXPECT_FALSE(map_.IsReady());
}

TEST_F(VoxelHashMapTest, UpdateCreatesOccupiedVoxel) {
    std::vector<PointLivox> points;
    PointLivox point;
    point.x = 1.0;
    point.y = 0.0;
    point.z = 0.0;
    point.intensity = 100.0f;
    points.push_back(point);

    map_.Update(points, Eigen::Vector3d::Zero());

    EXPECT_TRUE(map_.IsReady());
    EXPECT_EQ(map_.Size(), 1U);
    EXPECT_EQ(map_.OccupiedCount(), 1U);
    EXPECT_EQ(map_.PointsProcessed(), 1U);
}

TEST_F(VoxelHashMapTest, GetOccupiedPointsInRadiusReturnsExpectedPoint) {
    std::vector<PointLivox> points;
    PointLivox point;
    point.x = 2.0;
    point.y = 0.0;
    point.z = 0.0;
    point.intensity = 100.0f;
    points.push_back(point);

    map_.Update(points, Eigen::Vector3d::Zero());

    std::vector<Eigen::Vector3d> occupied;
    map_.GetOccupiedPointsInRadius(Eigen::Vector3d::Zero(), 5.0, occupied);

    ASSERT_EQ(occupied.size(), 1U);
    EXPECT_NEAR(occupied[0].x(), 2.0, px4_common::mapping::kDefaultVoxelResolutionM);
    EXPECT_NEAR(occupied[0].y(), 0.0, px4_common::mapping::kDefaultVoxelResolutionM);
    EXPECT_NEAR(occupied[0].z(), 0.0, px4_common::mapping::kDefaultVoxelResolutionM);
}

TEST_F(VoxelHashMapTest, EvictDistantRemovesFarVoxels) {
    std::vector<PointLivox> points;
    PointLivox point;
    point.x = 2.0;
    point.y = 0.0;
    point.z = 0.0;
    point.intensity = 100.0f;
    points.push_back(point);

    map_.Update(points, Eigen::Vector3d::Zero());
    EXPECT_EQ(map_.Size(), 1U);

    map_.EvictDistant(Eigen::Vector3d(100.0, 0.0, 0.0));
    EXPECT_EQ(map_.Size(), 0U);
    EXPECT_EQ(map_.OccupiedCount(), 0U);
}

TEST_F(VoxelHashMapTest, ImplementsVoxMapManagerInterface) {
    EXPECT_DOUBLE_EQ(map_.GetResolution(), px4_common::mapping::kDefaultVoxelResolutionM);
    EXPECT_EQ(map_.FramesDropped(), 0U);
    EXPECT_TRUE(map_.GetExtrinsicTranslation().isApprox(Eigen::Vector3d::Zero()));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
