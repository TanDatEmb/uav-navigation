#include <gtest/gtest.h>

#include <Eigen/Core>

#include <px4_navigation_common/mapping/voxel_types.hpp>
#include <px4_navigation_common/types.hpp>
#include <px4_mapping/voxel_hash_map.hpp>

using px4_navigation_common::PointLivox;
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
    EXPECT_NEAR(occupied[0].x(), 2.0, px4_navigation_common::mapping::kDefaultVoxelResolutionM);
    EXPECT_NEAR(occupied[0].y(), 0.0, px4_navigation_common::mapping::kDefaultVoxelResolutionM);
    EXPECT_NEAR(occupied[0].z(), 0.0, px4_navigation_common::mapping::kDefaultVoxelResolutionM);
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

TEST_F(VoxelHashMapTest, GetChangedPointsReturnsOccupiedVoxels) {
    std::vector<PointLivox> points;
    for (int i = 0; i < 10; ++i) {
        PointLivox p;
        p.x = 1.0f + static_cast<float>(i) * 0.05f;
        p.y = 0.0f;
        p.z = 0.0f;
        p.intensity = 100.0f;
        points.push_back(p);
    }

    map_.Update(points, Eigen::Vector3d::Zero());

    std::vector<PointLivox> changed;
    map_.GetChangedPoints(changed);

    EXPECT_FALSE(changed.empty());
    EXPECT_EQ(changed.size(), map_.OccupiedCount());
}

TEST_F(VoxelHashMapTest, GetChangedPointsAfterEvictDoesNotCrash) {
    std::vector<PointLivox> points;
    PointLivox point;
    point.x = 2.0;
    point.y = 0.0;
    point.z = 0.0;
    point.intensity = 100.0f;
    points.push_back(point);

    map_.Update(points, Eigen::Vector3d::Zero());
    ASSERT_EQ(map_.OccupiedCount(), 1U);

    // Evict all voxels far from the drone.
    const std::size_t evicted = map_.EvictDistant(Eigen::Vector3d(100.0, 0.0, 0.0));
    EXPECT_EQ(map_.Size(), 0U) << "Size after evict";
    EXPECT_EQ(map_.OccupiedCount(), 0U) << "OccupiedCount after evict";
    EXPECT_GT(evicted, 0U) << "Expected at least one voxel evicted";

    // Calling GetChangedPoints after eviction must not access deallocated
    // voxels. Previously GetChangedPoints held Voxel* pointers outside the
    // lock, so concurrent eviction could use-after-free.
    std::vector<PointLivox> changed;
    map_.GetChangedPoints(changed);
    EXPECT_TRUE(changed.empty()) << "changed size=" << changed.size();
}

TEST_F(VoxelHashMapTest, ImplementsVoxMapManagerInterface) {
    EXPECT_DOUBLE_EQ(map_.GetResolution(), px4_navigation_common::mapping::kDefaultVoxelResolutionM);
    EXPECT_EQ(map_.FramesDropped(), 0U);
    EXPECT_TRUE(map_.GetExtrinsicTranslation().isApprox(Eigen::Vector3d::Zero()));
}
