#include <gtest/gtest.h>

#include <px4_mapping/voxel.hpp>

using px4_mapping::Voxel;
using px4_mapping::VoxelPool;

TEST(VoxelPoolTest, AllocateDeallocateCycle) {
    constexpr std::size_t kCapacity = 4U;
    VoxelPool pool(kCapacity);

    EXPECT_EQ(pool.Available(), kCapacity);

    const Eigen::Vector3i idx(1, 2, 3);
    Voxel *voxel = pool.Allocate(idx);
    ASSERT_NE(voxel, nullptr);
    EXPECT_EQ(voxel->index, idx);
    EXPECT_EQ(pool.Available(), kCapacity - 1U);

    pool.Deallocate(voxel);
    EXPECT_EQ(pool.Available(), kCapacity);
}

TEST(VoxelPoolTest, ReturnsNullWhenExhausted) {
    constexpr std::size_t kCapacity = 2U;
    VoxelPool pool(kCapacity);

    Voxel *a = pool.Allocate(Eigen::Vector3i(0, 0, 0));
    Voxel *b = pool.Allocate(Eigen::Vector3i(1, 0, 0));
    Voxel *c = pool.Allocate(Eigen::Vector3i(2, 0, 0));

    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(c, nullptr);
}

TEST(VoxelPoolTest, ReusedVoxelIsInitialized) {
    VoxelPool pool(2U);
    Voxel *voxel = pool.Allocate(Eigen::Vector3i(5, 6, 7));
    voxel->log_odds = 1.0f;
    voxel->is_occupied = true;

    pool.Deallocate(voxel);
    Voxel *reused = pool.Allocate(Eigen::Vector3i(0, 0, 0));

    ASSERT_EQ(reused, voxel);
    EXPECT_EQ(reused->log_odds, 0.0f);
    EXPECT_FALSE(reused->is_occupied);
    EXPECT_EQ(reused->occupied_list_index, -1);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
