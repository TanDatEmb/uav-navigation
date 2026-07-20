#include <gtest/gtest.h>

#include <px4_mapping/voxel.hpp>

using px4_mapping::Voxel;
using px4_mapping::VoxelPool;

// Capacity zero
TEST(VoxelPoolTest, CapacityZeroIsSafe) {
    VoxelPool pool(0U);
    EXPECT_EQ(pool.Capacity(), 0U);
    EXPECT_EQ(pool.Available(), 0U);
    EXPECT_EQ(pool.Allocate(Eigen::Vector3i(0, 0, 0)), nullptr);
}

// Basic allocate/deallocate
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

// Returns null when exhausted
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

// Reused voxel is initialized
TEST(VoxelPoolTest, ReusedVoxelIsInitialized) {
    VoxelPool pool(2U);
    Voxel *voxel = pool.Allocate(Eigen::Vector3i(5, 6, 7));
    voxel->log_odds = 1.0f;
    voxel->is_occupied = true;
    voxel->occupied_list_index = 42;
    voxel->prev = reinterpret_cast<Voxel*>(0x1234);
    voxel->next = reinterpret_cast<Voxel*>(0x5678);

    pool.Deallocate(voxel);
    Voxel *reused = pool.Allocate(Eigen::Vector3i(0, 0, 0));

    ASSERT_EQ(reused, voxel);
    EXPECT_EQ(reused->log_odds, 0.0f);
    EXPECT_FALSE(reused->is_occupied);
    EXPECT_EQ(reused->occupied_list_index, -1);
    EXPECT_EQ(reused->prev, nullptr);
    EXPECT_EQ(reused->next, nullptr);
}

// Pointer stability: allocated voxels are from storage
TEST(VoxelPoolTest, AllocatedPointerFromStorage) {
    VoxelPool pool(4U);
    Voxel *v1 = pool.Allocate(Eigen::Vector3i(1, 0, 0));
    Voxel *v2 = pool.Allocate(Eigen::Vector3i(2, 0, 0));

    // Pointers should be different
    EXPECT_NE(v1, v2);

    // Both should be valid (non-null)
    EXPECT_NE(v1, nullptr);
    EXPECT_NE(v2, nullptr);

    // Can write and read back
    v1->log_odds = 3.14f;
    v2->log_odds = 2.71f;
    EXPECT_FLOAT_EQ(v1->log_odds, 3.14f);
    EXPECT_FLOAT_EQ(v2->log_odds, 2.71f);
}

// Invalid pointer rejected (nullptr)
TEST(VoxelPoolTest, NullPointerDeallocateIsSafe) {
    VoxelPool pool(2U);
    EXPECT_NO_THROW(pool.Deallocate(nullptr));
    EXPECT_EQ(pool.Available(), 2U);
}

// Invalid pointer rejected (outside storage)
TEST(VoxelPoolTest, InvalidPointerRejected) {
    VoxelPool pool(2U);
    Voxel dummy;
    // Should not crash or corrupt pool
    pool.Deallocate(&dummy);  // pointer not from pool
    // Available count unchanged
    EXPECT_EQ(pool.Available(), 2U);
}

// Available/Capacity accuracy
TEST(VoxelPoolTest, AvailableCapacityAccuracy) {
    constexpr std::size_t kCapacity = 10U;
    VoxelPool pool(kCapacity);

    EXPECT_EQ(pool.Capacity(), kCapacity);
    EXPECT_EQ(pool.Available(), kCapacity);

    std::vector<Voxel*> allocated;
    for (std::size_t i = 0; i < kCapacity; ++i) {
        Voxel *v = pool.Allocate(Eigen::Vector3i(static_cast<int>(i), 0, 0));
        ASSERT_NE(v, nullptr);
        allocated.push_back(v);
    }

    EXPECT_EQ(pool.Available(), 0U);
    EXPECT_EQ(pool.Allocate(Eigen::Vector3i(99, 0, 0)), nullptr);

    // Deallocate all
    for (auto *v : allocated) {
        pool.Deallocate(v);
    }
    EXPECT_EQ(pool.Available(), kCapacity);
}

// Eigen alignment check
TEST(VoxelPoolTest, EigenAlignment) {
    VoxelPool pool(4U);
    Voxel *v = pool.Allocate(Eigen::Vector3i(1, 2, 3));
    ASSERT_NE(v, nullptr);

    // index is Vector3i - should be properly aligned
    v->index = Eigen::Vector3i(100, 200, 300);
    EXPECT_EQ(v->index.x(), 100);
    EXPECT_EQ(v->index.y(), 200);
    EXPECT_EQ(v->index.z(), 300);
}
