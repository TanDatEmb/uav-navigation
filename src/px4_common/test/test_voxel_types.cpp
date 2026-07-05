#include <gtest/gtest.h>

#include <px4_common/mapping/voxel_types.hpp>

using px4_common::mapping::kDefaultVoxelResolutionM;
using px4_common::mapping::kEvictRadiusM;
using px4_common::mapping::kInflationVoxels;
using px4_common::mapping::kLogOddsHit;
using px4_common::mapping::kLogOddsMax;
using px4_common::mapping::kLogOddsMin;
using px4_common::mapping::kLogOddsMiss;
using px4_common::mapping::kLogOddsOccupiedThreshold;
using px4_common::mapping::kMaxRayLengthM;
using px4_common::mapping::kVoxelPoolSize;

TEST(VoxelTypesTest, DefaultResolutionIsPositive) {
    EXPECT_GT(kDefaultVoxelResolutionM, 0.0);
}

TEST(VoxelTypesTest, PoolSizeExceedsMaxVoxels) {
    EXPECT_GT(kVoxelPoolSize, px4_common::mapping::kMaxVoxels);
}

TEST(VoxelTypesTest, LogOddsRangeIsOrdered) {
    EXPECT_LT(kLogOddsMin, kLogOddsMiss);
    EXPECT_LT(kLogOddsMiss, kLogOddsOccupiedThreshold);
    EXPECT_LT(kLogOddsOccupiedThreshold, kLogOddsHit);
    EXPECT_LT(kLogOddsHit, kLogOddsMax);
}

TEST(VoxelTypesTest, InflationVoxelsIsNonNegative) {
    EXPECT_GE(kInflationVoxels, 0);
}

TEST(VoxelTypesTest, EvictRadiusCoversPlanningRange) {
    EXPECT_GT(kEvictRadiusM, 15.0);
    EXPECT_LT(kEvictRadiusM, kMaxRayLengthM);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
