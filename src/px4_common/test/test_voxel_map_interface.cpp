#include <gtest/gtest.h>

#include <px4_common/mapping/voxel_map_interface.hpp>

#include <Eigen/Core>
#include <vector>

using px4_common::mapping::IVoxMapManager;

namespace {

/**
 * @brief Minimal stub implementation used only to verify the interface shape.
 */
class StubVoxMapManager : public IVoxMapManager {
   public:
    double GetResolution() const override {
        return 0.2;
    }

    void GetOccupiedPointsInRadius(const Eigen::Vector3d &center, double radius,
                                   std::vector<Eigen::Vector3d> &out) override {
        (void)center;
        (void)radius;
        out.clear();
    }

    bool IsReady() const noexcept override {
        return ready_;
    }
    std::uint64_t FramesDropped() const noexcept override {
        return dropped_;
    }
    Eigen::Vector3d GetExtrinsicTranslation() const noexcept override {
        return extrinsic_;
    }

    bool ready_{true};
    std::uint64_t dropped_{0U};
    Eigen::Vector3d extrinsic_{Eigen::Vector3d::Zero()};
};

}  // namespace

TEST(VoxelMapInterfaceTest, StubImplementsInterface) {
    StubVoxMapManager stub;
    IVoxMapManager *interface_ptr = &stub;

    EXPECT_DOUBLE_EQ(interface_ptr->GetResolution(), 0.2);
    EXPECT_TRUE(interface_ptr->IsReady());
    EXPECT_EQ(interface_ptr->FramesDropped(), 0U);
    EXPECT_TRUE(interface_ptr->GetExtrinsicTranslation().isApprox(Eigen::Vector3d::Zero()));

    std::vector<Eigen::Vector3d> out;
    interface_ptr->GetOccupiedPointsInRadius(Eigen::Vector3d::Zero(), 10.0, out);
    EXPECT_TRUE(out.empty());
}

TEST(VoxelMapInterfaceTest, DroppedFramesCounterIsMutable) {
    StubVoxMapManager stub;
    stub.dropped_ = 42U;
    EXPECT_EQ(stub.FramesDropped(), 42U);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
