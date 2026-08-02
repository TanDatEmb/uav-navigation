#include <gtest/gtest.h>

#include <string>

#include "frame_id_contract.hpp"

namespace livox_ros {
namespace {

struct Header {
  std::string frame_id;
};

struct FakeMessage {
  Header header;
};

TEST(FrameIdContractTest, AssignsDistinctLidarAndImuHeaders) {
  const auto frames = makeFrameIdContract("livox_frame", "livox_imu_frame");
  FakeMessage lidar;
  FakeMessage imu;

  assignLidarFrameId(lidar, frames);
  assignImuFrameId(imu, frames);

  EXPECT_EQ(lidar.header.frame_id, "livox_frame");
  EXPECT_EQ(imu.header.frame_id, "livox_imu_frame");
  EXPECT_NE(lidar.header.frame_id, imu.header.frame_id);
}

TEST(FrameIdContractTest, RejectsEmptyOrAliasedFrames) {
  EXPECT_THROW(makeFrameIdContract("", "livox_imu_frame"), std::invalid_argument);
  EXPECT_THROW(makeFrameIdContract("livox_frame", ""), std::invalid_argument);
  EXPECT_THROW(makeFrameIdContract("livox_frame", "livox_frame"),
               std::invalid_argument);
}

}  // namespace
}  // namespace livox_ros
