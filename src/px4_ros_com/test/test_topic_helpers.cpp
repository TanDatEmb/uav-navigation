#include <px4_ros_com/topic_helpers.hpp>

#include <gtest/gtest.h>

#include <string>

namespace {

struct NoVersionMessage {};

struct ZeroVersionMessage {
    static constexpr int MESSAGE_VERSION = 0;
};

struct VersionedMessage {
    static constexpr int MESSAGE_VERSION = 2;
};

using px4_ros_com::topic::MessageVersionSuffix;
using px4_ros_com::topic::Px4TopicName;

TEST(TopicHelpers, NoVersionReturnsEmpty) {
    EXPECT_EQ(MessageVersionSuffix<NoVersionMessage>(), "");
}

TEST(TopicHelpers, ZeroVersionReturnsEmpty) {
    EXPECT_EQ(MessageVersionSuffix<ZeroVersionMessage>(), "");
}

TEST(TopicHelpers, NonZeroVersionReturnsSuffix) {
    EXPECT_EQ(MessageVersionSuffix<VersionedMessage>(), "_v2");
}

TEST(TopicHelpers, Px4TopicNameAppendsSuffix) {
    EXPECT_EQ(Px4TopicName<VersionedMessage>("/fmu/out/vehicle_local_position"),
              "/fmu/out/vehicle_local_position_v2");
}

}  // namespace
