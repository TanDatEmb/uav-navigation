#include <gtest/gtest.h>

#include <px4_ros_com/time_sync.hpp>

TEST(TimeSync, Px4UsToRosNs) {
    const uint64_t px4_us = 12'345'678ULL;
    const int64_t ros_ns = px4_ros_com::time::Px4MicrosecondsToNanoseconds(px4_us);
    EXPECT_EQ(ros_ns, 12'345'678'000LL);
}

TEST(TimeSync, RosNsToPx4Us) {
    const int64_t ros_ns = 9'876'543'210LL;
    const uint64_t px4_us = px4_ros_com::time::RosNanosecondsToPx4Microseconds(ros_ns);
    EXPECT_EQ(px4_us, 9'876'543ULL);
}

TEST(TimeSync, NegativeRosNsToPx4UsIsZero) {
    EXPECT_EQ(px4_ros_com::time::RosNanosecondsToPx4Microseconds(-1), 0ULL);
    EXPECT_EQ(px4_ros_com::time::RosNanosecondsToPx4Microseconds(0), 0ULL);
}

TEST(TimeSync, DomainGuard) {
    const int64_t now_ns = 10'000'000'000LL;
    const int64_t near_stamp_ns = 12'000'000'000LL;
    const int64_t far_stamp_ns = 40'000'000'000LL;

    EXPECT_TRUE(px4_ros_com::time::IsWithinTimestampDomainGuard(near_stamp_ns, now_ns));
    EXPECT_FALSE(px4_ros_com::time::IsWithinTimestampDomainGuard(far_stamp_ns, now_ns));
}