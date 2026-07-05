#include <px4_common/time/pose_buffer.hpp>

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace {

using px4_common::time::PoseBuffer;
using px4_common::time::PoseSample;

TEST(PoseBuffer, PushAndLookupExact) {
    PoseBuffer buf(10);
    PoseSample s1;
    s1.t_ns = 1'000'000'000LL;
    s1.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    s1.orientation = Eigen::Quaterniond::Identity();
    buf.Push(s1);

    PoseSample out;
    EXPECT_TRUE(buf.Lookup(1'000'000'000LL, out));
    EXPECT_DOUBLE_EQ(out.position.x(), 1.0);
    EXPECT_EQ(buf.Size(), 1u);
}

TEST(PoseBuffer, InterpolatesPosition) {
    PoseBuffer buf(10);
    PoseSample s1;
    s1.t_ns = 0;
    s1.position = Eigen::Vector3d(0.0, 0.0, 0.0);
    s1.orientation = Eigen::Quaterniond::Identity();
    PoseSample s2;
    s2.t_ns = 1'000'000'000LL;
    s2.position = Eigen::Vector3d(2.0, 0.0, 0.0);
    s2.orientation = Eigen::Quaterniond::Identity();
    buf.Push(s1);
    buf.Push(s2);

    PoseSample out;
    EXPECT_TRUE(buf.Lookup(500'000'000LL, out));
    EXPECT_NEAR(out.position.x(), 1.0, 1e-12);
}

TEST(PoseBuffer, DropsNonMonotonic) {
    PoseBuffer buf(10);
    PoseSample s1;
    s1.t_ns = 100;
    buf.Push(s1);
    PoseSample s2;
    s2.t_ns = 50;
    buf.Push(s2);
    EXPECT_EQ(buf.NonMonotonicCount(), 1u);
    EXPECT_EQ(buf.Size(), 1u);
}

TEST(PoseBuffer, LookupMissOutsideWindow) {
    PoseBuffer buf(10);
    PoseSample s1;
    s1.t_ns = 0;
    buf.Push(s1);
    PoseSample out;
    EXPECT_FALSE(buf.Lookup(1'000'000'000LL, out));
    EXPECT_EQ(buf.LookupMissCount(), 1u);
}

TEST(PoseBuffer, CapacityOverflow) {
    PoseBuffer buf(2);
    for (int64_t i = 0; i < 5; ++i) {
        PoseSample s;
        s.t_ns = i;
        buf.Push(s);
    }
    EXPECT_EQ(buf.Size(), 2u);
    EXPECT_EQ(buf.OverflowCount(), 3u);
}

TEST(PoseBuffer, TimeWindowTrim) {
    PoseBuffer buf(100, std::chrono::nanoseconds(100));
    for (int64_t i = 0; i < 200; i += 10) {
        PoseSample s;
        s.t_ns = i;
        buf.Push(s);
    }
    // With a window of 100 ns and newest at 190, oldest kept should be >= 90.
    EXPECT_GE(buf.Size(), 10u);
    PoseSample out;
    EXPECT_FALSE(buf.Lookup(0, out));
}

}  // namespace
