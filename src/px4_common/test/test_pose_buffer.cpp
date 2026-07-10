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

TEST(PoseBuffer, TrimFrontDoesNotUnderflowAtEarlyTimestamps) {
    PoseBuffer buf(10, std::chrono::nanoseconds(5'000'000'000LL));
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

TEST(PoseBuffer, TrimFrontEvictsOldSamplesWithinWindow) {
    PoseBuffer buf(10, std::chrono::nanoseconds(5'000'000'000LL));
    PoseSample s1;
    s1.t_ns = 1'000'000'000LL;
    s1.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    s1.orientation = Eigen::Quaterniond::Identity();
    PoseSample s2;
    s2.t_ns = 6'000'000'000LL;
    s2.position = Eigen::Vector3d(6.0, 0.0, 0.0);
    s2.orientation = Eigen::Quaterniond::Identity();
    buf.Push(s1);
    buf.Push(s2);

    // Window is 5s; newest is 6s so cutoff is 1s. s1 at 1s is exactly at the
    // boundary and is kept (front.t_ns < cutoff is false).
    EXPECT_EQ(buf.Size(), 2u);
    PoseSample out;
    EXPECT_TRUE(buf.Lookup(1'000'000'000LL, out));
    EXPECT_DOUBLE_EQ(out.position.x(), 1.0);

    PoseSample s3;
    s3.t_ns = 6'100'000'000LL;
    s3.position = Eigen::Vector3d(6.1, 0.0, 0.0);
    s3.orientation = Eigen::Quaterniond::Identity();
    buf.Push(s3);

    // Cutoff is now 1.1s; s1 at 1s should be evicted. The remaining front is s2.
    EXPECT_EQ(buf.Size(), 2u);
    EXPECT_TRUE(buf.Lookup(6'000'000'000LL, out));
    EXPECT_DOUBLE_EQ(out.position.x(), 6.0);
    // Lookup before the new front should clamp to the new front, not return s1.
    EXPECT_TRUE(buf.Lookup(1'000'000'000LL, out));
    EXPECT_DOUBLE_EQ(out.position.x(), 6.0);
}

TEST(PoseBuffer, LookupMissWhenEmpty) {
    PoseBuffer buf(10);
    PoseSample out;
    EXPECT_FALSE(buf.Lookup(1'000'000'000LL, out));
    EXPECT_EQ(buf.LookupMissCount(), 1u);
}

TEST(PoseBuffer, LookupClampsBeyondBack) {
    PoseBuffer buf(10);
    PoseSample s1;
    s1.t_ns = 0;
    s1.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    s1.orientation = Eigen::Quaterniond::Identity();
    buf.Push(s1);
    PoseSample out;
    EXPECT_TRUE(buf.Lookup(1'000'000'000LL, out));
    EXPECT_DOUBLE_EQ(out.position.x(), 1.0);
    EXPECT_EQ(out.t_ns, 0);
}

TEST(PoseBuffer, LookupClampsBeforeFront) {
    PoseBuffer buf(10);
    PoseSample s1;
    s1.t_ns = 1'000'000'000LL;
    s1.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    s1.orientation = Eigen::Quaterniond::Identity();
    buf.Push(s1);
    PoseSample out;
    EXPECT_TRUE(buf.Lookup(0, out));
    EXPECT_DOUBLE_EQ(out.position.x(), 1.0);
    EXPECT_EQ(out.t_ns, 1'000'000'000LL);
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

TEST(PoseBuffer, LookupFrontBoundaryReturnsFront) {
    PoseBuffer buf(10);
    PoseSample s1;
    s1.t_ns = 1'000'000'000LL;
    s1.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    s1.orientation = Eigen::Quaterniond::Identity();
    PoseSample s2;
    s2.t_ns = 2'000'000'000LL;
    s2.position = Eigen::Vector3d(2.0, 0.0, 0.0);
    s2.orientation = Eigen::Quaterniond::Identity();
    buf.Push(s1);
    buf.Push(s2);

    PoseSample out;
    EXPECT_TRUE(buf.Lookup(1'000'000'000LL, out));
    EXPECT_DOUBLE_EQ(out.position.x(), 1.0);
    EXPECT_EQ(out.t_ns, 1'000'000'000LL);
}

TEST(PoseBuffer, LookupBackBoundaryReturnsBack) {
    PoseBuffer buf(10);
    PoseSample s1;
    s1.t_ns = 1'000'000'000LL;
    s1.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    s1.orientation = Eigen::Quaterniond::Identity();
    PoseSample s2;
    s2.t_ns = 2'000'000'000LL;
    s2.position = Eigen::Vector3d(2.0, 0.0, 0.0);
    s2.orientation = Eigen::Quaterniond::Identity();
    buf.Push(s1);
    buf.Push(s2);

    PoseSample out;
    EXPECT_TRUE(buf.Lookup(2'000'000'000LL, out));
    EXPECT_DOUBLE_EQ(out.position.x(), 2.0);
    EXPECT_EQ(out.t_ns, 2'000'000'000LL);
}

TEST(PoseBuffer, LookupBeyondBackReturnsBack) {
    PoseBuffer buf(10);
    PoseSample s1;
    s1.t_ns = 1'000'000'000LL;
    s1.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    s1.orientation = Eigen::Quaterniond::Identity();
    PoseSample s2;
    s2.t_ns = 2'000'000'000LL;
    s2.position = Eigen::Vector3d(2.0, 0.0, 0.0);
    s2.orientation = Eigen::Quaterniond::Identity();
    buf.Push(s1);
    buf.Push(s2);

    PoseSample out;
    EXPECT_TRUE(buf.Lookup(3'000'000'000LL, out));
    EXPECT_DOUBLE_EQ(out.position.x(), 2.0);
    EXPECT_EQ(out.t_ns, 2'000'000'000LL);
}

TEST(PoseBuffer, LookupBeforeFrontReturnsFront) {
    PoseBuffer buf(10);
    PoseSample s1;
    s1.t_ns = 1'000'000'000LL;
    s1.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    s1.orientation = Eigen::Quaterniond::Identity();
    PoseSample s2;
    s2.t_ns = 2'000'000'000LL;
    s2.position = Eigen::Vector3d(2.0, 0.0, 0.0);
    s2.orientation = Eigen::Quaterniond::Identity();
    buf.Push(s1);
    buf.Push(s2);

    PoseSample out;
    EXPECT_TRUE(buf.Lookup(500'000'000LL, out));
    EXPECT_DOUBLE_EQ(out.position.x(), 1.0);
    EXPECT_EQ(out.t_ns, 1'000'000'000LL);
}

TEST(PoseBuffer, InterpolatesOrientation) {
    PoseBuffer buf(10);
    PoseSample s1;
    s1.t_ns = 0;
    s1.position = Eigen::Vector3d::Zero();
    s1.orientation = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
    PoseSample s2;
    s2.t_ns = 1'000'000'000LL;
    s2.position = Eigen::Vector3d::Zero();
    s2.orientation = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
    buf.Push(s1);
    buf.Push(s2);

    PoseSample out;
    EXPECT_TRUE(buf.Lookup(500'000'000LL, out));
    Eigen::Quaterniond expected(Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ()));
    EXPECT_NEAR(out.orientation.angularDistance(expected), 0.0, 1e-9);
}

}  // namespace
