#ifndef PX4_ROS_COM_TIME_SYNC_HPP_
#define PX4_ROS_COM_TIME_SYNC_HPP_

#include <cstdint>

#include <rclcpp/rclcpp.hpp>

namespace px4_ros_com::time {

inline constexpr int64_t kNanosecondsPerMicrosecond = 1000LL;
inline constexpr int64_t kTimestampDomainGuardNs = 5'000'000'000LL;
inline constexpr int64_t kRosSimTimeUpperBoundNs = 1'000'000'000'000'000LL;
inline constexpr int64_t kUnixEpochLowerBoundNs = 1'500'000'000'000'000'000LL;

// PX4 uORB timestamps are transported as microseconds on DDS topics.
inline int64_t Px4MicrosecondsToNanoseconds(uint64_t px4_us) {
    return static_cast<int64_t>(px4_us) * kNanosecondsPerMicrosecond;
}

inline uint64_t RosNanosecondsToPx4Microseconds(int64_t ros_ns) {
    if (ros_ns <= 0) {
        return 0;
    }
    return static_cast<uint64_t>(ros_ns / kNanosecondsPerMicrosecond);
}

inline uint64_t RosTimeToPx4Microseconds(const rclcpp::Time& ros_time) {
    return RosNanosecondsToPx4Microseconds(ros_time.nanoseconds());
}

inline bool IsWithinTimestampDomainGuard(int64_t sample_ns, int64_t now_ns,
                                         int64_t guard_ns = kTimestampDomainGuardNs) {
    const int64_t delta_ns = (sample_ns >= now_ns) ? (sample_ns - now_ns) : (now_ns - sample_ns);
    return delta_ns <= guard_ns;
}

// Adapts PX4 timestamps to ROS domain with one centralized rule:
// - If timestamps already match ROS time, pass through.
// - If ROS uses sim-time (small monotonic values) but PX4 is in unix-epoch
//   domain, anchor the first sample and map subsequent samples by delta.
class Px4TimestampDomainAdapter {
   public:
    int64_t ToRosNanoseconds(uint64_t px4_us, int64_t ros_now_ns) {
        const int64_t sample_ns = Px4MicrosecondsToNanoseconds(px4_us);
        if (sample_ns <= 0) {
            return sample_ns;
        }

        if (IsWithinTimestampDomainGuard(sample_ns, ros_now_ns)) {
            return sample_ns;
        }

        const bool ros_looks_like_sim_time =
            ros_now_ns > 0 && ros_now_ns < kRosSimTimeUpperBoundNs;
        const bool px4_looks_like_unix_epoch = sample_ns > kUnixEpochLowerBoundNs;

        if (!(ros_looks_like_sim_time && px4_looks_like_unix_epoch)) {
            return sample_ns;
        }

        if (!epoch_to_sim_anchor_valid_ || sample_ns < anchor_px4_ns_) {
            anchor_px4_ns_ = sample_ns;
            anchor_ros_ns_ = ros_now_ns;
            epoch_to_sim_anchor_valid_ = true;
        }

        return anchor_ros_ns_ + (sample_ns - anchor_px4_ns_);
    }

    bool HasEpochToSimAnchor() const {
        return epoch_to_sim_anchor_valid_;
    }

   private:
    bool epoch_to_sim_anchor_valid_{false};
    int64_t anchor_px4_ns_{0};
    int64_t anchor_ros_ns_{0};
};

}  // namespace px4_ros_com::time

#endif  // PX4_ROS_COM_TIME_SYNC_HPP_