#ifndef PX4_MAPPING_TIME_POSE_BUFFER_HPP_
#define PX4_MAPPING_TIME_POSE_BUFFER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>

namespace px4_mapping::time {

using Nanoseconds = std::chrono::nanoseconds;

/** @brief Timestamped pose sample used for scan-time interpolation. */
struct PoseSample {
    int64_t t_ns{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

/**
 * @brief Fixed-capacity, time-windowed pose buffer with SLERP interpolation.
 *
 * Mapping owns this buffer because its samples are an implementation detail of
 * scan deskewing and map alignment, not a mapping-to-navigation contract.
 */
class PoseBuffer {
   public:
    static constexpr size_t kDefaultMaxSamples = 500;
    static constexpr Nanoseconds kDefaultWindow{5'000'000'000LL};

    explicit PoseBuffer(size_t max_samples = kDefaultMaxSamples,
                        Nanoseconds window = kDefaultWindow)
        : max_samples_(max_samples), window_(window) {}

    void Push(const PoseSample& sample) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!buffer_.empty() && sample.t_ns <= buffer_.back().t_ns) {
            non_monotonic_count_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (buffer_.size() >= max_samples_) {
            buffer_.pop_front();
            overflow_count_.fetch_add(1, std::memory_order_relaxed);
        }
        buffer_.push_back(sample);
        trim_front(sample.t_ns);
    }

    bool Lookup(int64_t t_ns, PoseSample& out) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) {
            lookup_miss_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        const auto hi = std::lower_bound(buffer_.begin(), buffer_.end(), t_ns,
                                         [](const PoseSample& sample, int64_t time_ns) {
                                             return sample.t_ns < time_ns;
                                         });
        if (hi == buffer_.end()) {
            out = buffer_.back();
            return true;
        }
        if (hi == buffer_.begin()) {
            out = buffer_.front();
            return true;
        }
        if (hi->t_ns == t_ns) {
            out = *hi;
            return true;
        }

        const auto lo = std::prev(hi);
        const int64_t span_ns = hi->t_ns - lo->t_ns;
        const double alpha = static_cast<double>(t_ns - lo->t_ns) / static_cast<double>(span_ns);
        out.t_ns = t_ns;
        out.position = (1.0 - alpha) * lo->position + alpha * hi->position;
        out.orientation = lo->orientation.slerp(alpha, hi->orientation).normalized();
        return true;
    }

    size_t Size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size();
    }

    void Clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.clear();
    }

    uint64_t NonMonotonicCount() const noexcept {
        return non_monotonic_count_.load(std::memory_order_relaxed);
    }
    uint64_t OverflowCount() const noexcept {
        return overflow_count_.load(std::memory_order_relaxed);
    }
    uint64_t LookupMissCount() const noexcept {
        return lookup_miss_count_.load(std::memory_order_relaxed);
    }

   private:
    void trim_front(int64_t newest_t_ns) {
        const int64_t cutoff = newest_t_ns >= window_.count() ? newest_t_ns - window_.count() : 0;
        while (!buffer_.empty() && buffer_.front().t_ns < cutoff) {
            buffer_.pop_front();
        }
    }

    size_t max_samples_;
    Nanoseconds window_;
    mutable std::mutex mutex_;
    std::deque<PoseSample> buffer_;
    std::atomic<uint64_t> non_monotonic_count_{0};
    std::atomic<uint64_t> overflow_count_{0};
    mutable std::atomic<uint64_t> lookup_miss_count_{0};
};

}  // namespace px4_mapping::time

#endif  // PX4_MAPPING_TIME_POSE_BUFFER_HPP_
