#ifndef PX4_COMMON_TIME_POSE_BUFFER_HPP_
#define PX4_COMMON_TIME_POSE_BUFFER_HPP_

#include <px4_common/math/transforms.hpp>

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <utility>

namespace px4_common::time {

using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
using Nanoseconds = std::chrono::nanoseconds;

/**
 * @brief A timestamped pose sample for interpolation buffers.
 */
struct PoseSample {
    /// Timestamp in nanoseconds since an arbitrary but consistent epoch.
    int64_t t_ns{0};

    /// Position in metres.
    Eigen::Vector3d position = Eigen::Vector3d::Zero();

    /// Orientation as a unit quaternion.
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

/**
 * @brief Fixed-capacity time-windowed pose buffer with SLERP interpolation.
 *
 * - Push must be strictly monotonic in time; non-monotonic samples are
 *   dropped and counted.
 * - Lookup returns false if the requested time is outside the buffered
 *   window.
 * - Interpolation uses linear interpolation for position and spherical
 *   linear interpolation for orientation.
 */
class PoseBuffer {
   public:
    explicit PoseBuffer(size_t max_samples = kDefaultMaxSamples,
                        Nanoseconds window = kDefaultWindow)
        : max_samples_(max_samples), window_(window) {}

    /**
     * @brief Push a new pose sample into the buffer.
     *
     * Samples must have strictly increasing t_ns. Out-of-order samples are
     * dropped and the non-monotonic counter is incremented.
     */
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
        TrimFront(sample.t_ns);
    }

    /**
     * @brief Look up a pose at the requested time using SLERP interpolation.
     *
     * @param t_ns Target timestamp in nanoseconds.
     * @param[out] out Interpolated pose sample.
     * @return true if a pose could be interpolated or exactly matched,
     *         false if the target time is outside the buffered window.
     */
    bool Lookup(int64_t t_ns, PoseSample& out) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) {
            lookup_miss_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        auto hi = std::lower_bound(buffer_.begin(), buffer_.end(), t_ns,
                                   [](const PoseSample& s, int64_t t) {
                                       return s.t_ns < t;
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

        auto lo = std::prev(hi);
        const int64_t span_ns = hi->t_ns - lo->t_ns;
        const double alpha =
            span_ns > 0 ? static_cast<double>(t_ns - lo->t_ns) / static_cast<double>(span_ns) : 0.0;
        out.t_ns = t_ns;
        out.position = (1.0 - alpha) * lo->position + alpha * hi->position;
        out.orientation = lo->orientation.slerp(alpha, hi->orientation).normalized();
        return true;
    }

    /// Number of samples currently buffered.
    size_t Size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size();
    }

    /// True if the buffer contains no samples.
    bool Empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.empty();
    }

    /// Clear all samples.
    void Clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.clear();
    }

    /// Number of non-monotonic samples dropped since construction.
    uint64_t NonMonotonicCount() const noexcept {
        return non_monotonic_count_.load(std::memory_order_relaxed);
    }

    /// Number of samples dropped due to capacity overflow.
    uint64_t OverflowCount() const noexcept {
        return overflow_count_.load(std::memory_order_relaxed);
    }

    /// Number of lookup requests that missed the buffered window.
    uint64_t LookupMissCount() const noexcept {
        return lookup_miss_count_.load(std::memory_order_relaxed);
    }

    static constexpr size_t kDefaultMaxSamples = 500;
    static constexpr Nanoseconds kDefaultWindow = Nanoseconds(5'000'000'000LL);

   private:
    void TrimFront(int64_t newest_t_ns) {
        const int64_t cutoff =
            newest_t_ns >= window_.count() ? newest_t_ns - window_.count() : 0;
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

}  // namespace px4_common::time

#endif  // PX4_COMMON_TIME_POSE_BUFFER_HPP_
