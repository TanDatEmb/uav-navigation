#pragma once

#include <cstddef>
#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>

namespace uav::nav::rog {

enum class PairDropReason { kExpiredCloud, kExpiredOdom, kDuplicateCloud, kDuplicateOdom };

struct PairingCounters {
  std::size_t cloud_received{0};
  std::size_t odom_received{0};
  std::size_t paired{0};
  std::size_t expired_cloud{0};
  std::size_t expired_odom{0};
  std::size_t duplicate_cloud{0};
  std::size_t duplicate_odom{0};
  std::size_t timestamp_mismatch{0};
};

template <typename Cloud, typename Odom>
class ExactPairingCache {
 public:
  explicit ExactPairingCache(std::size_t capacity) : capacity_(capacity) {}
  [[nodiscard]] bool insertCloud(std::int64_t stamp, Cloud value) {
    if (clouds_.find(stamp) != clouds_.end()) { ++counters_.duplicate_cloud; return false; }
    if (clouds_.size() >= capacity_) { clouds_.erase(clouds_.begin()); ++counters_.expired_cloud; }
    clouds_.emplace(stamp, Timed<Cloud>{std::move(value), std::chrono::steady_clock::now()});
    ++counters_.cloud_received; return true;
  }
  [[nodiscard]] bool insertOdom(std::int64_t stamp, Odom value) {
    if (odometry_.find(stamp) != odometry_.end()) { ++counters_.duplicate_odom; return false; }
    if (odometry_.size() >= capacity_) { odometry_.erase(odometry_.begin()); ++counters_.expired_odom; }
    odometry_.emplace(stamp, Timed<Odom>{std::move(value), std::chrono::steady_clock::now()});
    ++counters_.odom_received; return true;
  }
  [[nodiscard]] std::optional<std::pair<Cloud, Odom>> takePair(std::int64_t stamp) {
    const auto cloud = clouds_.find(stamp); const auto odom = odometry_.find(stamp);
    if (cloud == clouds_.end() || odom == odometry_.end()) return std::nullopt;
    auto result = std::make_pair(std::move(cloud->second.value), std::move(odom->second.value));
    clouds_.erase(cloud); odometry_.erase(odom); ++counters_.paired; return result;
  }
  [[nodiscard]] PairingCounters counters() const { return counters_; }
  [[nodiscard]] std::size_t cloudDepth() const noexcept { return clouds_.size(); }
  [[nodiscard]] std::size_t odomDepth() const noexcept { return odometry_.size(); }
  void expire(std::chrono::steady_clock::time_point now,
              std::chrono::milliseconds timeout) {
    for (auto iterator = clouds_.begin(); iterator != clouds_.end();) {
      if (now - iterator->second.arrival > timeout) { iterator = clouds_.erase(iterator); ++counters_.expired_cloud; }
      else ++iterator;
    }
    for (auto iterator = odometry_.begin(); iterator != odometry_.end();) {
      if (now - iterator->second.arrival > timeout) { iterator = odometry_.erase(iterator); ++counters_.expired_odom; }
      else ++iterator;
    }
  }
  void clear() { clouds_.clear(); odometry_.clear(); }

 private:
  template <typename Value>
  struct Timed { Value value; std::chrono::steady_clock::time_point arrival; };
  std::size_t capacity_;
  std::map<std::int64_t, Timed<Cloud>> clouds_;
  std::map<std::int64_t, Timed<Odom>> odometry_;
  PairingCounters counters_;
};

}  // namespace uav::nav::rog
