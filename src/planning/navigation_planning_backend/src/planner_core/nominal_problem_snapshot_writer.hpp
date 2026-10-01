#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <navigation_world_model/world_model_view.hpp>
#include <traj_opt/nominal_trajectory_optimizer.hpp>

namespace navigation_planning_backend {

class NominalProblemSnapshotWriter final {
 public:
  explicit NominalProblemSnapshotWriter(std::size_t capacity,
                                        std::string directory);
  NominalProblemSnapshotWriter(const NominalProblemSnapshotWriter&) = delete;
  NominalProblemSnapshotWriter& operator=(
      const NominalProblemSnapshotWriter&) = delete;
  ~NominalProblemSnapshotWriter() noexcept;

  bool enqueue(std::optional<traj_opt::NominalProblemSnapshot> snapshot,
               const navigation_world_model::WorldModelViewPtr& world,
               bool include_world_snapshot) noexcept;
  [[nodiscard]] std::uint64_t droppedCount() const noexcept;

 private:
  struct Job {
    std::optional<traj_opt::NominalProblemSnapshot> snapshot;
    navigation_world_model::WorldModelViewPtr world;
    bool include_world_snapshot{false};
  };

  void writeStats(bool capture_complete) noexcept;
  void run() noexcept;

  const std::size_t capacity_;
  const std::string directory_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Job> queue_;
  bool stopping_{false};
  std::atomic<std::uint64_t> submitted_count_{0U};
  std::atomic<std::uint64_t> enqueued_count_{0U};
  std::atomic<std::uint64_t> written_count_{0U};
  std::atomic<std::uint64_t> dropped_count_{0U};
  std::atomic<std::uint64_t> write_error_count_{0U};
  std::atomic<std::uint64_t> stats_write_error_count_{0U};
  std::atomic<std::uint64_t> active_write_count_{0U};
  std::thread worker_;
};

}  // namespace navigation_planning_backend
