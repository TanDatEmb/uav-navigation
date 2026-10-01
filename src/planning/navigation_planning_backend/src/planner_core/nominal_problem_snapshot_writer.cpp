#include "nominal_problem_snapshot_writer.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <utility>

namespace navigation_planning_backend {

NominalProblemSnapshotWriter::NominalProblemSnapshotWriter(
    const std::size_t capacity, std::string directory)
    : capacity_(std::max<std::size_t>(1U, capacity)),
      directory_(std::move(directory)) {
  writeStats(false);
  worker_ = std::thread([this] { run(); });
}

NominalProblemSnapshotWriter::~NominalProblemSnapshotWriter() noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  condition_.notify_one();
  if (worker_.joinable()) worker_.join();
  writeStats(true);
  const auto dropped = dropped_count_.load(std::memory_order_relaxed);
  if (dropped != 0U) {
    std::fprintf(stderr,
                 "[planner] nominal snapshot writer dropped %llu bounded captures\n",
                 static_cast<unsigned long long>(dropped));
  }
}

bool NominalProblemSnapshotWriter::enqueue(
    std::optional<traj_opt::NominalProblemSnapshot> snapshot,
    const navigation_world_model::WorldModelViewPtr& world,
    const bool include_world_snapshot) noexcept {
  if (!snapshot.has_value() || directory_.empty()) return false;
  submitted_count_.fetch_add(1U, std::memory_order_relaxed);
  try {
    Job job;
    job.snapshot = std::move(snapshot);
    job.world = world;
    job.include_world_snapshot = include_world_snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_ || queue_.size() >= capacity_) {
        dropped_count_.fetch_add(1U, std::memory_order_relaxed);
        return false;
      }
      queue_.emplace_back(std::move(job));
      enqueued_count_.fetch_add(1U, std::memory_order_relaxed);
    }
    condition_.notify_one();
    return true;
  } catch (...) {
    dropped_count_.fetch_add(1U, std::memory_order_relaxed);
    return false;
  }
}

std::uint64_t NominalProblemSnapshotWriter::droppedCount() const noexcept {
  return dropped_count_.load(std::memory_order_relaxed);
}

void NominalProblemSnapshotWriter::writeStats(
    const bool capture_complete) noexcept {
  try {
    std::size_t pending = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending = queue_.size();
    }
    pending += static_cast<std::size_t>(
        active_write_count_.load(std::memory_order_relaxed));
    std::error_code directory_error;
    std::filesystem::create_directories(directory_, directory_error);
    if (directory_error) {
      stats_write_error_count_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    const auto path = std::filesystem::path(directory_) /
        "nominal_problem_snapshot_capture.json";
    const auto temporary = std::filesystem::path(directory_) /
        "nominal_problem_snapshot_capture.json.tmp";
    std::ofstream output(temporary, std::ios::out | std::ios::trunc);
    if (!output) {
      stats_write_error_count_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    output << "{\"schema_version\":1"
           << ",\"capacity\":" << capacity_
           << ",\"submitted_records\":"
           << submitted_count_.load(std::memory_order_relaxed)
           << ",\"accepted_records\":"
           << enqueued_count_.load(std::memory_order_relaxed)
           << ",\"written_records\":"
           << written_count_.load(std::memory_order_relaxed)
           << ",\"dropped_records\":"
           << dropped_count_.load(std::memory_order_relaxed)
           << ",\"write_error_count\":"
           << write_error_count_.load(std::memory_order_relaxed)
           << ",\"stats_write_error_count\":"
           << stats_write_error_count_.load(std::memory_order_relaxed)
           << ",\"pending_records\":" << pending
           << ",\"capture_complete\":"
           << (capture_complete ? "true" : "false")
           << "}\n";
    output.close();
    if (!output) {
      stats_write_error_count_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    std::error_code rename_error;
    std::filesystem::rename(temporary, path, rename_error);
    if (rename_error) {
      stats_write_error_count_.fetch_add(1U, std::memory_order_relaxed);
      std::error_code remove_error;
      std::filesystem::remove(temporary, remove_error);
    }
  } catch (...) {
    stats_write_error_count_.fetch_add(1U, std::memory_order_relaxed);
  }
}

void NominalProblemSnapshotWriter::run() noexcept {
  for (;;) {
    Job job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] {
        return stopping_ || !queue_.empty();
      });
      if (queue_.empty() && stopping_) return;
      job = std::move(queue_.front());
      queue_.pop_front();
    }
    active_write_count_.store(1U, std::memory_order_relaxed);
    bool written = false;
    try {
      if (job.snapshot.has_value() && job.include_world_snapshot && job.world) {
        job.snapshot->diagnostic_world_snapshot = job.world->diagnosticSnapshot();
      }
      if (job.snapshot.has_value()) {
        written = !traj_opt::writeNominalProblemSnapshotJson(
            *job.snapshot, directory_).empty();
      }
    } catch (...) {
      // Diagnostic capture is best effort and cannot affect planning.
    }
    if (written) {
      written_count_.fetch_add(1U, std::memory_order_relaxed);
    } else {
      write_error_count_.fetch_add(1U, std::memory_order_relaxed);
    }
    active_write_count_.store(0U, std::memory_order_relaxed);
    writeStats(false);
  }
}

}  // namespace navigation_planning_backend
