#pragma once

#include <filesystem>
#include <fstream>
#include <memory>
#include <span>
#include <utility>
#include <string>

#include "uavnav/core/event.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/result.hpp"

// JSONL event file (SYSTEM_DESIGN §6.1): one JSON object per record, one record per
// line. Nothing here throws: failures are SinkError::kIo values.
namespace uavnav::events {

/// Appends records to a file, one JSON line each, flushed once per batch. Not
/// thread-safe: the recorder calls it from its single writer thread.
class JsonlSink final : public EventSink {
 public:
  /// Opens `file` for appending, creating missing parent directories. Uses the
  /// std::error_code overloads of <filesystem>, so no exception escapes; any
  /// failure (including allocation failure) is kIo.
  static Result<std::unique_ptr<JsonlSink>, SinkError> open(const std::filesystem::path& file) noexcept;

  /// kIo when the stream fails (disk full, file gone read-only, ...). The batch is
  /// then lost, matching the recorder's contract; a later batch may succeed. A
  /// failed write can leave a partial line, so the next write starts on a fresh line.
  Result<void, SinkError> write(std::span<const EventRecord> batch) override;

 private:
  explicit JsonlSink(std::ofstream out) : out_(std::move(out)) {}

  std::ofstream out_;
  std::string buffer_;        // reused across batches to limit allocations
  bool needs_newline_{false};  // previous write failed; its last line may be partial
};

/// The JSON object for one record, without a trailing newline. Number values that
/// are NaN or infinite become `null` so the line is always valid JSON.
std::string to_json_line(const EventRecord& record);

}  // namespace uavnav::events
