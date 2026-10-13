#pragma once

#include <memory>
#include <span>

#include "uavnav/core/event_recorder.hpp"

namespace uavnav::events {

/// Fans every batch out to two sinks, so one EventRecorder per process can write the JSONL file and
/// publish /events (SYSTEM_DESIGN §6.1). Chain two TeeSinks for more.
///
/// Contract:
/// - write() hands the same batch to the first sink, then to the second, always both: a sink that
///   fails or throws never keeps the other from receiving the batch.
/// - It returns success only when both succeeded; otherwise SinkError::kIo (a thrown exception counts
///   as a failure). It never throws. The recorder counts a failed write as one sink failure for the
///   batch, whichever child failed.
/// - Threading: no internal locking. Like any EventSink it is called only from the recorder's single
///   writer thread; do not call write() from two threads at once.
class TeeSink final : public EventSink {
 public:
  /// Fails with RecorderError::kNoSink when either sink is null.
  static Result<std::unique_ptr<TeeSink>, RecorderError> create(std::unique_ptr<EventSink> a,
                                                                std::unique_ptr<EventSink> b);

  Result<void, SinkError> write(std::span<const EventRecord> batch) override;

 private:
  TeeSink(std::unique_ptr<EventSink> a, std::unique_ptr<EventSink> b) noexcept;

  std::unique_ptr<EventSink> a_;
  std::unique_ptr<EventSink> b_;
};

}  // namespace uavnav::events
