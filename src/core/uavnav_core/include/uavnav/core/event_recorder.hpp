#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string_view>
#include <thread>
#include <vector>

#include "uavnav/core/event.hpp"
#include "uavnav/core/limits.hpp"
#include "uavnav/core/result.hpp"

namespace uavnav::events {

enum class SinkError : std::uint8_t { kIo };

constexpr std::string_view to_string(SinkError e) {
  switch (e) {
    case SinkError::kIo: return "IO";
  }
  return "unknown";  // unreachable for valid enumerators
}

/// Destination of event batches. Called only from the recorder's writer thread,
/// never while the recorder holds a lock. A write may be slow; it must return.
class EventSink {
 public:
  virtual ~EventSink() = default;
  virtual Result<void, SinkError> write(std::span<const EventRecord> batch) = 0;
};

struct RecorderStats {
  std::uint64_t emitted{0};        ///< records accepted by emit()
  std::uint64_t written{0};        ///< accepted records the sink took (EventsDropped notices not counted)
  std::uint64_t dropped{0};        ///< records emit() rejected because the ring was full
  std::uint64_t sink_failures{0};  ///< sink writes that failed or threw; each loses that batch
};

/// One per process (SYSTEM_DESIGN §6.1). Callers copy records into a bounded ring
/// under a short mutex; a writer thread drains the ring every kEventWriterPeriod and
/// hands batches to the sink outside the lock. A slow sink fills the ring and makes
/// emit() drop (counted, reported later as an `EventsDropped` record); it never
/// makes emit() wait. A failing or throwing sink loses that batch, is counted, and
/// the writer carries on. A null sink counts every batch as a sink failure.
class EventRecorder {
 public:
  explicit EventRecorder(std::unique_ptr<EventSink> sink,
                         std::size_t capacity = limits::kEventRingCapacity);
  /// Stops the writer after it drains the ring; never throws. Returns once the sink
  /// does, so a sink that never returns from write() would block it.
  ~EventRecorder();

  EventRecorder(const EventRecorder&) = delete;
  EventRecorder& operator=(const EventRecorder&) = delete;

  /// false = dropped (ring full). Never waits on the sink. Thread-safe.
  bool emit(const EventRecord& record) noexcept;

  /// Blocks until every record accepted before the call has been handed to the sink
  /// (a failed write counts as handed over). Thread-safe; must not be called from
  /// inside EventSink::write.
  void flush();

  RecorderStats stats() const noexcept;

 private:
  struct Delivery {
    std::uint64_t written{0};
    std::uint64_t failures{0};
  };

  void run() noexcept;
  Delivery deliver(std::span<const EventRecord> batch, std::uint64_t dropped_total) noexcept;
  bool write_to_sink(std::span<const EventRecord> batch) noexcept;

  std::unique_ptr<EventSink> sink_;
  const std::size_t capacity_;

  mutable std::mutex mutex_;
  std::condition_variable wake_;     // writer: stop, flush request, or period
  std::condition_variable drained_;  // flush(): consumed_ advanced or writer exited
  std::vector<EventRecord> ring_;    // guarded; filled by emit(), swapped out by the writer
  std::uint64_t consumed_{0};        // guarded; accepted records the writer has finished with
  bool flush_requested_{false};      // guarded
  bool stop_{false};                 // guarded
  bool writer_done_{false};          // guarded
  RecorderStats stats_;              // guarded

  // Writer-thread only.
  std::vector<EventRecord> batch_;
  std::uint64_t reported_drops_{0};

  std::thread writer_;  // last: started after every member above is initialised
};

}  // namespace uavnav::events
