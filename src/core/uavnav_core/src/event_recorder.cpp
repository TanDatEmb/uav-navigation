#include "uavnav/core/event_recorder.hpp"

#include <chrono>
#include <utility>

#include "uavnav/core/time.hpp"

namespace uavnav::events {

EventRecorder::EventRecorder(std::unique_ptr<EventSink> sink, std::size_t capacity)
    : sink_(std::move(sink)), capacity_(capacity) {
  // Both buffers keep this capacity for life (swap and clear() never shrink), so
  // emit() never allocates.
  ring_.reserve(capacity_);
  batch_.reserve(capacity_);
  writer_ = std::thread([this] { run(); });
}

EventRecorder::~EventRecorder() {
  {
    std::lock_guard lock(mutex_);
    stop_ = true;
  }
  wake_.notify_one();
  writer_.join();  // the writer drains the ring before it exits
}

bool EventRecorder::emit(const EventRecord& record) noexcept {
  std::lock_guard lock(mutex_);
  if (ring_.size() >= capacity_) {
    ++stats_.dropped;
    return false;
  }
  ring_.push_back(record);  // within reserved capacity, trivially copyable: no throw
  ++stats_.emitted;
  return true;
}

void EventRecorder::flush() {
  std::unique_lock lock(mutex_);
  const std::uint64_t target = stats_.emitted;
  if (consumed_ >= target) return;
  flush_requested_ = true;
  wake_.notify_one();
  drained_.wait(lock, [&] { return consumed_ >= target || writer_done_; });
}

RecorderStats EventRecorder::stats() const noexcept {
  std::lock_guard lock(mutex_);
  return stats_;
}

void EventRecorder::run() noexcept {
  const auto period = std::chrono::nanoseconds(limits::kEventWriterPeriod.ns);
  std::unique_lock lock(mutex_);
  for (;;) {
    wake_.wait_for(lock, period, [this] { return stop_ || flush_requested_; });
    flush_requested_ = false;
    batch_.swap(ring_);  // ring_ takes over the empty buffer
    const std::uint64_t dropped_total = stats_.dropped;

    lock.unlock();  // the sink is only ever called without the lock
    const Delivery d = deliver(batch_, dropped_total);
    const std::uint64_t handled = batch_.size();
    batch_.clear();
    lock.lock();

    stats_.written += d.written;
    stats_.sink_failures += d.failures;
    consumed_ += handled;
    drained_.notify_all();
    if (stop_ && ring_.empty()) break;
  }
  writer_done_ = true;
  drained_.notify_all();
}

EventRecorder::Delivery EventRecorder::deliver(std::span<const EventRecord> batch,
                                               std::uint64_t dropped_total) noexcept {
  Delivery d;
  if (dropped_total > reported_drops_) {
    EventRecord notice;
    notice.t_steady_ns = time::steady_now().ns;  // no ROS clock in core: t_ros_ns stays 0
    notice.component = Component::kCore;
    notice.event = "EventsDropped";
    notice.add_value("count", static_cast<double>(dropped_total - reported_drops_));
    if (write_to_sink(std::span(&notice, 1))) {
      reported_drops_ = dropped_total;  // on failure the count is re-reported next time
    } else {
      ++d.failures;
    }
  }
  if (!batch.empty()) {
    if (write_to_sink(batch)) {
      d.written += batch.size();
    } else {
      ++d.failures;  // the batch is discarded
    }
  }
  return d;
}

bool EventRecorder::write_to_sink(std::span<const EventRecord> batch) noexcept {
  if (!sink_) return false;
  try {
    return sink_->write(batch).has_value();
  } catch (...) {
    return false;  // a throwing sink is a failing sink; nothing escapes the writer
  }
}

}  // namespace uavnav::events
