#include "uavnav/core/tee_sink.hpp"

#include <utility>

namespace uavnav::events {

namespace {

/// One child's write; a throw is a failure, never an escape.
bool write_ok(EventSink& sink, std::span<const EventRecord> batch) noexcept {
  try {
    return sink.write(batch).has_value();
  } catch (...) {
    return false;
  }
}

}  // namespace

TeeSink::TeeSink(std::unique_ptr<EventSink> a, std::unique_ptr<EventSink> b) noexcept
    : a_(std::move(a)), b_(std::move(b)) {}

Result<std::unique_ptr<TeeSink>, RecorderError> TeeSink::create(std::unique_ptr<EventSink> a,
                                                                std::unique_ptr<EventSink> b) {
  if (!a || !b) return std::unexpected(RecorderError::kNoSink);
  return std::unique_ptr<TeeSink>(new TeeSink(std::move(a), std::move(b)));
}

Result<void, SinkError> TeeSink::write(std::span<const EventRecord> batch) {
  const bool a_ok = write_ok(*a_, batch);
  const bool b_ok = write_ok(*b_, batch);
  if (!a_ok || !b_ok) return std::unexpected(SinkError::kIo);
  return {};
}

}  // namespace uavnav::events
