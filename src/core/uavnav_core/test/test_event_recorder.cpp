#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <vector>

#include "uavnav/core/event.hpp"
#include "uavnav/core/event_recorder.hpp"

using namespace uavnav::events;
using namespace std::chrono_literals;

static_assert(to_string(Component::kPx4Bridge) == "px4_bridge");
static_assert(to_string(Component::kPx4Mode) == "px4_mode");
static_assert(to_string(SinkError::kIo) == "IO");

namespace {

// Generous detection bound for "did not block / did not hang"; never a tight timing claim.
constexpr auto kHangBound = 2s;

enum class SinkMode { kOk, kFail, kThrow };

// State shared between a test and its FakeSink (the recorder owns the sink itself).
struct SinkState {
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<std::vector<EventRecord>> batches;  // every batch passed to write()
  std::vector<SinkMode> script;                   // mode per write() call; kOk once exhausted
  std::size_t calls{0};
  bool block{false};  // while true, write() parks until release()
  bool entered{false};

  void release() {
    {
      std::lock_guard lock(mutex);
      block = false;
    }
    cv.notify_all();
  }

  bool wait_entered() {
    std::unique_lock lock(mutex);
    return cv.wait_for(lock, kHangBound, [this] { return entered; });
  }

  std::vector<EventRecord> delivered() {
    std::lock_guard lock(mutex);
    std::vector<EventRecord> all;
    for (const auto& b : batches) all.insert(all.end(), b.begin(), b.end());
    return all;
  }
};

class FakeSink final : public EventSink {
 public:
  explicit FakeSink(std::shared_ptr<SinkState> state) : state_(std::move(state)) {}

  uavnav::Result<void, SinkError> write(std::span<const EventRecord> batch) override {
    std::unique_lock lock(state_->mutex);
    state_->entered = true;
    state_->cv.notify_all();
    state_->cv.wait(lock, [this] { return !state_->block; });
    const std::size_t call = state_->calls++;
    const SinkMode mode = call < state_->script.size() ? state_->script[call] : SinkMode::kOk;
    if (mode == SinkMode::kThrow) throw std::runtime_error("sink exploded");
    if (mode == SinkMode::kFail) return std::unexpected(SinkError::kIo);
    state_->batches.emplace_back(batch.begin(), batch.end());
    return {};
  }

 private:
  std::shared_ptr<SinkState> state_;
};

// Declared after the recorder in a test, so it runs first on scope exit: a failed
// assertion can never leave the writer parked in the sink while the recorder joins it.
struct ReleaseOnExit {
  SinkState& state;
  ~ReleaseOnExit() { state.release(); }
};

EventRecord make_event(std::string_view name) {
  EventRecord r;
  r.component = Component::kSupervisor;
  r.event = name;
  return r;
}

std::vector<std::string_view> event_names(const std::vector<EventRecord>& records) {
  std::vector<std::string_view> names;
  for (const auto& r : records) names.push_back(r.event);
  return names;
}

}  // namespace

TEST(EventRecorder, DeliversRecordsInOrder) {
  auto state = std::make_shared<SinkState>();
  EventRecorder recorder(std::make_unique<FakeSink>(state));

  EXPECT_TRUE(recorder.emit(make_event("A")));
  EXPECT_TRUE(recorder.emit(make_event("B")));
  EXPECT_TRUE(recorder.emit(make_event("C")));
  recorder.flush();

  EXPECT_EQ(event_names(state->delivered()), (std::vector<std::string_view>{"A", "B", "C"}));
  const RecorderStats s = recorder.stats();
  EXPECT_EQ(s.emitted, 3u);
  EXPECT_EQ(s.written, 3u);
  EXPECT_EQ(s.dropped, 0u);
  EXPECT_EQ(s.sink_failures, 0u);
}

TEST(EventRecorder, AddValueRejectsSeventeenth) {
  EventRecord r = make_event("Values");
  for (int i = 0; i < 16; ++i) {
    EXPECT_TRUE(r.add_value("k", static_cast<double>(i)));
  }
  const EventRecord before = r;
  EXPECT_FALSE(r.add_value("overflow", 99.0));
  EXPECT_EQ(r.value_count, 16u);
  EXPECT_EQ(r.values.back().key, before.values.back().key);
  EXPECT_EQ(r.values.back().value, 15.0);
}

TEST(EventRecorder, FullRingDropsWithoutBlockingAndReportsCount) {
  auto state = std::make_shared<SinkState>();
  state->block = true;
  EventRecorder recorder(std::make_unique<FakeSink>(state), 4);
  ReleaseOnExit guard{*state};

  // Park the writer inside the sink so nothing drains the ring during the burst.
  ASSERT_TRUE(recorder.emit(make_event("Prime")));
  ASSERT_TRUE(state->wait_entered());

  auto burst = std::async(std::launch::async, [&recorder] {
    int rejected = 0;
    for (int i = 0; i < 10; ++i) {
      if (!recorder.emit(make_event("Burst"))) ++rejected;
    }
    return rejected;
  });
  const bool finished = burst.wait_for(kHangBound) == std::future_status::ready;
  if (!finished) state->release();  // unblock before failing so the test can exit
  ASSERT_TRUE(finished) << "emit() blocked on a blocked sink";

  const int rejected = burst.get();
  EXPECT_GE(rejected, 6);
  EXPECT_EQ(recorder.stats().dropped, static_cast<std::uint64_t>(rejected));

  state->release();
  recorder.flush();

  const auto delivered = state->delivered();
  const EventRecord* notice = nullptr;
  for (const auto& r : delivered) {
    if (r.event == "EventsDropped") notice = &r;
  }
  ASSERT_NE(notice, nullptr);
  EXPECT_EQ(notice->component, Component::kCore);
  ASSERT_EQ(notice->value_count, 1u);
  EXPECT_EQ(notice->values[0].key, "count");
  EXPECT_EQ(notice->values[0].value, static_cast<double>(recorder.stats().dropped));
  EXPECT_EQ(recorder.stats().written, 5u);  // Prime + 4 accepted burst records
}

TEST(EventRecorder, SinkFailureIsCountedAndRecorderKeepsRunning) {
  auto state = std::make_shared<SinkState>();
  state->script = {SinkMode::kFail};
  {
    EventRecorder recorder(std::make_unique<FakeSink>(state));
    ASSERT_TRUE(recorder.emit(make_event("A")));
    recorder.flush();
    ASSERT_TRUE(recorder.emit(make_event("B")));
    recorder.flush();

    EXPECT_EQ(recorder.stats().sink_failures, 1u);
    EXPECT_EQ(recorder.stats().written, 1u);
    EXPECT_EQ(event_names(state->delivered()), (std::vector<std::string_view>{"B"}));
  }  // destructor returns
}

TEST(EventRecorder, ThrowingSinkIsTreatedAsFailure) {
  auto state = std::make_shared<SinkState>();
  state->script = {SinkMode::kThrow};
  EventRecorder recorder(std::make_unique<FakeSink>(state));

  ASSERT_TRUE(recorder.emit(make_event("A")));
  recorder.flush();
  ASSERT_TRUE(recorder.emit(make_event("B")));
  recorder.flush();

  EXPECT_EQ(recorder.stats().sink_failures, 1u);
  EXPECT_EQ(event_names(state->delivered()), (std::vector<std::string_view>{"B"}));
}

TEST(EventRecorder, FlushAndDestructorReturnWhileSinkKeepsFailing) {
  auto state = std::make_shared<SinkState>();
  state->script.assign(1000, SinkMode::kFail);

  auto run = std::async(std::launch::async, [state] {
    EventRecorder recorder(std::make_unique<FakeSink>(state));
    for (int i = 0; i < 3; ++i) {
      recorder.emit(make_event("Lost"));
      recorder.flush();
    }
    const auto failures = recorder.stats().sink_failures;
    recorder.emit(make_event("LostAtShutdown"));  // left for the destructor's drain
    return failures;
  });
  ASSERT_EQ(run.wait_for(kHangBound), std::future_status::ready) << "flush or destructor hung";
  EXPECT_EQ(run.get(), 3u);
  EXPECT_TRUE(state->delivered().empty());
}

TEST(EventRecorder, DestructorDrainsPendingRecords) {
  auto state = std::make_shared<SinkState>();
  {
    EventRecorder recorder(std::make_unique<FakeSink>(state));
    for (int i = 0; i < 100; ++i) ASSERT_TRUE(recorder.emit(make_event("Pending")));
  }  // no flush: the destructor must deliver everything
  EXPECT_EQ(state->delivered().size(), 100u);
}

TEST(EventRecorder, NullSinkCountsFailuresInsteadOfCrashing) {
  EventRecorder recorder(nullptr);
  ASSERT_TRUE(recorder.emit(make_event("A")));
  recorder.flush();
  EXPECT_EQ(recorder.stats().sink_failures, 1u);
  EXPECT_EQ(recorder.stats().written, 0u);
}
