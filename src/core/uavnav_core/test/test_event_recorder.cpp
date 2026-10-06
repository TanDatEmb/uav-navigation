#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

#include "uavnav/core/event.hpp"
#include "uavnav/core/event_builder.hpp"
#include "uavnav/core/event_recorder.hpp"

using namespace uavnav::events;
using namespace std::chrono_literals;

static_assert(to_string(Component::kPx4Bridge) == "px4_bridge");
static_assert(to_string(Component::kPx4Mode) == "px4_mode");
static_assert(to_string(SinkError::kIo) == "IO");
static_assert(to_string(RecorderError::kZeroCapacity) == "ZERO_CAPACITY");
static_assert(to_string(RecorderError::kNoSink) == "NO_SINK");

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

// Every test builds its recorder through create(); a failure here is a test bug.
std::unique_ptr<EventRecorder> make_recorder(std::unique_ptr<EventSink> sink,
                                             std::size_t capacity = uavnav::limits::kEventRingCapacity) {
  auto recorder = EventRecorder::create(std::move(sink), capacity);
  if (!recorder) throw std::runtime_error("EventRecorder::create failed");
  return std::move(*recorder);
}

std::vector<std::string_view> event_names(const std::vector<EventRecord>& records) {
  std::vector<std::string_view> names;
  for (const auto& r : records) names.push_back(r.event);
  return names;
}

}  // namespace

TEST(EventRecorder, DeliversRecordsInOrder) {
  auto state = std::make_shared<SinkState>();
  auto recorder = make_recorder(std::make_unique<FakeSink>(state));

  EXPECT_TRUE(recorder->emit(make_event("A")));
  EXPECT_TRUE(recorder->emit(make_event("B")));
  EXPECT_TRUE(recorder->emit(make_event("C")));
  recorder->flush();

  EXPECT_EQ(event_names(state->delivered()), (std::vector<std::string_view>{"A", "B", "C"}));
  const RecorderStats s = recorder->stats();
  EXPECT_EQ(s.emitted, 3u);
  EXPECT_EQ(s.written, 3u);
  EXPECT_EQ(s.dropped, 0u);
  EXPECT_EQ(s.sink_failures, 0u);
}

TEST(EventRecorder, AddValueRejectsSeventeenth) {
  EventRecord r = make_event("Values");
  static constexpr std::string_view kKeys[] = {"v00", "v01", "v02", "v03", "v04", "v05", "v06", "v07", "v08", "v09", "v10", "v11", "v12", "v13", "v14", "v15"};  // distinct: duplicates are rejected
  for (int i = 0; i < 16; ++i) {
    EXPECT_TRUE(r.add_value(kKeys[i], static_cast<double>(i)));
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
  auto recorder = make_recorder(std::make_unique<FakeSink>(state), 4);
  ReleaseOnExit guard{*state};

  // Park the writer inside the sink so nothing drains the ring during the burst.
  ASSERT_TRUE(recorder->emit(make_event("Prime")));
  ASSERT_TRUE(state->wait_entered());

  auto burst = std::async(std::launch::async, [&recorder] {
    int rejected = 0;
    for (int i = 0; i < 10; ++i) {
      if (!recorder->emit(make_event("Burst"))) ++rejected;
    }
    return rejected;
  });
  const bool finished = burst.wait_for(kHangBound) == std::future_status::ready;
  if (!finished) state->release();  // unblock before failing so the test can exit
  ASSERT_TRUE(finished) << "emit() blocked on a blocked sink";

  const int rejected = burst.get();
  EXPECT_GE(rejected, 6);
  EXPECT_EQ(recorder->stats().dropped, static_cast<std::uint64_t>(rejected));

  state->release();
  recorder->flush();

  const auto delivered = state->delivered();
  const EventRecord* notice = nullptr;
  for (const auto& r : delivered) {
    if (r.event == "EventsDropped") notice = &r;
  }
  ASSERT_NE(notice, nullptr);
  EXPECT_EQ(notice->component, Component::kCore);
  ASSERT_EQ(notice->value_count, 1u);
  EXPECT_EQ(notice->values[0].key, "count");
  EXPECT_EQ(notice->values[0].value, static_cast<double>(recorder->stats().dropped));
  EXPECT_EQ(recorder->stats().written, 5u);  // Prime + 4 accepted burst records
}

TEST(EventRecorder, SinkFailureIsCountedAndRecorderKeepsRunning) {
  auto state = std::make_shared<SinkState>();
  state->script = {SinkMode::kFail};
  {
    auto recorder = make_recorder(std::make_unique<FakeSink>(state));
    ASSERT_TRUE(recorder->emit(make_event("A")));
    recorder->flush();
    ASSERT_TRUE(recorder->emit(make_event("B")));
    recorder->flush();

    EXPECT_EQ(recorder->stats().sink_failures, 1u);
    EXPECT_EQ(recorder->stats().written, 1u);
    EXPECT_EQ(event_names(state->delivered()), (std::vector<std::string_view>{"B"}));
  }  // destructor returns
}

TEST(EventRecorder, ThrowingSinkIsTreatedAsFailure) {
  auto state = std::make_shared<SinkState>();
  state->script = {SinkMode::kThrow};
  auto recorder = make_recorder(std::make_unique<FakeSink>(state));

  ASSERT_TRUE(recorder->emit(make_event("A")));
  recorder->flush();
  ASSERT_TRUE(recorder->emit(make_event("B")));
  recorder->flush();

  EXPECT_EQ(recorder->stats().sink_failures, 1u);
  EXPECT_EQ(event_names(state->delivered()), (std::vector<std::string_view>{"B"}));
}

TEST(EventRecorder, FlushAndDestructorReturnWhileSinkKeepsFailing) {
  auto state = std::make_shared<SinkState>();
  state->script.assign(1000, SinkMode::kFail);

  auto run = std::async(std::launch::async, [state] {
    auto recorder = make_recorder(std::make_unique<FakeSink>(state));
    for (int i = 0; i < 3; ++i) {
      recorder->emit(make_event("Lost"));
      recorder->flush();
    }
    const auto failures = recorder->stats().sink_failures;
    recorder->emit(make_event("LostAtShutdown"));  // left for the destructor's drain
    return failures;
  });
  ASSERT_EQ(run.wait_for(kHangBound), std::future_status::ready) << "flush or destructor hung";
  EXPECT_EQ(run.get(), 3u);
  EXPECT_TRUE(state->delivered().empty());
}

TEST(EventRecorder, DestructorDrainsPendingRecords) {
  auto state = std::make_shared<SinkState>();
  {
    auto recorder = make_recorder(std::make_unique<FakeSink>(state));
    for (int i = 0; i < 100; ++i) ASSERT_TRUE(recorder->emit(make_event("Pending")));
  }  // no flush: the destructor must deliver everything
  EXPECT_EQ(state->delivered().size(), 100u);
}

TEST(EventRecorder, CreateRejectsZeroCapacityAndNullSink) {
  EXPECT_EQ(EventRecorder::create(std::make_unique<FakeSink>(std::make_shared<SinkState>()), 0).error(),
            RecorderError::kZeroCapacity);
  EXPECT_EQ(EventRecorder::create(nullptr, 4).error(), RecorderError::kNoSink);
  // Zero capacity is reported first, whatever else is wrong.
  EXPECT_EQ(EventRecorder::create(nullptr, 0).error(), RecorderError::kZeroCapacity);
}

TEST(EventRecorder, DroppedNoticeFollowsTheBatchItSummarises) {
  auto state = std::make_shared<SinkState>();
  state->block = true;
  auto recorder = make_recorder(std::make_unique<FakeSink>(state), 2);
  ReleaseOnExit guard{*state};

  // Park the writer inside the sink with "Prime" so the ring fills deterministically.
  ASSERT_TRUE(recorder->emit(make_event("Prime")));
  ASSERT_TRUE(state->wait_entered());
  EXPECT_TRUE(recorder->emit(make_event("A")));
  EXPECT_TRUE(recorder->emit(make_event("B")));
  EXPECT_FALSE(recorder->emit(make_event("C")));
  EXPECT_FALSE(recorder->emit(make_event("D")));

  state->release();
  recorder->flush();

  // The notice summarises the drops that happened before the batch [A, B] was taken,
  // so it follows that batch: log order matches time order.
  const auto delivered = state->delivered();
  EXPECT_EQ(event_names(delivered),
            (std::vector<std::string_view>{"Prime", "A", "B", "EventsDropped"}));
  ASSERT_EQ(delivered.size(), 4u);
  EXPECT_EQ(delivered[3].values[0].key, "count");
  EXPECT_EQ(delivered[3].values[0].value, 2.0);
  EXPECT_EQ(recorder->stats().written, 3u);  // the notice is not counted
  EXPECT_EQ(recorder->stats().dropped, 2u);
}

namespace {

// Sleeps 1 ms per batch so the ring really overflows; counts what it is given.
class SlowCountingSink final : public EventSink {
 public:
  uavnav::Result<void, SinkError> write(std::span<const EventRecord> batch) override {
    std::this_thread::sleep_for(1ms);
    for (const auto& r : batch) {
      if (r.event == "EventsDropped") {
        notices.fetch_add(1);
        reported_drops.fetch_add(static_cast<std::uint64_t>(r.values[0].value));
      } else {
        records.fetch_add(1);
      }
    }
    return {};
  }
  std::atomic<std::uint64_t> records{0}, notices{0}, reported_drops{0};
};

}  // namespace

TEST(EventRecorder, ConcurrentEmittersAccountForEveryRecord) {
  constexpr int kThreads = 4;
  constexpr int kEmitsPerThread = 10000;
  auto sink = std::make_unique<SlowCountingSink>();
  SlowCountingSink* observed = sink.get();  // the recorder owns it; alive until recorder dies
  auto recorder = make_recorder(std::move(sink), 1024);

  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&recorder] {
      for (int i = 0; i < kEmitsPerThread; ++i) recorder->emit(make_event("Burst"));
    });
  }
  for (auto& th : threads) th.join();
  recorder->flush();

  const RecorderStats s = recorder->stats();
  EXPECT_EQ(s.emitted + s.dropped, static_cast<std::uint64_t>(kThreads) * kEmitsPerThread);
  EXPECT_EQ(s.written, s.emitted);
  EXPECT_EQ(s.sink_failures, 0u);
  // The sink saw exactly the accepted records, and the notices add up to the drops:
  // none lost, none double-counted.
  EXPECT_EQ(observed->records.load(), s.emitted);
  EXPECT_EQ(observed->reported_drops.load(), s.dropped);
}

TEST(EventRecord, AddValueRejectsDuplicateKey) {
  EventRecord r = make_event("Dup");
  EXPECT_TRUE(r.add_value("a", 1.0));
  EXPECT_FALSE(r.add_value("a", 2.0));
  EXPECT_EQ(r.value_count, 1u);
  EXPECT_EQ(r.values[0].value, 1.0);
  // Equal by content, not by pointer: a different array holding the same text.
  const char other[] = {'a', '\0'};
  EXPECT_FALSE(r.add_value(std::string_view(other, 1), 3.0));
  EXPECT_TRUE(r.add_value("b", 4.0));
  EXPECT_EQ(r.value_count, 2u);
}

namespace {
enum class Phase : std::uint8_t { kRunning, kStale };
constexpr std::string_view to_string(Phase p) { return p == Phase::kRunning ? "RUNNING" : "STALE"; }
static_assert(uavnav::ReasonEnum<Phase>);
}  // namespace

TEST(EventBuilder, StoresTypedNames) {
  const uavnav::time::TimeSnapshot now{uavnav::time::SteadyTime{123}, uavnav::time::RosTime{456}};
  EventIdentity id;
  id.mission_id = 7;
  id.lio_epoch = 2;
  const EventRecord r = EventBuilder(Component::kLio, "PhaseChanged", now)
                            .states(Phase::kRunning, Phase::kStale)
                            .reason(Phase::kStale)
                            .identity(id)
                            .value("age_s", 0.5)
                            .build();
  EXPECT_EQ(r.t_steady_ns, 123);
  EXPECT_EQ(r.t_ros_ns, 456);
  EXPECT_EQ(r.component, Component::kLio);
  EXPECT_EQ(r.event, "PhaseChanged");
  EXPECT_EQ(r.state_before, "RUNNING");
  EXPECT_EQ(r.state_after, "STALE");
  EXPECT_EQ(r.reason, "STALE");
  EXPECT_EQ(r.identity.mission_id, 7u);
  EXPECT_EQ(r.identity.lio_epoch, 2u);
  ASSERT_EQ(r.value_count, 1u);
  EXPECT_EQ(r.values[0].key, "age_s");
}

TEST(EventBuilder, ValueIgnoresDuplicateAndSeventeenth) {
  const uavnav::time::TimeSnapshot now{};
  EventBuilder b(Component::kCore, "Many", now);
  b.value("first", 1.0).value("first", 2.0);  // duplicate: first one stays
  static constexpr std::string_view kKeys[] = {"k1", "k2", "k3", "k4", "k5", "k6", "k7", "k8",
                                               "k9", "k10", "k11", "k12", "k13", "k14", "k15", "k16"};
  for (const auto key : kKeys) b.value(key, 0.0);  // "first" + 16 = 17 distinct keys
  const EventRecord r = b.build();
  EXPECT_EQ(r.value_count, 16u);
  EXPECT_EQ(r.values[0].key, "first");
  EXPECT_EQ(r.values[0].value, 1.0);
  EXPECT_EQ(r.values[15].key, "k15");  // k16 was the 17th and is ignored
}
