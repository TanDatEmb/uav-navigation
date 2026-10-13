#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "uavnav/core/event.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/tee_sink.hpp"

using namespace uavnav::events;

namespace {

enum class Mode { kOk, kFail, kThrow };

struct Probe {
  std::vector<std::size_t> batch_sizes;  // one entry per write() call
};

class ProbeSink final : public EventSink {
 public:
  ProbeSink(Probe& probe, Mode mode) : probe_(probe), mode_(mode) {}
  uavnav::Result<void, SinkError> write(std::span<const EventRecord> batch) override {
    probe_.batch_sizes.push_back(batch.size());
    if (mode_ == Mode::kThrow) throw std::runtime_error("sink exploded");
    if (mode_ == Mode::kFail) return std::unexpected(SinkError::kIo);
    return {};
  }

 private:
  Probe& probe_;
  Mode mode_;
};

std::unique_ptr<TeeSink> make_tee(Probe& a, Mode ma, Probe& b, Mode mb) {
  auto tee = TeeSink::create(std::make_unique<ProbeSink>(a, ma), std::make_unique<ProbeSink>(b, mb));
  EXPECT_TRUE(tee.has_value());
  return tee ? std::move(*tee) : nullptr;
}

}  // namespace

TEST(TeeSink, WritesBothAndReportsEitherFailure) {
  const std::vector<EventRecord> batch(3);

  {  // both fine
    Probe a, b;
    auto tee = make_tee(a, Mode::kOk, b, Mode::kOk);
    ASSERT_TRUE(tee);
    EXPECT_TRUE(tee->write(batch).has_value());
    EXPECT_EQ(a.batch_sizes, (std::vector<std::size_t>{3}));
    EXPECT_EQ(b.batch_sizes, (std::vector<std::size_t>{3}));
  }
  {  // the first fails: kIo, and the second is still written
    Probe a, b;
    auto tee = make_tee(a, Mode::kFail, b, Mode::kOk);
    ASSERT_TRUE(tee);
    const auto r = tee->write(batch);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), SinkError::kIo);
    EXPECT_EQ(a.batch_sizes, (std::vector<std::size_t>{3}));
    EXPECT_EQ(b.batch_sizes, (std::vector<std::size_t>{3}));
  }
  {  // the second fails: kIo, the first was written
    Probe a, b;
    auto tee = make_tee(a, Mode::kOk, b, Mode::kFail);
    ASSERT_TRUE(tee);
    const auto r = tee->write(batch);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), SinkError::kIo);
    EXPECT_EQ(a.batch_sizes.size(), 1u);
    EXPECT_EQ(b.batch_sizes.size(), 1u);
  }
  {  // both fail
    Probe a, b;
    auto tee = make_tee(a, Mode::kFail, b, Mode::kFail);
    ASSERT_TRUE(tee);
    const auto r = tee->write(batch);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), SinkError::kIo);
    EXPECT_EQ(a.batch_sizes.size(), 1u);
    EXPECT_EQ(b.batch_sizes.size(), 1u);
  }
}

TEST(TeeSink, ThrowingSinkDoesNotStopTheOther) {
  const std::vector<EventRecord> batch(2);
  {
    Probe a, b;
    auto tee = make_tee(a, Mode::kThrow, b, Mode::kOk);
    ASSERT_TRUE(tee);
    uavnav::Result<void, SinkError> r;
    EXPECT_NO_THROW(r = tee->write(batch));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), SinkError::kIo);
    EXPECT_EQ(b.batch_sizes, (std::vector<std::size_t>{2}));
  }
  {
    Probe a, b;
    auto tee = make_tee(a, Mode::kOk, b, Mode::kThrow);
    ASSERT_TRUE(tee);
    uavnav::Result<void, SinkError> r;
    EXPECT_NO_THROW(r = tee->write(batch));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), SinkError::kIo);
    EXPECT_EQ(a.batch_sizes, (std::vector<std::size_t>{2}));
  }
}

TEST(TeeSink, KeepsWorkingAfterAFailure) {
  const std::vector<EventRecord> batch(1);
  Probe a, b;
  auto tee = make_tee(a, Mode::kFail, b, Mode::kOk);
  ASSERT_TRUE(tee);
  EXPECT_FALSE(tee->write(batch).has_value());
  EXPECT_FALSE(tee->write(batch).has_value());
  EXPECT_EQ(a.batch_sizes.size(), 2u);
  EXPECT_EQ(b.batch_sizes.size(), 2u);
}

TEST(TeeSink, RejectsANullSink) {
  Probe a;
  const auto no_second = TeeSink::create(std::make_unique<ProbeSink>(a, Mode::kOk), nullptr);
  ASSERT_FALSE(no_second.has_value());
  EXPECT_EQ(no_second.error(), RecorderError::kNoSink);
  const auto no_first = TeeSink::create(nullptr, std::make_unique<ProbeSink>(a, Mode::kOk));
  ASSERT_FALSE(no_first.has_value());
  EXPECT_EQ(no_first.error(), RecorderError::kNoSink);
  const auto neither = TeeSink::create(nullptr, nullptr);
  ASSERT_FALSE(neither.has_value());
  EXPECT_EQ(neither.error(), RecorderError::kNoSink);
}

TEST(TeeSink, DrivesARecorderEndToEnd) {
  Probe a, b;
  auto tee = make_tee(a, Mode::kOk, b, Mode::kFail);
  ASSERT_TRUE(tee);
  auto rec = EventRecorder::create(std::move(tee));
  ASSERT_TRUE(rec.has_value());
  EXPECT_TRUE((*rec)->emit(EventRecord{}));
  (*rec)->flush();
  EXPECT_EQ((*rec)->stats().sink_failures, 1u);
  rec->reset();  // joins the writer before the probes go out of scope
  EXPECT_EQ(a.batch_sizes, b.batch_sizes);
  EXPECT_FALSE(a.batch_sizes.empty());
}
