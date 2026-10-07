#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <stop_token>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "uavnav/core/result.hpp"
#include "uavnav/core/spsc_queue.hpp"

using uavnav::concurrency::QueueError;
using uavnav::concurrency::QueueStats;
using uavnav::concurrency::SpscQueue;
using namespace std::chrono_literals;

static_assert(to_string(QueueError::kFull) == "FULL");
static_assert(uavnav::ReasonEnum<QueueError>);
static_assert(SpscQueue<int, 3>::capacity() == 3);
static_assert(!std::is_copy_constructible_v<SpscQueue<int, 3>>);
static_assert(!std::is_copy_assignable_v<SpscQueue<int, 3>>);
static_assert(!std::is_move_constructible_v<SpscQueue<int, 3>>);
static_assert(!std::is_move_assignable_v<SpscQueue<int, 3>>);
static_assert(noexcept(std::declval<SpscQueue<int, 3>&>().try_push(1)));
static_assert(noexcept(std::declval<SpscQueue<int, 3>&>().try_pop()));

namespace {

// Generous bound for "did not block / did not hang"; never a tight timing claim.
constexpr auto kHangBound = 10s;

bool same(const QueueStats& a, const QueueStats& b) {
  return a.pushed == b.pushed && a.popped == b.popped && a.rejected == b.rejected;
}

}  // namespace

TEST(SpscQueue, FifoOrderCapacityAndUnmovedRejects) {
  SpscQueue<std::unique_ptr<int>, 4> q;
  for (int i = 1; i <= 4; ++i) {
    ASSERT_TRUE(q.try_push(std::make_unique<int>(i)).has_value()) << i;
  }
  auto fifth = std::make_unique<int>(5);
  const auto full = q.try_push(std::move(fifth));
  ASSERT_FALSE(full.has_value());
  EXPECT_EQ(full.error(), QueueError::kFull);
  ASSERT_NE(fifth, nullptr);  // a rejected item is left untouched
  EXPECT_EQ(*fifth, 5);
  for (int i = 1; i <= 4; ++i) {
    auto item = q.try_pop();
    ASSERT_TRUE(item.has_value()) << i;
    ASSERT_NE(*item, nullptr);
    EXPECT_EQ(**item, i);
  }
  EXPECT_FALSE(q.try_pop().has_value());
  EXPECT_TRUE(same(q.stats(), QueueStats{4, 4, 1}));
}

TEST(SpscQueue, WrapsAroundManyTimes) {
  SpscQueue<int, 3> q;
  for (int i = 0; i < 10'000; ++i) {
    ASSERT_TRUE(q.try_push(int{i}).has_value()) << i;
    const auto item = q.try_pop();
    ASSERT_TRUE(item.has_value()) << i;
    EXPECT_EQ(*item, i);
  }
  // Fill to capacity after many wraps: exactly Capacity slots are usable.
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(q.try_push(int{i}).has_value());
  }
  EXPECT_FALSE(q.try_push(99).has_value());
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(q.try_pop(), std::optional<int>{i});
  }
  EXPECT_TRUE(same(q.stats(), QueueStats{10'003, 10'003, 1}));
}

TEST(SpscQueue, DestructorDestroysRemainingItems) {
  auto counter = std::make_shared<int>(0);
  {
    SpscQueue<std::shared_ptr<int>, 4> q;
    ASSERT_TRUE(q.try_push(std::shared_ptr<int>(counter)).has_value());
    ASSERT_TRUE(q.try_push(std::shared_ptr<int>(counter)).has_value());
    ASSERT_TRUE(q.try_push(std::shared_ptr<int>(counter)).has_value());
    ASSERT_TRUE(q.try_pop().has_value());  // popped copy destroyed at end of statement
    EXPECT_EQ(counter.use_count(), 3);
  }
  EXPECT_EQ(counter.use_count(), 1);
}

TEST(SpscQueue, FullPushNeverBlocks) {  // Review Focus 4
  SpscQueue<std::unique_ptr<int>, 4> q;
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(q.try_push(std::make_unique<int>(i)).has_value());
  }
  auto pushes = std::async(std::launch::async, [&q] {
    int full = 0;
    for (int i = 0; i < 1000; ++i) {
      auto item = std::make_unique<int>(i);
      const auto r = q.try_push(std::move(item));
      if (!r.has_value() && r.error() == QueueError::kFull && item != nullptr) {
        ++full;
      }
    }
    return full;
  });
  ASSERT_EQ(pushes.wait_for(kHangBound), std::future_status::ready);
  EXPECT_EQ(pushes.get(), 1000);
  EXPECT_EQ(q.stats().rejected, 1000U);
  EXPECT_EQ(q.stats().pushed, 4U);
}

TEST(SpscQueue, ConcurrentProducerConsumerAccountsForEveryItem) {  // Review Focus 4
  constexpr std::uint64_t kItems = 200'000;
  struct Outcome {
    std::uint64_t accepted = 0;
    std::uint64_t rejected = 0;
    std::uint64_t popped = 0;
    bool strictly_increasing = true;
    bool stats_consistent_while_running = true;
    QueueStats final_stats{};
  };
  auto run = std::async(std::launch::async, [] {
    SpscQueue<std::uint64_t, 64> q;
    Outcome out;

    std::jthread consumer([&q, &out](const std::stop_token& stop) {
      std::optional<std::uint64_t> last;
      const auto drain = [&] {
        while (auto item = q.try_pop()) {
          if (last.has_value() && *item <= *last) {
            out.strictly_increasing = false;
          }
          last = *item;
          ++out.popped;
        }
      };
      for (;;) {
        q.wait_nonempty(stop);
        drain();
        if (stop.stop_requested()) {
          drain();  // the producer finished before stop was requested
          return;
        }
      }
    });

    std::jthread producer([&q, &out] {
      for (std::uint64_t i = 0; i < kItems; ++i) {
        if (q.try_push(std::uint64_t{i}).has_value()) {
          ++out.accepted;
        } else {
          ++out.rejected;
        }
      }
    });

    // A third thread reads the counters while both sides run.
    std::jthread monitor([&q, &out](const std::stop_token& stop) {
      while (!stop.stop_requested()) {
        const QueueStats s = q.stats();
        if (s.popped > s.pushed || s.pushed + s.rejected > kItems) {
          out.stats_consistent_while_running = false;
        }
        std::this_thread::yield();
      }
    });

    producer.join();
    consumer.request_stop();
    consumer.join();
    monitor.request_stop();
    monitor.join();
    out.final_stats = q.stats();
    return out;
  });
  ASSERT_EQ(run.wait_for(60s), std::future_status::ready);
  const Outcome out = run.get();
  EXPECT_TRUE(out.strictly_increasing);
  EXPECT_TRUE(out.stats_consistent_while_running);
  EXPECT_EQ(out.accepted + out.rejected, kItems);
  EXPECT_EQ(out.popped, out.accepted);
  EXPECT_EQ(out.popped + out.rejected, kItems);
  EXPECT_TRUE(same(out.final_stats, QueueStats{out.accepted, out.popped, out.rejected}));
}

TEST(SpscQueue, WaitReturnsOnPushAndOnStop) {
  SpscQueue<int, 4> q;
  std::promise<std::optional<int>> first;
  auto first_done = first.get_future();
  std::promise<bool> second;
  auto second_done = second.get_future();

  std::jthread consumer([&](const std::stop_token& stop) {
    q.wait_nonempty(stop);
    first.set_value(q.try_pop());
    q.wait_nonempty(stop);  // empty queue: only a stop request may end this wait
    second.set_value(stop.stop_requested());
  });

  // Give the consumer time to block (not a timing assertion; the result does not depend on it).
  std::this_thread::sleep_for(50ms);
  ASSERT_TRUE(q.try_push(7).has_value());
  ASSERT_EQ(first_done.wait_for(kHangBound), std::future_status::ready);
  EXPECT_EQ(first_done.get(), std::optional<int>{7});

  // Nothing pushed and no stop: the second wait must still be blocked.
  EXPECT_EQ(second_done.wait_for(100ms), std::future_status::timeout);
  consumer.request_stop();
  ASSERT_EQ(second_done.wait_for(kHangBound), std::future_status::ready);
  EXPECT_TRUE(second_done.get());
}

TEST(SpscQueue, WaitReturnsAtOnceWhenNonEmptyOrAlreadyStopped) {
  SpscQueue<int, 2> q;
  std::stop_source source;
  ASSERT_TRUE(q.try_push(1).has_value());
  auto nonempty = std::async(std::launch::async, [&] { q.wait_nonempty(source.get_token()); });
  ASSERT_EQ(nonempty.wait_for(kHangBound), std::future_status::ready);
  ASSERT_TRUE(q.try_pop().has_value());
  source.request_stop();
  auto stopped = std::async(std::launch::async, [&] { q.wait_nonempty(source.get_token()); });
  ASSERT_EQ(stopped.wait_for(kHangBound), std::future_status::ready);
}
