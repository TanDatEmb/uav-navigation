#include "navigation_runtime/execution_trace_snapshot.hpp"

#include <atomic>
#include <barrier>
#include <thread>
#include <type_traits>

#include <gtest/gtest.h>

namespace navigation_runtime {
namespace {

ExecutionTraceSnapshot record(std::uint64_t generation) {
  ExecutionTraceSnapshot value;
  value.planning_cycle_id = generation;
  value.solve_generation = generation;
  value.timestamp_ns = static_cast<std::int64_t>(generation * 10U);
  value.execution_localization_epoch = 7U;
  value.execution_goal_epoch = generation;
  value.execution_request_id = generation + 100U;
  value.execution_bundle_generation = generation + 1000U;
  value.execution_state_source_stamp_ns = static_cast<std::int64_t>(generation * 20U);
  value.execution_state_receive_stamp_ns = static_cast<std::int64_t>(generation * 30U);
  value.measured_position_at_state_source = Eigen::Vector3d(
      static_cast<double>(generation), static_cast<double>(generation + 1U),
      static_cast<double>(generation + 2U));
  value.measured_velocity_at_state_source = Eigen::Vector3d(
      static_cast<double>(generation + 3U), static_cast<double>(generation + 4U),
      static_cast<double>(generation + 5U));
  value.anchor_error_m = static_cast<double>(generation);
  value.backup_available = generation % 2U == 0U;
  return value;
}

TEST(ExecutionTraceStore, PublishesOneImmutableIdentityAndClockRecord) {
  static_assert(!std::is_copy_constructible_v<ExecutionTraceStore>);
  ExecutionTraceStore store;
  EXPECT_FALSE(store.load());

  ASSERT_TRUE(store.publish(record(4U)));
  const auto snapshot = store.load();
  ASSERT_TRUE(snapshot);
  EXPECT_EQ(snapshot->planning_cycle_id, 4U);
  EXPECT_EQ(snapshot->solve_generation, 4U);
  EXPECT_EQ(snapshot->execution_goal_epoch, 4U);
  EXPECT_EQ(snapshot->execution_bundle_generation, 1004U);
  EXPECT_EQ(snapshot->execution_state_source_stamp_ns, 80);
  EXPECT_EQ(snapshot->execution_state_receive_stamp_ns, 120);
  EXPECT_EQ(snapshot->measured_position_at_state_source.x(), 4.0);
  EXPECT_EQ(snapshot->measured_position_at_state_source.y(), 5.0);
  EXPECT_EQ(snapshot->measured_position_at_state_source.z(), 6.0);

  EXPECT_FALSE(store.publish(record(3U)));
  EXPECT_EQ(store.load()->solve_generation, 4U);
}

TEST(ExecutionTraceStore, DoesNotMatchAReplacementExecutionOwner) {
  ExecutionTraceStore store;
  ASSERT_TRUE(store.publish(record(8U)));
  const auto snapshot = store.load();
  ASSERT_TRUE(snapshot);

  EXPECT_TRUE(executionTraceMatchesCommand(*snapshot, 7U, 8U, 108U, 1008U));
  EXPECT_FALSE(executionTraceMatchesCommand(*snapshot, 7U, 9U, 109U, 1009U));
  EXPECT_FALSE(executionTraceMatchesCommand(*snapshot, 8U, 8U, 108U, 1008U));
  EXPECT_FALSE(executionTraceMatchesCommand(*snapshot, 7U, 8U, 108U, 1009U));
  EXPECT_FALSE(executionTraceMatchesCommand(*snapshot, 7U, 8U, 0U, 1008U));
}

TEST(ExecutionTraceStore, RejectsAStaleLocalizationEpochAfterReset) {
  ExecutionTraceStore store;
  auto initial = record(8U);
  initial.execution_localization_epoch = 8U;
  ASSERT_TRUE(store.publish(std::move(initial)));

  store.advanceLocalizationEpoch(8U);
  EXPECT_TRUE(store.load());
  store.advanceLocalizationEpoch(9U);
  EXPECT_FALSE(store.load());

  auto stale = record(9U);
  stale.execution_localization_epoch = 8U;
  EXPECT_FALSE(store.publish(std::move(stale)));
  auto current = record(10U);
  current.execution_localization_epoch = 9U;
  EXPECT_TRUE(store.publish(std::move(current)));
}

TEST(ExecutionTraceStore, ConcurrentReadersNeverObserveTornVectorOrIdentity) {
  ExecutionTraceStore store;
  ASSERT_TRUE(store.publish(record(1U)));
  std::atomic_bool writer_ok{true};
  std::atomic_bool reader_ok{true};
  std::atomic_uint64_t read_count{0U};
  // Pair every publication with a load so this test cannot pass after the
  // reader misses the entire writer run. Neither worker uses an assertion:
  // returning early would strand the other worker at the next barrier.
  std::barrier operation_start(2);
  std::barrier operation_end(2);

  std::thread writer([&] {
    for (std::uint64_t generation = 2U; generation <= 500U; ++generation) {
      operation_start.arrive_and_wait();
      if (!store.publish(record(generation))) {
        writer_ok.store(false, std::memory_order_release);
      }
      operation_end.arrive_and_wait();
    }
  });
  std::thread reader([&] {
    for (std::uint64_t generation = 2U; generation <= 500U; ++generation) {
      operation_start.arrive_and_wait();
      const auto snapshot = store.load();
      read_count.fetch_add(1U, std::memory_order_relaxed);
      if (!snapshot) {
        reader_ok.store(false, std::memory_order_release);
      } else {
        const auto observed_generation = snapshot->solve_generation;
        if (snapshot->planning_cycle_id != observed_generation ||
            snapshot->execution_goal_epoch != observed_generation ||
            snapshot->execution_bundle_generation != observed_generation + 1000U ||
            snapshot->execution_state_source_stamp_ns !=
                static_cast<std::int64_t>(observed_generation * 20U) ||
            snapshot->execution_state_receive_stamp_ns !=
                static_cast<std::int64_t>(observed_generation * 30U) ||
            snapshot->measured_position_at_state_source.x() !=
                static_cast<double>(observed_generation) ||
            snapshot->measured_position_at_state_source.y() !=
                static_cast<double>(observed_generation + 1U) ||
            snapshot->measured_position_at_state_source.z() !=
                static_cast<double>(observed_generation + 2U)) {
          reader_ok.store(false, std::memory_order_release);
        }
      }
      operation_end.arrive_and_wait();
    }
  });
  writer.join();
  reader.join();

  EXPECT_TRUE(writer_ok.load(std::memory_order_acquire));
  EXPECT_TRUE(reader_ok.load(std::memory_order_acquire));
  EXPECT_EQ(read_count.load(std::memory_order_acquire), 499U);
  EXPECT_EQ(store.load()->solve_generation, 500U);
}

}  // namespace
}  // namespace navigation_runtime
