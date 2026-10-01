#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "px4_navigation_external_mode/paired_node_lifetime.hpp"

namespace px4_navigation_external_mode {
namespace {

struct FakeMode {
  explicit FakeMode(std::atomic_int& destruction_count)
      : destruction_count(destruction_count) {}
  ~FakeMode() { ++destruction_count; }
  void callback() { ++callback_count; }
  std::atomic_int& destruction_count;
  std::atomic_int callback_count{0};
};

struct FakeState {};

TEST(PairedNodeLifetime, JoinsCallbackBeforeDestroyingModeOwner) {
  PairedNodeLifetime<FakeMode, FakeState> lifetime;
  std::atomic_int destruction_count{0};
  lifetime.mode = std::make_shared<FakeMode>(destruction_count);
  lifetime.state_input = std::make_shared<FakeState>();
  const std::weak_ptr<FakeMode> mode_weak = lifetime.mode;
  std::mutex mutex;
  std::condition_variable condition;
  bool callback_started = false;
  bool release_callback = false;
  bool callback_saw_live_mode = false;

  std::thread receiver([&] {
    {
      std::lock_guard lock(mutex);
      callback_started = true;
    }
    condition.notify_one();
    {
      std::unique_lock lock(mutex);
      condition.wait(lock, [&] { return release_callback; });
    }
    callback_saw_live_mode = !mode_weak.expired();
    lifetime.mode->callback();
  });

  {
    std::unique_lock lock(mutex);
    ASSERT_TRUE(condition.wait_for(
        lock, std::chrono::seconds(2), [&] { return callback_started; }));
    release_callback = true;
  }
  condition.notify_one();
  lifetime.joinReceiver(receiver);

  EXPECT_TRUE(callback_saw_live_mode);
  EXPECT_EQ(destruction_count.load(), 1);
  EXPECT_TRUE(lifetime.mode == nullptr);
  EXPECT_TRUE(lifetime.state_input == nullptr);
}

}  // namespace
}  // namespace px4_navigation_external_mode
