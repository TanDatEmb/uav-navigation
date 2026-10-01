#pragma once

#include <memory>
#include <thread>

namespace px4_navigation_external_mode {

// State-input callbacks capture NavigationMode's raw `this`. This owner is a
// small explicit lifetime barrier: receiver join must happen before either
// node owner is released, including exception/setup paths.
template <typename ModeNode, typename StateNode>
class PairedNodeLifetime final {
 public:
  std::shared_ptr<ModeNode> mode;
  std::shared_ptr<StateNode> state_input;

  PairedNodeLifetime() = default;
  PairedNodeLifetime(const PairedNodeLifetime&) = delete;
  PairedNodeLifetime& operator=(const PairedNodeLifetime&) = delete;

  void joinReceiver(std::thread& receiver) noexcept {
    if (receiver.joinable()) receiver.join();
    state_input.reset();
    mode.reset();
  }
};

}  // namespace px4_navigation_external_mode
