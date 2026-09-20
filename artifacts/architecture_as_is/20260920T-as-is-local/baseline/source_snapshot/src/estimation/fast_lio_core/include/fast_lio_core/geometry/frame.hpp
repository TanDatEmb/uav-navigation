#pragma once

#include <string>
#include <string_view>
#include <utility>

namespace uav::nav::lio {

class FrameId {
 public:
  FrameId() = default;
  explicit FrameId(std::string name) : name_(std::move(name)) {}

  [[nodiscard]] bool empty() const noexcept { return name_.empty(); }
  [[nodiscard]] std::string_view name() const noexcept { return name_; }

  friend bool operator==(const FrameId&, const FrameId&) = default;

 private:
  std::string name_;
};

}  // namespace uav::nav::lio
