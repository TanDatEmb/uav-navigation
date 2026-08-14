#pragma once

#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "fast_lio_core/common/status.hpp"

namespace uav::nav::lio {

template <typename T>
class Result {
 public:
  Result(const T& value) : value_(value) {}
  Result(T&& value) : value_(std::move(value)) {}
  Result(Status status) : status_(std::move(status)) {
    if (status_.ok()) {
      throw std::invalid_argument("A value-less Result must contain an error");
    }
  }

  [[nodiscard]] bool ok() const noexcept { return status_.ok(); }
  [[nodiscard]] const Status& status() const noexcept { return status_; }

  [[nodiscard]] const T& value() const& {
    if (!value_) {
      throw std::logic_error(status_.message());
    }
    return *value_;
  }

  [[nodiscard]] T& value() & {
    if (!value_) {
      throw std::logic_error(status_.message());
    }
    return *value_;
  }

  [[nodiscard]] T&& value() && {
    if (!value_) {
      throw std::logic_error(status_.message());
    }
    return std::move(*value_);
  }

 private:
  Status status_;
  std::optional<T> value_;
};

template <>
class Result<void> {
 public:
  Result() = default;
  Result(Status status) : status_(std::move(status)) {
    if (status_.ok()) {
      throw std::invalid_argument("Use the default constructor for success");
    }
  }

  [[nodiscard]] bool ok() const noexcept { return status_.ok(); }
  [[nodiscard]] const Status& status() const noexcept { return status_; }

 private:
  Status status_;
};

}  // namespace uav::nav::lio
