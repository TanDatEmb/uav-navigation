#pragma once

#include <string>
#include <utility>

namespace uav::nav::lio {

enum class StatusCode {
  kOk = 0,
  kInvalidArgument,
  kOutOfRange,
  kClockDomainMismatch,
  kTimestampRegression,
  kBufferFull,
  kNotReady,
  kInsufficientData,
  kNumericalFailure,
  kFrameMismatch,
  kInitializationRejected,
  kDeskewRejected,
  kScanOverlap,
  kImuGap,
  kMissingStartBracket,
  kMissingEndBracket,
  kTransportMessageLoss,
};

class Status {
 public:
  Status() = default;
  Status(StatusCode code, std::string message) : code_(code), message_(std::move(message)) {}

  [[nodiscard]] static Status Ok() { return {}; }

  [[nodiscard]] bool ok() const noexcept { return code_ == StatusCode::kOk; }
  [[nodiscard]] StatusCode code() const noexcept { return code_; }
  [[nodiscard]] const std::string& message() const noexcept { return message_; }

 private:
  StatusCode code_{StatusCode::kOk};
  std::string message_;
};

}  // namespace uav::nav::lio
