#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <array>
#include <span>

#include "fast_lio_core/common/status.hpp"
#include "fast_lio_core/estimation/ikfom_estimator.hpp"
#include "fast_lio_core/pipeline/process_result.hpp"
#include "fast_lio_core/sensor/imu_sample.hpp"

namespace uav::nav::lio {

enum class PropagatedOdometryStatus {
  kWaitingForCorrection,
  kReady,
  kTimestampRegression,
  kImuGap,
  kMissingBracket,
  kInvalidState,
  kStaleCorrection,
  kMainEstimatorInvalid,
  kQueueOverflow,
};

struct ImuStatePropagatorConfig {
  IkfomEstimatorConfig ikfom{};
  ResidualBuilderConfig residual_builder{};
  std::int64_t imu_history_duration_ns{1'000'000'000};
};

struct ImuStatePropagatorDiagnostics {
  PropagatedOdometryStatus status{
      PropagatedOdometryStatus::kWaitingForCorrection};
  std::optional<Timestamp> anchor_time;
  std::optional<Timestamp> propagated_time;
  std::optional<Timestamp> latest_imu_time;
  std::size_t current_imu_history_size{0U};
  std::size_t maximum_imu_history_size{0U};
  std::uint64_t reanchor_count{0U};
  std::uint64_t replay_count{0U};
  std::size_t last_replay_sample_count{0U};
  std::uint64_t timestamp_regression_count{0U};
  std::uint64_t imu_gap_count{0U};
  std::uint64_t missing_bracket_count{0U};
  std::uint64_t invalid_state_count{0U};
};

class ImuStatePropagator {
 public:
  explicit ImuStatePropagator(ImuStatePropagatorConfig config);

  [[nodiscard]] Status acceptImu(const ImuSample& sample);
  [[nodiscard]] Status flushPendingPrediction();
  [[nodiscard]] Status reanchorAndReplay(const StateEstimate& corrected);
  void invalidate(PropagatedOdometryStatus status) noexcept;

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] std::optional<StateEstimate> estimate() const;
  [[nodiscard]] const ImuStatePropagatorDiagnostics& diagnostics() const noexcept;

 private:
  [[nodiscard]] Status validateState() const;
  [[nodiscard]] Status bracketHistory(const Timestamp& boundary,
                                      std::size_t& first_index) const;
  void pruneHistory();
  void setFailure(const Status& failure);

  ImuStatePropagatorConfig config_;
  IkfomEstimator estimator_;
  std::deque<ImuSample> history_;
  ImuStatePropagatorDiagnostics diagnostics_;
  bool valid_{false};
  std::optional<ImuSample> previous_imu_sample_;
  std::deque<ImuSample> pending_prediction_samples_;
};

[[nodiscard]] const char* toString(PropagatedOdometryStatus status) noexcept;

}  // namespace uav::nav::lio
