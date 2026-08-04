#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <navigation_interfaces/msg/odometry_supervisor_status.hpp>
#include <navigation_interfaces/srv/sample_odometry_at_time.hpp>
#include <navigation_interfaces/odometry_validity.hpp>

#include "odometry_supervisor/diagnostic_adapter.hpp"
#include "odometry_supervisor/alignment_lifecycle_manager.hpp"
#include "odometry_supervisor/lio_lifecycle_coordinator.hpp"
#include "odometry_supervisor/odometry_alignment_estimator.hpp"
#include "odometry_supervisor/residual_calculator.hpp"
#include "odometry_supervisor/supervisor_state_machine.hpp"
#include "odometry_supervisor/world_alignment.hpp"
#include "odometry_supervisor/query_epoch_eligibility.hpp"

namespace odometry_supervisor {
namespace {
using StatusMessage = navigation_interfaces::msg::OdometrySupervisorStatus;
using QueryService = navigation_interfaces::srv::SampleOdometryAtTime;
constexpr char kLioOdomFrame[] = "lio_odom";
constexpr char kPx4OdomFrame[] = "px4_odom";
constexpr char kBaseLinkFrame[] = "base_link";

std::int64_t time_ns(const builtin_interfaces::msg::Time& time) {
  return static_cast<std::int64_t>(time.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(time.nanosec);
}

builtin_interfaces::msg::Time ros_time(std::int64_t timestamp_ns) {
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(timestamp_ns / 1'000'000'000LL);
  result.nanosec = static_cast<std::uint32_t>(timestamp_ns % 1'000'000'000LL);
  return result;
}

OdometryState state_from_ros(const nav_msgs::msg::Odometry& message) {
  OdometryState state;
  state.timestamp_ns = time_ns(message.header.stamp);
  state.position_odom = Eigen::Vector3d(message.pose.pose.position.x, message.pose.pose.position.y,
                                        message.pose.pose.position.z);
  state.orientation_odom_base = Eigen::Quaterniond(
      message.pose.pose.orientation.w, message.pose.pose.orientation.x,
      message.pose.pose.orientation.y, message.pose.pose.orientation.z);
  state.velocity_base = Eigen::Vector3d(message.twist.twist.linear.x,
                                        message.twist.twist.linear.y,
                                        message.twist.twist.linear.z);
  state.frame_id = message.header.frame_id;
  state.child_frame_id = message.child_frame_id;
  state.valid = true;
  return state;
}

const char* health_name(HealthState state) {
  switch (state) {
    case HealthState::kStartup: return "STARTUP";
    case HealthState::kHealthy: return "HEALTHY";
    case HealthState::kSuspect: return "SUSPECT";
    case HealthState::kDegraded: return "DEGRADED";
    case HealthState::kDiverged: return "DIVERGED";
  }
  return "UNKNOWN";
}

const char* lifecycle_name(LioLifecycleState state) {
  switch (state) {
    case LioLifecycleState::kStartup: return "STARTUP";
    case LioLifecycleState::kTracking: return "TRACKING";
    case LioLifecycleState::kResetting: return "RESETTING";
    case LioLifecycleState::kLost: return "LOST";
    case LioLifecycleState::kReinitializing: return "REINITIALIZING";
  }
  return "UNKNOWN";
}

const char* alignment_lifecycle_name(AlignmentLifecycleState state) {
  switch (state) {
    case AlignmentLifecycleState::kUnaligned: return "UNALIGNED";
    case AlignmentLifecycleState::kCollecting: return "COLLECTING";
    case AlignmentLifecycleState::kProvisional: return "PROVISIONAL";
    case AlignmentLifecycleState::kLocked: return "LOCKED";
    case AlignmentLifecycleState::kRevalidating: return "REVALIDATING";
    case AlignmentLifecycleState::kInvalid: return "INVALID";
  }
  return "UNKNOWN";
}

}  // namespace

class OdometrySupervisorNode final : public rclcpp::Node {
 public:
  OdometrySupervisorNode() : Node("odometry_supervisor") {
    config_ = load_config();
    machine_ = SupervisorStateMachine(config_);
    alignment_estimator_ = OdometryAlignmentEstimator({
        .window_size = config_.alignment_window_size,
        .minimum_samples = config_.alignment_minimum_samples,
        .minimum_horizontal_excitation_m =
            config_.alignment_minimum_horizontal_excitation_m});
    alignment_manager_ = AlignmentLifecycleManager({
        .stable_candidate_estimates = config_.alignment_lock_stable_windows,
        .minimum_novel_pairs = config_.alignment_minimum_novel_pairs,
        .candidate_history_capacity = config_.alignment_candidate_history_capacity,
        .max_translation_step_m = config_.alignment_lock_max_translation_step_m,
        .max_yaw_step_rad = config_.alignment_lock_max_yaw_step_rad,
        .max_cluster_translation_m = config_.alignment_max_cluster_translation_m,
        .max_cluster_yaw_rad = config_.alignment_max_cluster_yaw_rad,
        .covariance_nis_chi_square = config_.alignment_covariance_nis_chi_square,
        .revalidation_samples = config_.alignment_revalidation_samples,
        .revalidation_failure_limit = config_.alignment_revalidation_failure_limit,
        .revalidation_residual = config_.degraded,
        .revalidation_covariance_nis_chi_square = config_.alignment_covariance_nis_chi_square});
    status_ = create_publisher<StatusMessage>(
        "/navigation/odometry_supervisor/status", rclcpp::QoS(1).reliable().transient_local());
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/navigation/odometry_supervisor/diagnostics", rclcpp::QoS(10).reliable());
    propagated_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lio/odometry_propagated", rclcpp::QoS(20).reliable(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          const auto state = state_from_ros(*message);
          if (!ResidualCalculator::valid(state) || state.frame_id != kLioOdomFrame) return;
          if (!propagated_history_.empty() &&
              state.timestamp_ns <= propagated_history_.back().timestamp_ns) {
            propagated_history_.clear();
          }
          propagated_history_.push_back(state);
          while (propagated_history_.size() > kPropagatedHistoryCapacity) {
            propagated_history_.pop_front();
          }
          latest_propagated_ = state;
        });
    corrected_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lio/odometry_corrected", rclcpp::QoS(20).reliable(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          const auto state = state_from_ros(*message);
          if (ResidualCalculator::valid(state) && state.frame_id == kLioOdomFrame) {
            latest_corrected_ = state;
          }
        });
    px4_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/px4/estimator_odometry", rclcpp::QoS(20).reliable(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          const auto state = state_from_ros(*message);
          if (ResidualCalculator::valid(state) && state.frame_id == kPx4OdomFrame) {
            if (!px4_history_.empty() &&
                state.timestamp_ns <= px4_history_.back().timestamp_ns) {
              px4_history_.clear();
            }
            px4_history_.push_back(state);
            while (px4_history_.size() > kPx4HistoryCapacity) px4_history_.pop_front();
            latest_px4_ = state;
          }
        });
    lio_diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/lio/diagnostics", rclcpp::QoS(10).reliable(),
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
          const auto snapshot = selectDiagnostic(*message, "fast_lio/estimator");
          // FAST-LIO emits estimator, transport, and propagated-worker
          // diagnostics as separate DiagnosticArray messages on this topic.
          // A non-estimator array must not erase the last valid estimator
          // snapshot and create a false schema mismatch.
          if (snapshot.found) lio_diagnostics_ = snapshot;
        });
    px4_diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/px4/diagnostics", rclcpp::QoS(10).reliable(),
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
          const auto snapshot = selectDiagnostic(*message, "px4_odometry_bridge");
          if (snapshot.found) px4_diagnostics_ = snapshot;
        });
    external_diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/px4/external_odometry_diagnostics", rclcpp::QoS(10).reliable(),
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
          const auto snapshot = selectDiagnostic(*message, "px4_external_odometry_bridge");
          if (snapshot.found) external_diagnostics_ = snapshot;
        });
    query_client_ = create_client<QueryService>("/px4/sample_odometry_at_time");
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / config_.evaluation_rate_hz));
    evaluation_timer_ = create_timer(std::max(period, std::chrono::milliseconds(1)),
                                     [this]() { evaluate_once(); });
  }

 private:
  SupervisorConfig load_config() {
    SupervisorConfig config;
    const auto reference = declare_parameter<std::string>("reference_mode", "independent");
    if (reference == "independent") {
      config.reference_mode = ReferenceMode::kIndependent;
    } else if (reference == "correlated") {
      config.reference_mode = ReferenceMode::kCorrelated;
    } else {
      throw std::invalid_argument("unknown reference_mode");
    }
    config.evaluation_rate_hz = declare_parameter("evaluation_rate_hz", config.evaluation_rate_hz);
    config.propagated_max_age_ns = declare_parameter("freshness.propagated_max_age_ns",
                                                      config.propagated_max_age_ns);
    config.corrected_max_age_ns = declare_parameter("freshness.corrected_max_age_ns",
                                                     config.corrected_max_age_ns);
    config.px4_max_age_ns = declare_parameter("freshness.px4_max_age_ns", config.px4_max_age_ns);
    config.diagnostics_max_age_ns = declare_parameter("freshness.diagnostics_max_age_ns",
                                                       config.diagnostics_max_age_ns);
    config.maximum_alignment_gap_ns = declare_parameter(
        "alignment.maximum_gap_ns", config.maximum_alignment_gap_ns);
    config.service_timeout_ns = declare_parameter("alignment.service_timeout_ns",
                                                  config.service_timeout_ns);
    config.maximum_comparison_age_ns = declare_parameter(
        "alignment.maximum_comparison_age_ns", config.maximum_comparison_age_ns);
    config.query_epoch_max_age_ns = declare_parameter(
        "alignment.query_epoch_max_age_ns", config.query_epoch_max_age_ns);
    config.authoritative_yaw = declare_parameter("alignment.authoritative_yaw", false);
    config.alignment_window_size = static_cast<std::size_t>(declare_parameter<std::int64_t>(
        "alignment.window_size", static_cast<std::int64_t>(config.alignment_window_size)));
    config.alignment_minimum_samples = static_cast<std::size_t>(declare_parameter<std::int64_t>(
        "alignment.minimum_samples", static_cast<std::int64_t>(config.alignment_minimum_samples)));
    config.alignment_lock_stable_windows = static_cast<std::size_t>(declare_parameter<std::int64_t>(
        "alignment.lock_stable_windows",
        static_cast<std::int64_t>(config.alignment_lock_stable_windows)));
    config.alignment_lock_max_translation_step_m = declare_parameter(
        "alignment.lock_max_translation_step_m", config.alignment_lock_max_translation_step_m);
    config.alignment_lock_max_yaw_step_rad = declare_parameter(
        "alignment.lock_max_yaw_step_rad", config.alignment_lock_max_yaw_step_rad);
    config.alignment_minimum_novel_pairs = static_cast<std::size_t>(declare_parameter<std::int64_t>(
        "alignment.minimum_novel_pairs", static_cast<std::int64_t>(config.alignment_minimum_novel_pairs)));
    config.alignment_candidate_history_capacity = static_cast<std::size_t>(declare_parameter<std::int64_t>(
        "alignment.candidate_history_capacity",
        static_cast<std::int64_t>(config.alignment_candidate_history_capacity)));
    config.alignment_max_cluster_translation_m = declare_parameter(
        "alignment.max_cluster_translation_m", config.alignment_max_cluster_translation_m);
    config.alignment_max_cluster_yaw_rad = declare_parameter(
        "alignment.max_cluster_yaw_rad", config.alignment_max_cluster_yaw_rad);
    config.alignment_covariance_nis_chi_square = declare_parameter(
        "alignment.covariance_nis_chi_square", config.alignment_covariance_nis_chi_square);
    config.alignment_revalidation_samples = static_cast<std::size_t>(declare_parameter<std::int64_t>(
        "alignment.revalidation_samples", static_cast<std::int64_t>(config.alignment_revalidation_samples)));
    config.alignment_revalidation_failure_limit = static_cast<std::size_t>(declare_parameter<std::int64_t>(
        "alignment.revalidation_failure_limit",
        static_cast<std::int64_t>(config.alignment_revalidation_failure_limit)));
    config.alignment_minimum_horizontal_excitation_m = declare_parameter(
        "alignment.minimum_horizontal_excitation_m",
        config.alignment_minimum_horizontal_excitation_m);
    config.clear_ratio = declare_parameter("hysteresis.clear_ratio", config.clear_ratio);
    config.suspect_enter_ns = declare_parameter("persistence.suspect_enter_ns",
                                                config.suspect_enter_ns);
    config.degraded_enter_ns = declare_parameter("persistence.degraded_enter_ns",
                                                 config.degraded_enter_ns);
    config.diverged_enter_ns = declare_parameter("persistence.diverged_enter_ns",
                                                 config.diverged_enter_ns);
    config.recovery_confirm_ns = declare_parameter("persistence.recovery_confirm_ns",
                                                   config.recovery_confirm_ns);
    config.reset_grace_ns = declare_parameter("persistence.reset_grace_ns", config.reset_grace_ns);
    config.lio_diagnostics_invalid_enter_ns = declare_parameter(
        "persistence.lio_diagnostics_invalid_enter_ns", config.lio_diagnostics_invalid_enter_ns);
    config.suspect_speed_limit_m_s = declare_parameter(
        "actions.suspect_speed_limit_m_s", config.suspect_speed_limit_m_s);
    config.degraded_speed_limit_m_s = declare_parameter(
        "actions.degraded_speed_limit_m_s", config.degraded_speed_limit_m_s);
    config.suspect.position_m = declare_parameter("thresholds.suspect.position_m",
                                                  config.suspect.position_m);
    config.suspect.velocity_m_s = declare_parameter("thresholds.suspect.velocity_m_s",
                                                    config.suspect.velocity_m_s);
    config.suspect.orientation_rad = declare_parameter("thresholds.suspect.orientation_rad",
                                                       config.suspect.orientation_rad);
    config.suspect.yaw_rad = declare_parameter("thresholds.suspect.yaw_rad",
                                                config.suspect.yaw_rad);
    config.degraded.position_m = declare_parameter("thresholds.degraded.position_m",
                                                   config.degraded.position_m);
    config.degraded.velocity_m_s = declare_parameter("thresholds.degraded.velocity_m_s",
                                                     config.degraded.velocity_m_s);
    config.degraded.orientation_rad = declare_parameter("thresholds.degraded.orientation_rad",
                                                        config.degraded.orientation_rad);
    config.degraded.yaw_rad = declare_parameter("thresholds.degraded.yaw_rad",
                                                 config.degraded.yaw_rad);
    config.diverged.position_m = declare_parameter("thresholds.diverged.position_m",
                                                   config.diverged.position_m);
    config.diverged.velocity_m_s = declare_parameter("thresholds.diverged.velocity_m_s",
                                                     config.diverged.velocity_m_s);
    config.diverged.orientation_rad = declare_parameter("thresholds.diverged.orientation_rad",
                                                        config.diverged.orientation_rad);
    config.diverged.yaw_rad = declare_parameter("thresholds.diverged.yaw_rad",
                                                 config.diverged.yaw_rad);
    return config;
  }

  bool fresh(const std::optional<OdometryState>& state, std::int64_t max_age,
             std::int64_t now_ns, std::int64_t* age) const {
    if (!state || !ResidualCalculator::valid(*state) || state->timestamp_ns > now_ns) {
      if (age) *age = -1;
      return false;
    }
    *age = now_ns - state->timestamp_ns;
    return *age <= max_age;
  }

  std::optional<OdometryState> latest_propagated_epoch_available_to_px4() const {
    return odometry_supervisor::latest_propagated_epoch_available_to_px4(
               propagated_history_, latest_px4_)
        .state;
  }

  struct PendingQuery {
    std::uint64_t sequence{0};
    QueryEpochKey key;
    std::int64_t requested_epoch_ns{0};
    std::uint64_t expected_reset_generation{0};
    std::uint64_t expected_frame_generation{0};
    std::uint64_t expected_time_generation{0};
    OdometryState lio;
    WorldAlignment alignment;
    std::chrono::steady_clock::time_point started{};
  };

  struct PendingAlignmentQuery {
    std::uint64_t sequence{0};
    QueryEpochKey key;
    std::int64_t requested_epoch_ns{0};
    std::uint64_t expected_lio_generation{0};
    std::uint64_t expected_reset_generation{0};
    std::uint64_t expected_frame_generation{0};
    std::uint64_t expected_time_generation{0};
    OdometryState lio;
    std::chrono::steady_clock::time_point started{};
  };

  void set_query_failure(const char* failure_class, const std::string& reason) {
    query_failure_class_ = failure_class;
    query_last_failure_reason_ = reason;
  }

  void suppress_query_epoch(const QueryEpochKey& key) {
    last_query_key_ = key;
    suppressed_query_key_ = key;
  }

  void clear_query_suppression() {
    last_query_key_.reset();
    suppressed_query_key_.reset();
    service_retry_key_.reset();
    service_retry_attempts_ = 0;
    last_expired_query_key_.reset();
    last_not_yet_buffered_epoch_.reset();
  }

  bool prepare_query_epoch(const QueryEpochKey& key) {
    if (pending_query_ || pending_alignment_query_) return false;
    if ((suppressed_query_key_ && *suppressed_query_key_ == key) ||
        (last_query_key_ && *last_query_key_ == key)) {
      ++query_duplicate_suppressed_count_;
      set_query_failure("EPOCH_ELIGIBILITY", "query epoch already submitted or suppressed");
      return false;
    }
    if (!query_client_->service_is_ready()) {
      ++query_service_unavailable_count_;
      ++query_transport_failure_count_;
      set_query_failure("TRANSPORT_FAILURE", "PX4 sampling service unavailable");
      if (!service_retry_key_ || *service_retry_key_ != key) {
        service_retry_key_ = key;
        service_retry_attempts_ = 0;
      }
      const auto now_steady = std::chrono::steady_clock::now();
      if (now_steady < next_service_retry_) return false;
      ++service_retry_attempts_;
      if (service_retry_attempts_ >= kMaxServiceRetryAttempts) {
        suppress_query_epoch(key);
        ++query_failure_count_;
        query_last_failure_reason_ = "PX4 sampling service retry budget exhausted";
      } else {
        next_service_retry_ = now_steady +
                              std::chrono::milliseconds(10U << (service_retry_attempts_ - 1U));
      }
      return false;
    }
    service_retry_key_.reset();
    service_retry_attempts_ = 0;
    last_query_key_ = key;
    return true;
  }

  void record_transport_failure(const QueryEpochKey& key, const std::string& reason) {
    suppress_query_epoch(key);
    ++query_transport_failure_count_;
    set_query_failure("TRANSPORT_FAILURE", reason);
    invalidate_comparison();
    alignment_manager_.observeTransportFailure(reason);
  }

  void record_contract_failure(const QueryEpochKey& key, const std::string& reason) {
    suppress_query_epoch(key);
    ++query_failure_count_;
    set_query_failure("GENERATION_CONTRACT_FAILURE", reason);
    invalidate_comparison();
    alignment_manager_.observeTransportFailure(reason);
  }

  void record_geometric_failure(const QueryEpochKey& key, const std::string& reason) {
    suppress_query_epoch(key);
    ++query_geometric_failure_count_;
    ++query_failure_count_;
    set_query_failure("GEOMETRIC_FAILURE", reason);
    alignment_manager_.beginRevalidation(reason, key.epoch_ns);
    invalidate_comparison();
  }

  static AlignmentRevalidationObservation invalid_revalidation(
      const QueryEpochKey& key, std::uint64_t evidence_id) {
    AlignmentRevalidationObservation observation;
    observation.exact_time_pair_valid = false;
    observation.epoch_ns = key.epoch_ns;
    observation.lio_generation = key.lio_generation;
    observation.frame_generation = key.frame_generation;
    observation.time_generation = key.time_generation;
    observation.evidence_id = evidence_id;
    return observation;
  }

  void record_query_result(const PendingQuery& pending, bool success) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
        std::chrono::steady_clock::now() - pending.started).count();
    if (success) {
      ++query_success_count_;
    } else {
      ++query_failure_count_;
    }
    ++query_rtt_count_;
    query_rtt_max_ms_ = std::max(query_rtt_max_ms_, elapsed);
    constexpr std::array<double, 15> upper_bounds{
        0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0,
        40.0, 80.0, 160.0, 320.0, 640.0, 1280.0, 2560.0};
    std::size_t bin = upper_bounds.size();
    for (std::size_t index = 0; index < upper_bounds.size(); ++index) {
      if (elapsed <= upper_bounds[index]) { bin = index; break; }
    }
    ++query_rtt_histogram_[bin];
  }

  double query_rtt_percentile(double quantile) const {
    if (query_rtt_count_ == 0) return 0.0;
    const auto target = std::max<std::uint64_t>(1, static_cast<std::uint64_t>(
        std::ceil(quantile * static_cast<double>(query_rtt_count_))));
    std::uint64_t cumulative = 0;
    constexpr std::array<double, 16> upper_bounds{
        0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0,
        40.0, 80.0, 160.0, 320.0, 640.0, 1280.0, 2560.0, 5120.0};
    for (std::size_t index = 0; index < query_rtt_histogram_.size(); ++index) {
      cumulative += query_rtt_histogram_[index];
      if (cumulative >= target) return upper_bounds[index];
    }
    return query_rtt_max_ms_;
  }

  void invalidate_comparison() {
    aligned_comparison_.reset();
    previous_residual_.reset();
    new_comparison_sample_pending_ = false;
  }

  void invalidate_alignment() {
    alignment_manager_.invalidate("alignment invalidated");
    invalidate_comparison();
    alignment_estimator_.reset();
    last_alignment_pair_epoch_ns_ = 0;
  }

  void update_alignment_if_ready(const EvaluationInput& input) {
    const auto manager_state = alignment_manager_.state();
    if (!input.lio_valid || !input.px4_available || !input.px4_fresh ||
        !input.px4_diagnostics_valid || !input.px4_continuity_valid ||
        !input.px4_post_reset_stable) {
      return;
    }
    if (manager_state == AlignmentLifecycleState::kLocked) {
      return;
    }
    if (manager_state == AlignmentLifecycleState::kUnaligned ||
        manager_state == AlignmentLifecycleState::kInvalid) {
      alignment_manager_.observeBindingGeneration(input.lio_generation,
                                                   input.px4_frame_generation,
                                                   input.px4_time_generation);
    }
    const auto eligibility = odometry_supervisor::latest_propagated_epoch_available_to_px4(
        propagated_history_, latest_px4_);
    if (!eligibility.state) {
      if (eligibility.reason == QueryEpochEligibilityReason::kLioEpochAheadOfPx4 &&
          latest_propagated_ &&
          (!last_not_yet_buffered_epoch_ ||
           *last_not_yet_buffered_epoch_ != latest_propagated_->timestamp_ns)) {
        last_not_yet_buffered_epoch_ = latest_propagated_->timestamp_ns;
        ++query_epoch_not_yet_buffered_count_;
        set_query_failure("EPOCH_ELIGIBILITY", "LIO epoch is not yet buffered by PX4");
      }
      return;
    }
    last_not_yet_buffered_epoch_.reset();
    const auto& lio = *eligibility.state;
    if (input.evaluation_time_ns > lio.timestamp_ns &&
        input.evaluation_time_ns - lio.timestamp_ns > config_.query_epoch_max_age_ns) {
      const auto key = query_epoch_key(lio, input.lio_generation,
                                       input.px4_frame_generation, input.px4_time_generation);
      if (!last_expired_query_key_ || *last_expired_query_key_ != key) {
        last_expired_query_key_ = key;
        ++query_epoch_expired_count_;
        set_query_failure("EPOCH_ELIGIBILITY", "eligible LIO epoch expired before submission");
      }
      suppress_query_epoch(key);
      return;
    }
    if (lio.timestamp_ns > last_alignment_pair_epoch_ns_) {
      request_alignment_at(lio, input.lio_generation, input.px4_reset_generation,
                           input.px4_frame_generation, input.px4_time_generation);
    }
  }

  void request_alignment_at(const OdometryState& lio,
                            std::uint64_t expected_lio_generation,
                            std::uint64_t expected_reset_generation,
                            std::uint64_t expected_frame_generation,
                            std::uint64_t expected_time_generation) {
    const auto key = query_epoch_key(lio, expected_lio_generation,
                                     expected_frame_generation, expected_time_generation);
    if (!prepare_query_epoch(key) ||
        alignment_manager_.state() == AlignmentLifecycleState::kLocked) return;
    auto request = std::make_shared<QueryService::Request>();
    request->sample_time = ros_time(lio.timestamp_ns);
    const auto sequence = ++query_sequence_;
    pending_alignment_query_ = PendingAlignmentQuery{
        sequence, key, lio.timestamp_ns, expected_lio_generation,
        expected_reset_generation, expected_frame_generation, expected_time_generation, lio,
        std::chrono::steady_clock::now()};
    query_client_->async_send_request(
        request, [this, sequence](rclcpp::Client<QueryService>::SharedFuture future) {
          if (!pending_alignment_query_ || pending_alignment_query_->sequence != sequence) {
            ++query_stale_sequence_count_;
            return;
          }
          const auto pending = *pending_alignment_query_;
          pending_alignment_query_.reset();
          PendingQuery timing_query;
          timing_query.sequence = pending.sequence;
          timing_query.key = pending.key;
          timing_query.requested_epoch_ns = pending.requested_epoch_ns;
          timing_query.started = pending.started;
          QueryService::Response::SharedPtr response;
          try {
            response = future.get();
          } catch (const std::exception&) {
            record_query_result(timing_query, false);
            record_transport_failure(pending.key, "alignment query transport exception");
            return;
          }
          if (!response || !response->success ||
              time_ns(response->odometry.header.stamp) != pending.requested_epoch_ns ||
              response->odometry.header.frame_id != kPx4OdomFrame ||
              response->odometry.child_frame_id != kBaseLinkFrame) {
            record_query_result(timing_query, false);
            alignment_rejection_reason_ = "exact-time PX4 alignment query rejected";
            record_contract_failure(pending.key, alignment_rejection_reason_);
            return;
          }
          record_query_result(timing_query, true);
          constexpr auto required_components = navigation_interfaces::kPositionValid |
                                               navigation_interfaces::kOrientationValid;
          if ((response->component_validity_mask & required_components) != required_components) {
            ++query_invalid_component_count_;
            alignment_rejection_reason_ = "PX4 alignment response component validity failed";
            record_contract_failure(pending.key, alignment_rejection_reason_);
            return;
          }
          const auto current_lio_generation = lio_diagnostics_.unsignedInteger("lio_generation");
          const auto current_reset_generation = px4_diagnostics_.unsignedInteger("reset_generation");
          const auto current_frame_generation = px4_diagnostics_.unsignedInteger("frame_generation");
          const auto current_time_generation = px4_diagnostics_.unsignedInteger("time_generation");
          if (pending.expected_lio_generation != current_lio_generation ||
              pending.expected_reset_generation != current_reset_generation ||
              pending.expected_frame_generation != current_frame_generation ||
              pending.expected_time_generation != current_time_generation ||
              response->reset_generation != pending.expected_reset_generation ||
              response->frame_generation != pending.expected_frame_generation ||
              response->time_generation != pending.expected_time_generation) {
            ++query_generation_mismatch_count_;
            alignment_rejection_reason_ = "alignment query generation mismatch";
            record_contract_failure(pending.key, alignment_rejection_reason_);
            return;
          }
          last_alignment_pair_epoch_ns_ = pending.requested_epoch_ns;
          const auto px4 = state_from_ros(response->odometry);
          if (alignment_manager_.revalidating()) {
            const auto frozen = alignment_manager_.lockedAlignment();
            const auto aligned_px4 = frozen ? applyWorldAlignment(px4, *frozen) : std::nullopt;
            const auto residual = aligned_px4
                                      ? ResidualCalculator::compare(pending.lio, *aligned_px4,
                                                                    previous_residual_)
                                      : std::nullopt;
            if (!residual) {
              ++query_geometric_failure_count_;
              ++query_failure_count_;
              set_query_failure("GEOMETRIC_FAILURE", "frozen alignment residual unavailable");
              alignment_manager_.observeRevalidation(
                  invalid_revalidation(pending.key, pending.sequence));
              invalidate_comparison();
            } else {
              AlignmentRevalidationObservation observation;
              observation.exact_time_pair_valid = true;
              observation.residual = *residual;
              observation.epoch_ns = pending.requested_epoch_ns;
              observation.lio_generation = pending.expected_lio_generation;
              observation.frame_generation = pending.expected_frame_generation;
              observation.time_generation = pending.expected_time_generation;
              observation.evidence_id = pending.sequence;
              alignment_manager_.observeRevalidation(observation);
            }
            return;
          }
          AlignmentSample sample;
          sample.timestamp_ns = pending.requested_epoch_ns;
          sample.lio_position = pending.lio.position_odom;
          sample.lio_orientation = pending.lio.orientation_odom_base;
          sample.px4_position = px4.position_odom;
          sample.px4_orientation = px4.orientation_odom_base;
          sample.lio_generation = pending.expected_lio_generation;
          sample.px4_reset_generation = pending.expected_reset_generation;
          sample.px4_frame_generation = pending.expected_frame_generation;
          sample.px4_time_generation = pending.expected_time_generation;
          sample.lio_tracking = lio_diagnostics_.string("status") == "TRACKING" &&
                                lio_diagnostics_.boolean("navigation_valid");
          sample.px4_continuity_valid = px4_diagnostics_.boolean("continuity_valid");
          sample.yaw_authoritative = config_.authoritative_yaw;
          sample.weight = 1.0;
          if (!alignment_estimator_.addSample(std::move(sample))) {
            alignment_rejection_reason_ = "alignment sample validation failed";
            alignment_manager_.rejectCandidate(alignment_rejection_reason_);
            return;
          }
          const auto estimate = alignment_estimator_.estimate();
          last_alignment_estimate_ = estimate;
          alignment_rejection_reason_ = estimate.rejection_reason;
          if (!estimate.valid()) {
            alignment_manager_.rejectCandidate(alignment_rejection_reason_);
            return;
          }
          auto candidate = estimate.alignment;
          candidate.reinitialization_count = alignment_reinitialization_count_ + 1U;
          candidate.source = "px4.sample_odometry_at_time.4dof.circular_mean";
          const auto before_lock_count = alignment_manager_.snapshot().lock_count;
          alignment_manager_.observeCandidate(
              {candidate, pending.expected_lio_generation, pending.expected_frame_generation,
               pending.expected_time_generation, pending.sequence, 1});
          const auto after = alignment_manager_.snapshot();
          alignment_rejection_reason_ = after.rejection_reason;
          if (after.lock_count > before_lock_count) {
            ++alignment_reinitialization_count_;
          }
        });
  }

  void request_px4_at(const OdometryState& lio, std::uint64_t expected_reset_generation,
                      std::uint64_t expected_frame_generation,
                      std::uint64_t expected_time_generation,
                      const WorldAlignment& alignment) {
    if (!alignment.valid) return;
    const auto key = query_epoch_key(lio, alignment.lio_generation,
                                     expected_frame_generation, expected_time_generation);
    if (!prepare_query_epoch(key)) return;
    auto request = std::make_shared<QueryService::Request>();
    request->sample_time = ros_time(lio.timestamp_ns);
    const auto sequence = ++query_sequence_;
    pending_query_ = PendingQuery{sequence, key, lio.timestamp_ns, expected_reset_generation,
                                  expected_frame_generation, expected_time_generation, lio, alignment,
                                  std::chrono::steady_clock::now()};
    query_client_->async_send_request(
        request, [this, sequence](
                     rclcpp::Client<QueryService>::SharedFuture future) {
          if (!pending_query_ || pending_query_->sequence != sequence) {
            ++query_stale_sequence_count_;
            return;
          }
          const auto pending = *pending_query_;
          pending_query_.reset();
          QueryService::Response::SharedPtr response;
          try {
            response = future.get();
          } catch (const std::exception&) {
            record_query_result(pending, false);
            record_transport_failure(pending.key, "comparison query transport exception");
            return;
          }
          if (!response || !response->success ||
              time_ns(response->odometry.header.stamp) != pending.requested_epoch_ns ||
              response->odometry.header.frame_id != kPx4OdomFrame ||
              response->odometry.child_frame_id != kBaseLinkFrame) {
            record_query_result(pending, false);
            record_contract_failure(pending.key, "comparison exact-time query rejected");
            return;
          }
          record_query_result(pending, true);
          constexpr auto required_components = navigation_interfaces::kPositionValid |
                                               navigation_interfaces::kOrientationValid |
                                               navigation_interfaces::kLinearVelocityValid;
          if ((response->component_validity_mask & required_components) != required_components) {
            ++query_failure_count_;
            ++query_invalid_component_count_;
            record_contract_failure(pending.key, "comparison component validity failed");
            return;
          }
          const auto current_reset_generation =
              px4_diagnostics_.unsignedInteger("reset_generation");
          const auto current_frame_generation =
              px4_diagnostics_.unsignedInteger("frame_generation");
          const auto current_time_generation =
              px4_diagnostics_.unsignedInteger("time_generation");
          if (response->reset_generation != pending.expected_reset_generation ||
              response->frame_generation != pending.expected_frame_generation ||
              response->time_generation != pending.expected_time_generation ||
              response->reset_generation != current_reset_generation ||
              response->frame_generation != current_frame_generation ||
              response->time_generation != current_time_generation) {
            ++query_generation_mismatch_count_;
            record_contract_failure(pending.key, "comparison query generation mismatch");
            return;
          }
          const auto locked_alignment = alignment_manager_.lockedAlignment();
          if (!locked_alignment || !locked_alignment->valid ||
              locked_alignment->frame_generation != pending.alignment.frame_generation ||
              locked_alignment->time_generation != pending.alignment.time_generation) {
            record_contract_failure(pending.key, "locked alignment binding changed");
            return;
          }
          const auto px4_source = state_from_ros(response->odometry);
          const auto px4 = applyWorldAlignment(px4_source, *locked_alignment);
          if (!px4) {
            record_geometric_failure(pending.key, "frozen alignment application failed");
            return;
          }
          const auto residual = ResidualCalculator::compare(pending.lio, *px4, previous_residual_);
          if (!residual) {
            record_geometric_failure(pending.key, "frozen alignment residual validation failed");
            return;
          }
          aligned_comparison_ = AlignedComparison{pending.lio,
                                                   *px4,
                                                   *residual,
                                                   pending.requested_epoch_ns,
                                                   now().nanoseconds(),
                                                   sequence,
                                                   response->reset_generation,
                                                   response->frame_generation,
                                                   response->time_generation,
                                                   response->component_validity_mask,
                                                   response->covariance_availability_mask,
                                                   response->interpolated,
                                                   *locked_alignment};
          previous_residual_ = *residual;
          new_comparison_sample_pending_ = true;
        });
  }

  bool aligned_comparison_fresh(std::int64_t now_ns) const {
    if (!aligned_comparison_) return false;
    if (now_ns < aligned_comparison_->comparison_epoch_ns ||
        now_ns - aligned_comparison_->comparison_epoch_ns >
            config_.maximum_comparison_age_ns) return false;
    return true;
  }

  void evaluate_once() {
    const auto now_ns = now().nanoseconds();
    // A simulated ROS clock is zero until the first /clock message.  Do not
    // start freshness or persistence timers in that pre-clock epoch.
    if (get_parameter("use_sim_time").as_bool() && now_ns == 0) return;
    if (pending_query_) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - pending_query_->started).count();
      if (elapsed > config_.service_timeout_ns) {
        const auto pending = *pending_query_;
        pending_query_.reset();
        ++query_timeout_count_;
        record_query_result(pending, false);
        record_transport_failure(pending.key, "comparison query timed out");
      }
    }
    if (pending_alignment_query_) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - pending_alignment_query_->started).count();
      if (elapsed > config_.service_timeout_ns) {
        const auto pending = *pending_alignment_query_;
        pending_alignment_query_.reset();
        ++query_timeout_count_;
        alignment_rejection_reason_ = "alignment query timed out";
        PendingQuery timing_query;
        timing_query.sequence = pending.sequence;
        timing_query.key = pending.key;
        timing_query.requested_epoch_ns = pending.requested_epoch_ns;
        timing_query.started = pending.started;
        record_query_result(timing_query, false);
        record_transport_failure(pending.key, alignment_rejection_reason_);
      }
    }
    std::int64_t propagated_age = -1;
    std::int64_t corrected_age = -1;
    std::int64_t px4_age = -1;
    const bool propagated_fresh = fresh(latest_propagated_, config_.propagated_max_age_ns,
                                        now_ns, &propagated_age);
    const bool corrected_fresh = fresh(latest_corrected_, config_.corrected_max_age_ns,
                                       now_ns, &corrected_age);
    const bool px4_fresh = fresh(latest_px4_, config_.px4_max_age_ns, now_ns, &px4_age);
    EvaluationInput input;
    input.evaluation_time_ns = now_ns;
    const auto lio_status = lio_diagnostics_.string("status");
    const auto lio_status_is = [&lio_status](const char* canonical, const char* legacy) {
      return lio_status == canonical || lio_status == legacy;
    };
    const bool lio_schema_valid = lio_diagnostics_.found && lio_diagnostics_.hasSchemaV1();
    const bool px4_schema_valid = px4_diagnostics_.found && px4_diagnostics_.hasSchemaV1();
    const bool lio_diagnostics_fresh =
        lio_schema_valid && lio_diagnostics_.stamp_ns > 0 && now_ns >= lio_diagnostics_.stamp_ns &&
        now_ns - lio_diagnostics_.stamp_ns <= config_.diagnostics_max_age_ns;
    const bool px4_diagnostics_fresh =
        px4_schema_valid && px4_diagnostics_.stamp_ns > 0 && now_ns >= px4_diagnostics_.stamp_ns &&
        now_ns - px4_diagnostics_.stamp_ns <= config_.diagnostics_max_age_ns;
    input.lio_diagnostics_schema_valid = lio_schema_valid;
    input.px4_diagnostics_schema_valid = px4_schema_valid;
    input.lio_diagnostics_stale = lio_schema_valid && !lio_diagnostics_fresh;
    input.px4_diagnostics_stale = px4_schema_valid && !px4_diagnostics_fresh;
    input.lio_diagnostics_valid = lio_diagnostics_fresh;
    input.px4_diagnostics_valid = px4_diagnostics_fresh;
    input.lio_valid = input.lio_diagnostics_valid && lio_status_is("TRACKING", "Tracking") &&
                      lio_diagnostics_.boolean("navigation_valid");
    input.lio_lost = input.lio_diagnostics_valid && lio_status_is("LOST", "Lost");
    input.lio_state_corruption =
        input.lio_diagnostics_valid &&
        lio_diagnostics_.string("last_update_failure_class") == "StateCorruption";
    input.lio_degraded = input.lio_diagnostics_valid && lio_status_is("DEGRADED", "Degraded");
    input.lio_resetting = input.lio_diagnostics_valid && lio_status_is("RESETTING", "Resetting");
    input.propagated_fresh = propagated_fresh;
    input.corrected_fresh = corrected_fresh;
    input.px4_available = latest_px4_.has_value() &&
                          latest_px4_->frame_id == kPx4OdomFrame;
    input.px4_fresh = px4_fresh;
    input.px4_continuity_valid = input.px4_diagnostics_valid &&
                                 px4_diagnostics_.boolean("continuity_valid");
    input.px4_post_reset_stable = input.px4_diagnostics_valid &&
                                  px4_diagnostics_.boolean("post_reset_stable");
    input.lio_generation = lio_diagnostics_.unsignedInteger("lio_generation");
    input.lio_generation_locked = input.lio_diagnostics_valid &&
                                  lio_diagnostics_.boolean("lio_generation_locked");
    input.correction_quality_valid = input.lio_diagnostics_valid &&
                                    lio_diagnostics_.boolean("corrected_estimate_valid") &&
                                    lio_diagnostics_.boolean("navigation_valid");
    input.timestamp_valid = input.lio_diagnostics_valid &&
                            lio_diagnostics_.integer("output_time_ns", -1) > 0 &&
                            propagated_fresh && corrected_fresh;
    const double covariance_minimum =
        lio_diagnostics_.number("covariance_minimum_eigenvalue", -1.0);
    const double covariance_trace = lio_diagnostics_.number("covariance_trace", -1.0);
    input.covariance_valid = input.lio_diagnostics_valid &&
                             std::isfinite(covariance_minimum) && covariance_minimum >= 0.0 &&
                             std::isfinite(covariance_trace) && covariance_trace > 0.0;
    input.continuity_unrecoverable =
        lio_diagnostics_.boolean("requires_reanchor") || input.lio_state_corruption;
    input.external_publisher_ready = external_diagnostics_.found &&
                                     external_diagnostics_.hasSchemaV1() &&
                                     external_diagnostics_.boolean("ready") &&
                                     external_diagnostics_.stamp_ns > 0 &&
                                     now_ns >= external_diagnostics_.stamp_ns &&
                                     now_ns - external_diagnostics_.stamp_ns <=
                                         config_.diagnostics_max_age_ns;
    input.propagated_age_ns = propagated_age;
    input.corrected_age_ns = corrected_age;
    input.px4_age_ns = px4_age;
    input.px4_reset_generation = px4_diagnostics_.unsignedInteger("reset_generation");
    input.px4_frame_generation = px4_diagnostics_.unsignedInteger("frame_generation");
    input.px4_time_generation = px4_diagnostics_.unsignedInteger("time_generation");
    auto lio_epoch = latest_propagated_epoch_available_to_px4();
    if (lio_epoch && now_ns > lio_epoch->timestamp_ns &&
        now_ns - lio_epoch->timestamp_ns > config_.query_epoch_max_age_ns) {
      const auto key = query_epoch_key(*lio_epoch, input.lio_generation,
                                       input.px4_frame_generation, input.px4_time_generation);
      if (!last_expired_query_key_ || *last_expired_query_key_ != key) {
        last_expired_query_key_ = key;
        ++query_epoch_expired_count_;
        set_query_failure("EPOCH_ELIGIBILITY", "eligible LIO epoch expired before comparison");
      }
      suppress_query_epoch(key);
      lio_epoch.reset();
    }
    input.latest_eligible_epoch_ns = lio_epoch ? lio_epoch->timestamp_ns : 0;
    input.alignment_gap_ns = aligned_comparison_
                                 ? std::llabs(aligned_comparison_->px4.timestamp_ns -
                                              aligned_comparison_->lio.timestamp_ns)
                                 : -1;
    input.comparison_epoch_ns = aligned_comparison_ ? aligned_comparison_->comparison_epoch_ns : 0;
    input.comparison_lag_to_latest_eligible_ns =
        aligned_comparison_ && lio_epoch &&
                lio_epoch->timestamp_ns >= aligned_comparison_->comparison_epoch_ns
            ? lio_epoch->timestamp_ns - aligned_comparison_->comparison_epoch_ns
            : -1;
    input.aligned_comparison_fresh = aligned_comparison_fresh(now_ns);
    input.pending_query_epoch_ns = pending_query_ ? pending_query_->requested_epoch_ns : 0;
    if (pending_query_) {
      input.pending_query_age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - pending_query_->started).count();
    }
    input.new_comparison_sample = new_comparison_sample_pending_;
    input.residual = aligned_comparison_ ? aligned_comparison_->residual : Residual{};
    input.query_sequence = aligned_comparison_ ? aligned_comparison_->query_sequence : 0;
    input.component_validity_mask = aligned_comparison_ ? aligned_comparison_->component_validity_mask : 0;
    input.covariance_availability_mask = aligned_comparison_ ? aligned_comparison_->covariance_availability_mask : 0;
    input.query_invalid_component_count = query_invalid_component_count_;
    input.query_generation_mismatch_count = query_generation_mismatch_count_;
    input.query_stale_sequence_count = query_stale_sequence_count_;
    input.query_timeout_count = query_timeout_count_;
    input.query_service_unavailable_count = query_service_unavailable_count_;
    input.query_success_count = query_success_count_;
    input.query_failure_count = query_failure_count_;
    input.query_epoch_not_yet_buffered_count = query_epoch_not_yet_buffered_count_;
    input.query_epoch_expired_count = query_epoch_expired_count_;
    input.query_duplicate_suppressed_count = query_duplicate_suppressed_count_;
    input.query_transport_failure_count = query_transport_failure_count_;
    input.query_geometric_failure_count = query_geometric_failure_count_;
    input.query_failure_class = query_failure_class_;
    input.query_last_failure_reason = query_last_failure_reason_;
    input.query_rtt_count = query_rtt_count_;
    input.query_rtt_p50_ms = query_rtt_percentile(0.50);
    input.query_rtt_p95_ms = query_rtt_percentile(0.95);
    input.query_rtt_p99_ms = query_rtt_percentile(0.99);
    input.query_rtt_max_ms = query_rtt_max_ms_;
    if (aligned_comparison_ && !input.new_comparison_sample && input.aligned_comparison_fresh) {
      ++stale_residual_reuse_count_;
    }
    input.stale_residual_reuse_count = stale_residual_reuse_count_;
    input.time_generation_changed = false;
    if (px4_diagnostics_.found && px4_schema_valid) {
      if (last_seen_px4_time_generation_ &&
          *last_seen_px4_time_generation_ != input.px4_time_generation) {
        input.time_generation_changed = true;
        pending_query_.reset();
        pending_alignment_query_.reset();
        clear_query_suppression();
        invalidate_comparison();
      }
      const bool reset_event_changed = last_seen_px4_reset_generation_ &&
                                       *last_seen_px4_reset_generation_ !=
                                           input.px4_reset_generation;
      if (reset_event_changed) {
        const bool compensated = last_seen_px4_frame_generation_ &&
                                 *last_seen_px4_frame_generation_ == input.px4_frame_generation;
        alignment_manager_.observeResetEvent(input.px4_reset_generation, now_ns, compensated);
        invalidate_comparison();
      }
      alignment_manager_.observeBindingGeneration(input.lio_generation,
                                                  input.px4_frame_generation,
                                                  input.px4_time_generation);
      if (last_query_key_ &&
          (last_query_key_->lio_generation != input.lio_generation ||
           last_query_key_->frame_generation != input.px4_frame_generation ||
           last_query_key_->time_generation != input.px4_time_generation)) {
        clear_query_suppression();
      }
      last_seen_px4_reset_generation_ = input.px4_reset_generation;
      last_seen_px4_frame_generation_ = input.px4_frame_generation;
      last_seen_px4_time_generation_ = input.px4_time_generation;
    }
    if (alignment_manager_.locked() &&
        (!input.px4_diagnostics_valid || !input.px4_continuity_valid ||
         !input.px4_post_reset_stable || !input.lio_diagnostics_valid)) {
      alignment_manager_.beginRevalidation("temporary diagnostics or continuity loss", now_ns);
      invalidate_comparison();
    }
    update_alignment_if_ready(input);
    const auto alignment_snapshot = alignment_manager_.snapshot(now_ns);
    input.alignment_candidate_valid = alignment_manager_.candidateValid();
    input.alignment_locked = alignment_manager_.locked();
    input.alignment_revalidating = alignment_manager_.revalidating();
    input.alignment_valid_for_comparison = input.alignment_locked;
    input.alignment_valid = input.alignment_valid_for_comparison;
    input.alignment_lifecycle = alignment_snapshot.state;
    input.alignment_candidate_estimate_count = alignment_snapshot.candidate_estimate_count;
    input.alignment_candidate_transition_count = alignment_snapshot.candidate_transition_count;
    input.alignment_revalidation_sample_count = alignment_snapshot.revalidation_sample_count;
    input.alignment_revalidation_success_count = alignment_snapshot.revalidation_success_count;
    input.alignment_revalidation_failure_count = alignment_snapshot.revalidation_failure_count;
    input.alignment_revalidation_start_count = alignment_snapshot.revalidation_start_count;
    input.alignment_revalidation_start_epoch_ns = alignment_snapshot.revalidation_start_epoch_ns;
    input.alignment_locked_transform_age_ns = alignment_snapshot.locked_transform_age_ns;
    input.alignment_frame_generation = alignment_snapshot.frame_generation;
    if (alignment_snapshot.locked_alignment) {
      input.alignment = *alignment_snapshot.locked_alignment;
    } else if (alignment_snapshot.candidate_alignment) {
      input.alignment = *alignment_snapshot.candidate_alignment;
    } else {
      input.alignment.target_frame = kLioOdomFrame;
      input.alignment.source_frame = kPx4OdomFrame;
      input.alignment.reset_generation = input.px4_reset_generation;
      input.alignment.frame_generation = input.px4_frame_generation;
      input.alignment.time_generation = input.px4_time_generation;
      input.alignment.lio_generation = input.lio_generation;
      input.alignment.rejection_reason = alignment_snapshot.rejection_reason;
    }
    input.alignment.reinitialization_count = alignment_reinitialization_count_;
    if (propagated_fresh && lio_epoch && !pending_query_ && !pending_alignment_query_ &&
        input.alignment_locked && !input.alignment_revalidating &&
        (!aligned_comparison_ || aligned_comparison_->comparison_epoch_ns != lio_epoch->timestamp_ns)) {
      request_px4_at(*lio_epoch, input.px4_reset_generation, input.px4_frame_generation,
                     input.px4_time_generation, input.alignment);
    }
    lifecycle_coordinator_.observe({input.lio_generation, input.lio_valid, input.lio_resetting,
                                    input.continuity_unrecoverable, latest_corrected_});
    const auto output = machine_.evaluate(input);
    if (output.reinitialization_requested) {
      static_cast<void>(lifecycle_coordinator_.requestReinitialization());
    }
    new_comparison_sample_pending_ = false;
    publish(output);
  }

  void publish(const SupervisorOutput& output) {
    StatusMessage message;
    message.header.stamp = now();
    message.header.frame_id = kLioOdomFrame;
    message.health = static_cast<std::uint8_t>(output.health);
    message.reference_mode = static_cast<std::uint8_t>(output.reference_mode);
    message.reason_code = output.reason_code;
    message.reason = output.reason;
    message.monitoring_available = output.monitoring_available;
    message.comparison_valid = output.comparison_valid;
    message.lio_valid = output.lio_valid;
    message.px4_valid = output.px4_valid;
    message.time_aligned = output.time_aligned;
    message.alignment_valid = output.alignment_valid;
    message.alignment_candidate_valid = output.alignment_candidate_valid;
    message.alignment_locked = output.alignment_locked;
    message.alignment_revalidating = output.alignment_revalidating;
    message.alignment_valid_for_comparison = output.alignment_valid_for_comparison;
    message.alignment_lifecycle_state = static_cast<std::uint8_t>(output.alignment_lifecycle);
    message.alignment_source = output.alignment.source;
    message.alignment_epoch = ros_time(output.alignment.epoch_ns);
    message.alignment_target_frame = output.alignment.target_frame;
    message.alignment_source_frame = output.alignment.source_frame;
    for (int index = 0; index < 3; ++index) {
      message.alignment_translation_xyz[static_cast<std::size_t>(index)] =
          output.alignment.target_from_source_translation[index];
    }
    message.alignment_yaw_rad = output.alignment.yaw_rad;
    for (int index = 0; index < 16; ++index) {
      message.alignment_covariance[static_cast<std::size_t>(index)] =
          output.alignment.covariance(index / 4, index % 4);
    }
    message.alignment_translation_dispersion_m = output.alignment.translation_dispersion_m;
    message.alignment_yaw_dispersion_rad = output.alignment.yaw_dispersion_rad;
    message.alignment_roll_pitch_disagreement_rad =
        output.alignment.roll_pitch_disagreement_rad;
    message.alignment_excitation_metric_m = output.alignment.excitation_metric_m;
    message.alignment_effective_sample_count = output.alignment.effective_sample_count;
    message.alignment_yaw_mode = output.alignment.yaw_mode;
    message.alignment_sample_count = output.alignment.sample_count;
    message.alignment_epoch_start_ns = output.alignment.epoch_start_ns;
    message.alignment_epoch_end_ns = output.alignment.epoch_end_ns;
    message.lio_generation = output.lio_generation;
    message.alignment_rejection_reason = output.alignment.rejection_reason;
    message.alignment_reset_generation = output.alignment.reset_generation;
    message.alignment_frame_generation = output.alignment_frame_generation;
    message.alignment_time_generation = output.alignment.time_generation;
    message.alignment_reinitialization_count = output.alignment.reinitialization_count;
    message.alignment_candidate_estimate_count = output.alignment_candidate_estimate_count;
    message.alignment_candidate_transition_count = output.alignment_candidate_transition_count;
    message.alignment_revalidation_sample_count = output.alignment_revalidation_sample_count;
    message.alignment_revalidation_success_count = output.alignment_revalidation_success_count;
    message.alignment_revalidation_failure_count = output.alignment_revalidation_failure_count;
    message.alignment_revalidation_start_count = output.alignment_revalidation_start_count;
    message.alignment_revalidation_start_epoch_ns = output.alignment_revalidation_start_epoch_ns;
    message.alignment_locked_transform_age_ns = output.alignment_locked_transform_age_ns;
    message.external_odometry_allowed = output.external_odometry_allowed;
    message.cross_comparison_valid = output.cross_comparison_valid;
    message.external_measurement_publishable = output.external_measurement_publishable;
    message.external_measurement_authorized = output.external_measurement_authorized;
    message.reinitialization_requested = output.reinitialization_requested;
    message.reinitialization_request_sequence = output.reinitialization_request_sequence;
    message.planner_speed_limit_active = output.planner_speed_limit_active;
    message.planner_speed_limit_m_s = output.planner_speed_limit_m_s;
    message.hover_or_failsafe_requested = output.hover_or_failsafe_requested;
    message.evaluation_time = ros_time(output.evaluation_time_ns);
    message.lio_propagated_age_ns = output.lio_propagated_age_ns;
    message.lio_corrected_age_ns = output.lio_corrected_age_ns;
    message.px4_age_ns = output.px4_age_ns;
    message.alignment_gap_ns = output.alignment_gap_ns;
    message.aligned_comparison_age_ns = output.aligned_comparison_age_ns;
    message.position_error_m = output.residual.position_error_m;
    message.velocity_error_m_s = output.residual.velocity_error_m_s;
    message.orientation_error_rad = output.residual.orientation_error_rad;
    message.yaw_error_rad = output.residual.yaw_error_rad;
    message.position_error_growth_m_s = output.residual.position_error_growth_m_s;
    message.px4_reset_generation = output.px4_reset_generation;
    message.px4_frame_generation = output.px4_frame_generation;
    message.px4_time_generation = output.px4_time_generation;
    message.correction_quality_valid = output.correction_quality_valid;
    message.timestamp_valid = output.timestamp_valid;
    message.covariance_valid = output.covariance_valid;
    message.lio_generation_locked = output.lio_generation_locked;
    message.continuity_unrecoverable = output.continuity_unrecoverable;
    message.external_publisher_ready = output.external_publisher_ready;
    message.state_transition_count = output.state_transition_count;
    message.evaluation_count = output.evaluation_count;
    message.alignment_failure_count = output.alignment_failure_count;
    message.comparison_epoch = ros_time(output.comparison_epoch_ns);
    message.new_comparison_sample = output.new_comparison_sample;
    message.aligned_comparison_fresh = output.aligned_comparison_fresh;
    message.query_sequence = output.query_sequence;
    message.component_validity_mask = output.component_validity_mask;
    message.covariance_availability_mask = output.covariance_availability_mask;
    message.query_invalid_component_count = output.query_invalid_component_count;
    message.query_generation_mismatch_count = output.query_generation_mismatch_count;
    message.query_stale_sequence_count = output.query_stale_sequence_count;
    message.query_timeout_count = output.query_timeout_count;
    message.query_service_unavailable_count = output.query_service_unavailable_count;
    message.query_success_count = output.query_success_count;
    message.query_failure_count = output.query_failure_count;
    message.query_epoch_not_yet_buffered_count = output.query_epoch_not_yet_buffered_count;
    message.query_epoch_expired_count = output.query_epoch_expired_count;
    message.query_duplicate_suppressed_count = output.query_duplicate_suppressed_count;
    message.query_transport_failure_count = output.query_transport_failure_count;
    message.query_geometric_failure_count = output.query_geometric_failure_count;
    message.query_failure_class = output.query_failure_class;
    message.query_last_failure_reason = output.query_last_failure_reason;
    message.query_rtt_count = output.query_rtt_count;
    message.query_rtt_p50_ms = output.query_rtt_p50_ms;
    message.query_rtt_p95_ms = output.query_rtt_p95_ms;
    message.query_rtt_p99_ms = output.query_rtt_p99_ms;
    message.query_rtt_max_ms = output.query_rtt_max_ms;
    message.stale_residual_reuse_count = output.stale_residual_reuse_count;
    status_->publish(message);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = message.header.stamp;
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = "odometry_supervisor";
    diagnostic.level = output.health == HealthState::kHealthy
                           ? diagnostic_msgs::msg::DiagnosticStatus::OK
                           : (output.health == HealthState::kDiverged
                                  ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                                  : diagnostic_msgs::msg::DiagnosticStatus::WARN);
    diagnostic.message = health_name(output.health) + std::string(": ") + output.reason;
    const auto add_value = [&diagnostic](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      diagnostic.values.push_back(std::move(item));
    };
    add_value("diagnostic_schema_version", "1");
    add_value("health", health_name(output.health));
    add_value("reference_mode", output.reference_mode == ReferenceMode::kIndependent ? "independent" : "correlated");
    add_value("comparison_valid", output.comparison_valid ? "true" : "false");
    add_value("alignment_valid", output.alignment_valid ? "true" : "false");
    add_value("alignment_candidate_valid", output.alignment_candidate_valid ? "true" : "false");
    add_value("alignment_locked", output.alignment_locked ? "true" : "false");
    add_value("alignment_revalidating", output.alignment_revalidating ? "true" : "false");
    add_value("alignment_valid_for_comparison",
              output.alignment_valid_for_comparison ? "true" : "false");
    add_value("alignment_lifecycle_state",
              alignment_lifecycle_name(output.alignment_lifecycle));
    add_value("alignment_source", output.alignment.source);
    add_value("alignment_epoch_ns", std::to_string(output.alignment.epoch_ns));
    add_value("alignment_reset_generation",
              std::to_string(output.alignment.reset_generation));
    add_value("alignment_frame_generation", std::to_string(output.alignment_frame_generation));
    add_value("alignment_time_generation",
              std::to_string(output.alignment.time_generation));
    add_value("alignment_reinitialization_count",
              std::to_string(output.alignment.reinitialization_count));
    add_value("alignment_candidate_estimate_count",
              std::to_string(output.alignment_candidate_estimate_count));
    add_value("alignment_candidate_transition_count",
              std::to_string(output.alignment_candidate_transition_count));
    add_value("alignment_revalidation_sample_count",
              std::to_string(output.alignment_revalidation_sample_count));
    add_value("alignment_revalidation_success_count",
              std::to_string(output.alignment_revalidation_success_count));
    add_value("alignment_revalidation_failure_count",
              std::to_string(output.alignment_revalidation_failure_count));
    add_value("alignment_revalidation_start_count",
              std::to_string(output.alignment_revalidation_start_count));
    add_value("alignment_revalidation_start_epoch_ns",
              std::to_string(output.alignment_revalidation_start_epoch_ns));
    add_value("alignment_locked_transform_age_ns",
              std::to_string(output.alignment_locked_transform_age_ns));
    add_value("alignment_rejection_reason", output.alignment.rejection_reason);
    add_value("alignment_sample_count", std::to_string(output.alignment.sample_count));
    add_value("alignment_translation_dispersion_m",
              std::to_string(output.alignment.translation_dispersion_m));
    add_value("alignment_yaw_dispersion_rad",
              std::to_string(output.alignment.yaw_dispersion_rad));
    add_value("alignment_excitation_metric_m",
              std::to_string(output.alignment.excitation_metric_m));
    add_value("alignment_effective_sample_count",
              std::to_string(output.alignment.effective_sample_count));
    add_value("alignment_yaw_mode", output.alignment.yaw_mode);
    add_value("lio_generation", std::to_string(output.lio_generation));
    add_value("px4_frame_generation", std::to_string(output.px4_frame_generation));
    add_value("lio_generation_locked", output.lio_generation_locked ? "true" : "false");
    add_value("correction_quality_valid", output.correction_quality_valid ? "true" : "false");
    add_value("timestamp_valid", output.timestamp_valid ? "true" : "false");
    add_value("covariance_valid", output.covariance_valid ? "true" : "false");
    add_value("continuity_unrecoverable", output.continuity_unrecoverable ? "true" : "false");
    add_value("external_publisher_ready", output.external_publisher_ready ? "true" : "false");
    add_value("lio_lifecycle_state", lifecycle_name(lifecycle_coordinator_.state()));
    add_value("lio_odometry_only_snapshot_valid",
              lifecycle_coordinator_.snapshot().has_value() ? "true" : "false");
    add_value("lio_tracking_ever_confirmed",
              lifecycle_coordinator_.trackingEverConfirmed() ? "true" : "false");
    add_value("lio_reinitialization_count",
              std::to_string(lifecycle_coordinator_.reinitialization_count()));
    add_value("aligned_comparison_fresh", output.aligned_comparison_fresh ? "true" : "false");
    add_value("lio_diagnostics_valid", output.lio_diagnostics_valid ? "true" : "false");
    add_value("px4_diagnostics_valid", output.px4_diagnostics_valid ? "true" : "false");
    add_value("lio_diagnostics_schema_valid", output.lio_diagnostics_schema_valid ? "true" : "false");
    add_value("px4_diagnostics_schema_valid", output.px4_diagnostics_schema_valid ? "true" : "false");
    add_value("lio_diagnostics_stale", output.lio_diagnostics_stale ? "true" : "false");
    add_value("px4_diagnostics_stale", output.px4_diagnostics_stale ? "true" : "false");
    add_value("new_comparison_sample", output.new_comparison_sample ? "true" : "false");
    add_value("latest_eligible_epoch_ns", std::to_string(output.latest_eligible_epoch_ns));
    add_value("comparison_epoch_ns", std::to_string(output.comparison_epoch_ns));
    add_value("comparison_lag_to_latest_eligible_ns",
              std::to_string(output.comparison_lag_to_latest_eligible_ns));
    add_value("pending_query_epoch_ns", std::to_string(output.pending_query_epoch_ns));
    add_value("pending_query_age_ns", std::to_string(output.pending_query_age_ns));
    add_value("query_sequence", std::to_string(output.query_sequence));
    add_value("component_validity_mask", std::to_string(output.component_validity_mask));
    add_value("covariance_availability_mask", std::to_string(output.covariance_availability_mask));
    add_value("query_invalid_component_count", std::to_string(output.query_invalid_component_count));
    add_value("query_generation_mismatch_count", std::to_string(output.query_generation_mismatch_count));
    add_value("query_stale_sequence_count", std::to_string(output.query_stale_sequence_count));
    add_value("query_timeout_count", std::to_string(output.query_timeout_count));
    add_value("query_service_unavailable_count", std::to_string(output.query_service_unavailable_count));
    add_value("query_success_count", std::to_string(output.query_success_count));
    add_value("query_failure_count", std::to_string(output.query_failure_count));
    add_value("query_epoch_not_yet_buffered_count",
              std::to_string(output.query_epoch_not_yet_buffered_count));
    add_value("query_epoch_expired_count", std::to_string(output.query_epoch_expired_count));
    add_value("query_duplicate_suppressed_count",
              std::to_string(output.query_duplicate_suppressed_count));
    add_value("query_transport_failure_count",
              std::to_string(output.query_transport_failure_count));
    add_value("query_geometric_failure_count",
              std::to_string(output.query_geometric_failure_count));
    add_value("query_failure_class", output.query_failure_class);
    add_value("query_last_failure_reason", output.query_last_failure_reason);
    add_value("query_rtt_count", std::to_string(output.query_rtt_count));
    add_value("query_rtt_p50_ms", std::to_string(output.query_rtt_p50_ms));
    add_value("query_rtt_p95_ms", std::to_string(output.query_rtt_p95_ms));
    add_value("query_rtt_p99_ms", std::to_string(output.query_rtt_p99_ms));
    add_value("query_rtt_max_ms", std::to_string(output.query_rtt_max_ms));
    add_value("stale_residual_reuse_count", std::to_string(output.stale_residual_reuse_count));
    add_value("external_odometry_allowed", output.external_odometry_allowed ? "true" : "false");
    add_value("cross_comparison_valid", output.cross_comparison_valid ? "true" : "false");
    add_value("external_measurement_publishable",
              output.external_measurement_publishable ? "true" : "false");
    add_value("external_measurement_authorized",
              output.external_measurement_authorized ? "true" : "false");
    add_value("reinitialization_requested", output.reinitialization_requested ? "true" : "false");
    add_value("state_transition_count", std::to_string(output.state_transition_count));
    add_value("reason", output.reason);
    add_value("position_error_m", std::to_string(output.residual.position_error_m));
    add_value("velocity_error_m_s", std::to_string(output.residual.velocity_error_m_s));
    add_value("orientation_error_rad", std::to_string(output.residual.orientation_error_rad));
    add_value("yaw_error_rad", std::to_string(output.residual.yaw_error_rad));
    add_value("heading_observable", output.residual.heading_observable ? "true" : "false");
    add_value("euler_yaw_error_rad", std::to_string(output.residual.euler_yaw_error_rad));
    add_value("robust_heading_lio_rad", std::to_string(output.residual.robust_heading_lio_rad));
    add_value("robust_heading_px4_rad", std::to_string(output.residual.robust_heading_px4_rad));
    add_value("q_error_axis_x", std::to_string(output.residual.q_error_axis.x()));
    add_value("q_error_axis_y", std::to_string(output.residual.q_error_axis.y()));
    add_value("q_error_axis_z", std::to_string(output.residual.q_error_axis.z()));
    add_value("body_z_dot", std::to_string(output.residual.body_z_dot));
    add_value("body_x_horizontal_norm_lio",
              std::to_string(output.residual.body_x_horizontal_norm_lio));
    add_value("body_x_horizontal_norm_px4",
              std::to_string(output.residual.body_x_horizontal_norm_px4));
    add_value("position_error_growth_m_s", std::to_string(output.residual.position_error_growth_m_s));
    array.status.push_back(std::move(diagnostic));
    diagnostics_->publish(std::move(array));
  }

  SupervisorConfig config_;
  SupervisorStateMachine machine_;
  rclcpp::Publisher<StatusMessage>::SharedPtr status_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr propagated_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr corrected_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr px4_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr lio_diagnostics_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr px4_diagnostics_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr external_diagnostics_sub_;
  rclcpp::Client<QueryService>::SharedPtr query_client_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;
  std::optional<OdometryState> latest_propagated_;
  static constexpr std::size_t kPropagatedHistoryCapacity = 256;
  std::deque<OdometryState> propagated_history_;
  std::optional<OdometryState> latest_corrected_;
  std::optional<OdometryState> latest_px4_;
  std::optional<AlignedComparison> aligned_comparison_;
  static constexpr std::size_t kPx4HistoryCapacity = 256;
  std::deque<OdometryState> px4_history_;
  OdometryAlignmentEstimator alignment_estimator_;
  AlignmentLifecycleManager alignment_manager_;
  AlignmentEstimate last_alignment_estimate_;
  std::int64_t last_alignment_pair_epoch_ns_{0};
  std::string alignment_rejection_reason_;
  std::optional<Residual> previous_residual_;
  DiagnosticSnapshot lio_diagnostics_;
  DiagnosticSnapshot px4_diagnostics_;
  DiagnosticSnapshot external_diagnostics_;
  std::optional<PendingQuery> pending_query_;
  std::optional<PendingAlignmentQuery> pending_alignment_query_;
  std::uint64_t query_sequence_{0};
  std::uint64_t query_invalid_component_count_{0};
  std::uint64_t query_generation_mismatch_count_{0};
  std::uint64_t query_stale_sequence_count_{0};
  std::uint64_t query_timeout_count_{0};
  std::uint64_t query_service_unavailable_count_{0};
  std::array<std::uint64_t, 16> query_rtt_histogram_{};
  std::uint64_t query_rtt_count_{0};
  double query_rtt_max_ms_{0.0};
  std::uint64_t query_success_count_{0};
  std::uint64_t query_failure_count_{0};
  std::uint64_t query_epoch_not_yet_buffered_count_{0};
  std::uint64_t query_epoch_expired_count_{0};
  std::uint64_t query_duplicate_suppressed_count_{0};
  std::uint64_t query_transport_failure_count_{0};
  std::uint64_t query_geometric_failure_count_{0};
  std::string query_failure_class_{"NONE"};
  std::string query_last_failure_reason_;
  std::optional<QueryEpochKey> last_query_key_;
  std::optional<QueryEpochKey> suppressed_query_key_;
  std::optional<QueryEpochKey> service_retry_key_;
  std::optional<QueryEpochKey> last_expired_query_key_;
  std::optional<std::int64_t> last_not_yet_buffered_epoch_;
  std::size_t service_retry_attempts_{0};
  std::chrono::steady_clock::time_point next_service_retry_{};
  static constexpr std::size_t kMaxServiceRetryAttempts = 3;
  std::uint64_t stale_residual_reuse_count_{0};
  bool new_comparison_sample_pending_{false};
  std::optional<std::uint64_t> last_seen_px4_time_generation_;
  std::optional<std::uint64_t> last_seen_px4_reset_generation_;
  std::optional<std::uint64_t> last_seen_px4_frame_generation_;
  std::uint64_t alignment_reinitialization_count_{0};
  LioLifecycleCoordinator lifecycle_coordinator_;
};

}  // namespace odometry_supervisor

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odometry_supervisor::OdometrySupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
