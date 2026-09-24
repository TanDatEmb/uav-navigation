#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

#include <navigation_contracts/msg/navigation_mission_progress.hpp>
#include <tracking_experiment.hpp>
#include <navigation_contracts/msg/navigation_mode_status.hpp>
#include <navigation_contracts/msg/propagated_odometry.hpp>
#include <navigation_contracts/msg/estimator_health.hpp>
#include <navigation_contracts/msg/navigation_command.hpp>
#include <navigation_contracts/msg/navigation_command_admission.hpp>
#include <navigation_contracts/msg/navigation_command_rejection.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/health_and_arming_checks.hpp>
#include <px4_ros2/components/shared_subscription.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include "px4_navigation_external_mode/px4_input_trace.hpp"
#include "px4_navigation_external_mode/px4_tracking_adapter.hpp"
#include <navigation_common/bounded_spsc_queue.hpp>

#include "px4_navigation_external_mode/velocity_only_continuity.hpp"
#include "px4_navigation_external_mode/command_admission_assessment.hpp"
#include "px4_navigation_external_mode/tracking_envelope.hpp"
#include "px4_navigation_external_mode/reject_provenance.hpp"

namespace px4_navigation_external_mode {

class NavigationMode final : public px4_ros2::ModeBase {
 public:
  explicit NavigationMode(rclcpp::Node& node);
  ~NavigationMode() override;

  void setPx4HoldHandover(std::function<void()> callback);
  void attachStateInputNode(rclcpp::Node& state_input_node);
  void onActivate() override;
  void onDeactivate() override;
  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter& reporter) override;
 void updateSetpoint(float dt_s) override;

 private:
  friend class NavigationModeProgressionTest;

  struct VelocityOnlySnapshot final {
    nav_msgs::msg::Odometry odometry;
    Px4InputStateTrace state_input_trace;
    tracking_adapter::RawPx4State px4;
    std::optional<velocity_only::Previous> previous;
    tracking_adapter::ResetCounters last_reset_counters;
    bool reset_counters_seen{false};
    std::uint64_t lio_localization_epoch{0U};
    std::uint64_t lio_sequence{0U};
    std::int64_t lio_receive_steady_ns{0};
    bool health_navigation_valid{false};
    bool health_covariance_valid{false};
    bool health_observability_valid{false};
    bool health_correction_fresh{false};
    bool health_propagation_valid{false};
  };

  rclcpp::Node& node_;
  void onNavigationCommand(
      const navigation_contracts::msg::NavigationCommand::ConstSharedPtr& message);
  void publishAdmissionRejection(
      const CommandAdmissionAssessment& assessment,
      const navigation_contracts::msg::NavigationCommand* command,
      std::int64_t callback_ros_ns, std::int64_t callback_steady_ns,
      double source_age_ms = 0.0, double receive_age_ms = 0.0,
      double longitudinal_error_m = 0.0, double lateral_error_m = 0.0);
  // Called under trajectory_mutex_; pure geometric/adaptive checks remain in
  // their existing bounded helpers, while this method records local metrics.
  TrackingEnvelopeResult assessTrackingLocked(
      const navigation_contracts::msg::NavigationCommand& command,
      const nav_msgs::msg::Odometry& odometry, bool& anchor_invalid,
      std::optional<RejectProvenance>& reject_provenance);
  void finishAcceptedCommand(
      const navigation_contracts::msg::NavigationCommand& command,
      bool completed_command, bool terminal_recovery_needed,
      bool terminal_backup_hold_inside_acceptance,
      bool terminal_main_hold_inside_acceptance);
  void onOdometry(
      const navigation_contracts::msg::PropagatedOdometry::ConstSharedPtr& message);
  void onPx4LocalPosition(
      const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr& message);
  void tryAlignPx4LocalFrameLocked();
  void onEstimatorHealth(
      const navigation_contracts::msg::EstimatorHealth::ConstSharedPtr& message);
  void updateBoundary();
  void onMissionProgress(
      const navigation_contracts::msg::NavigationMissionProgress::ConstSharedPtr& message);
  void clearPlannerRecoveryEpisodeLocked() noexcept;
  void rememberPlannerRecoveryEpisodeLocked(
      const navigation_contracts::msg::NavigationCommand& command);
  [[nodiscard]] bool plannerRecoveryEpisodeMatchesLocked(
      const navigation_contracts::msg::NavigationCommand& command) const noexcept;
  void safetyStopNavigation(const char* reason);
  void failNavigation(const char* reason);
  void logRuntimeMetrics(const rclcpp::Time& now);
  [[nodiscard]] Px4InputTraceRecord makePx4InputTraceRecord(
      const navigation_contracts::msg::NavigationCommand* command,
      const std::optional<Eigen::Vector3f>& position_ned,
      const std::optional<Eigen::Vector3f>& velocity_ned,
      const std::optional<Eigen::Vector3f>& acceleration_ned,
      float yaw_ned, float yaw_rate_ned, Px4InputTraceBoundary boundary,
      std::int64_t update_start_ros_ns, std::int64_t update_start_steady_ns,
      std::string_view velocity_only_reason,
      std::uint64_t velocity_only_limited_count,
      const Px4InputStateTrace& state_input_trace);
  void enqueuePx4InputTrace(Px4InputTraceRecord record);
  void drainPx4InputTrace();
  void publishPx4InputTrace(const Px4InputTraceRecord& record);
  void publishAlignmentLatchWitnessLocked();
  bool publishVelocityOnlySetpoint(
      const navigation_contracts::msg::NavigationCommand& command,
      const VelocityOnlySnapshot& snapshot, const rclcpp::Time& now);
  void setVelocityOnlyLastReason(std::string_view reason);
  void requestVelocityOnlyHold(const char* reason);
  void publishStatus(std::uint8_t state, std::uint8_t reason);

  std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;
  rclcpp::Subscription<navigation_contracts::msg::PropagatedOdometry>::SharedPtr
      odometry_subscription_;
  rclcpp::Subscription<navigation_contracts::msg::NavigationCommand>::SharedPtr
      navigation_command_subscription_;
  rclcpp::Subscription<navigation_contracts::msg::EstimatorHealth>::SharedPtr
      estimator_health_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      px4_local_position_subscription_;
  rclcpp::Subscription<navigation_contracts::msg::NavigationMissionProgress>::SharedPtr
      mission_progress_subscription_;
  rclcpp::Publisher<navigation_contracts::msg::NavigationModeStatus>::SharedPtr
      status_publisher_;
  rclcpp::Publisher<navigation_contracts::msg::NavigationCommandAdmission>::SharedPtr
      command_admission_publisher_;
  rclcpp::Publisher<navigation_contracts::msg::NavigationCommandRejection>::SharedPtr
      command_rejection_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      px4_input_trace_publisher_;
  rclcpp::TimerBase::SharedPtr boundary_timer_;
  std::mutex trajectory_mutex_;
  rclcpp::Time activation_time_;
  rclcpp::Time last_setpoint_time_;
  std::string navigation_command_topic_;
  std::string state_topic_;
  std::string planning_frame_;
  navigation_contracts::TrackingExperimentPolicy tracking_experiment_;
  std::uint64_t experimental_tracking_suppressed_count_{0};
  std::string body_frame_;
  double stale_after_s_{0.10};
  double state_stale_after_s_{0.20};
  double trajectory_wait_timeout_s_{5.0};
  double planner_recovery_wait_timeout_s_{5.0};
  std::int64_t stale_after_ns_{0};
  std::int64_t state_stale_after_ns_{0};
  std::int64_t planner_recovery_wait_timeout_ns_{0};
  std::optional<nav_msgs::msg::Odometry> odometry_;
  std::optional<Eigen::Vector3d> px4_local_position_ned_;
  std::optional<Eigen::Vector3d> px4_local_velocity_ned_;
  std::optional<Eigen::Vector3d> lio_to_px4_local_translation_ned_;
  std::uint8_t px4_xy_reset_counter_{0U};
  std::uint8_t px4_z_reset_counter_{0U};
  std::uint8_t px4_vxy_reset_counter_{0U};
  std::uint8_t px4_vz_reset_counter_{0U};
  std::uint8_t px4_heading_reset_counter_{0U};
  std::uint64_t last_px4_local_position_timestamp_us_{0U};
  std::uint64_t last_px4_local_position_timestamp_sample_us_{0U};
  std::int64_t last_px4_local_position_receive_ns_{0};
  std::int64_t last_px4_position_receive_steady_ns_{0};
  bool last_px4_xy_valid_{false};
  bool last_px4_z_valid_{false};
  bool last_px4_vxy_valid_{false};
  bool last_px4_vz_valid_{false};
  bool last_px4_dead_reckoning_{false};
  bool last_px4_heading_good_for_control_{false};
  bool last_px4_heading_valid_{false};
  double last_px4_heading_ned_{0.0};
  double last_px4_heading_variance_rad2_{0.0};
  float last_px4_delta_xy_north_m_{0.0F};
  float last_px4_delta_xy_east_m_{0.0F};
  float last_px4_delta_z_m_{0.0F};
  float last_px4_delta_heading_rad_{0.0F};
  std::int64_t last_px4_local_position_receive_steady_ns_{0};
  bool px4_local_frame_aligned_{false};
  std::optional<navigation_contracts::msg::NavigationCommand> navigation_command_;
  bool lio_health_valid_{false};
  bool typed_health_seen_{false};
  std::uint8_t last_health_state_{0U};
  bool last_health_navigation_valid_{false};
  bool last_health_covariance_valid_{false};
  bool last_health_observability_valid_{false};
  bool last_health_correction_fresh_{false};
  bool last_health_propagation_valid_{false};
  std::uint64_t lio_localization_epoch_{0U};
  std::int64_t last_propagated_state_stamp_ns_{0};
  std::uint64_t last_propagated_state_sequence_{0U};
  std::int64_t last_lio_diagnostics_ns_{0};
  std::int64_t last_health_source_stamp_ns_{0};
  std::int64_t last_health_correction_stamp_ns_{0};
  std::int64_t last_health_propagated_stamp_ns_{0};
  std::int64_t last_health_receive_steady_ns_{0};
  std::function<void()> px4_hold_handover_;
  bool failure_reported_{false};
  bool mode_active_{false};
  // Immutable Core-owned completion receipt used only for PX4 mode handover.
  std::optional<navigation_contracts::msg::NavigationMissionProgress>
      mission_completion_receipt_;
  std::uint64_t mode_activation_id_{0U};
  bool handover_requested_{false};
  bool planner_recovery_pending_{false};
  std::int64_t planner_recovery_deadline_ns_{0};
  std::string planner_recovery_mission_id_;
  std::uint32_t planner_recovery_waypoint_index_{0U};
  std::uint64_t planner_recovery_request_id_{0U};
  std::uint64_t planner_recovery_bundle_generation_{0U};
  std::optional<Eigen::Vector3d> completion_position_;
  std::optional<Eigen::Vector3d> safety_hold_position_;
  std::uint8_t last_status_state_{navigation_contracts::msg::NavigationModeStatus::PAUSED};
  std::uint64_t odometry_callback_count_{0U};
  std::uint64_t trajectory_received_count_{0U};
  std::uint64_t trajectory_accepted_count_{0U};
  std::uint64_t trajectory_rejected_count_{0U};
  std::array<std::uint64_t, 8> admission_rejections_by_stage_{};
  std::uint64_t waypoint_handoff_retained_command_count_{0U};
  std::uint64_t setpoint_update_count_{0U};
  std::uint64_t stale_state_failure_count_{0U};
  std::int64_t last_odometry_receive_ns_{0};
  std::int64_t last_odometry_receive_steady_ns_{0};
  Px4InputStateTrace odometry_input_trace_;
  std::int64_t last_command_receive_ns_{0};
  // PX4-local start of the airborne first-command acquisition lease.
  std::int64_t airborne_start_ns_{0};
  std::int64_t maximum_odometry_callback_gap_us_{0};
  std::int64_t last_setpoint_update_ns_{0};
  std::int64_t maximum_setpoint_callback_gap_us_{0};
  std::int64_t last_metrics_log_ns_{0};
  std::int64_t last_planner_debug_log_ns_{0};
  double last_state_age_s_{-1.0};
  Eigen::Vector3d last_velocity_command_enu_{Eigen::Vector3d::Zero()};
  std::uint64_t last_forward_guard_count_{0U};
  std::uint64_t px4_input_trace_sequence_{0U};
  // Single producer: ModeBase invokes updateSetpoint(); its stationary,
  // position-hold and velocity-only helpers are synchronous nested calls.
  // Single consumer: px4_input_trace_worker_. Do not enqueue from ROS
  // subscription/timer callbacks without replacing this queue contract.
  navigation_common::BoundedSpscQueue<Px4InputTraceRecord, 256U>
      px4_input_trace_queue_;
  std::atomic<bool> px4_input_trace_worker_stop_{false};
  std::thread px4_input_trace_worker_;
  std::atomic<std::uint64_t> px4_input_trace_drop_count_{0U};
  std::atomic<std::uint64_t> px4_input_trace_enqueued_count_{0U};
  std::atomic<std::uint64_t> px4_input_trace_published_count_{0U};
  std::atomic<std::uint64_t> px4_input_trace_publish_error_count_{0U};
  std::uint64_t alignment_latch_generation_{0U};
  std::optional<velocity_only::Previous> velocity_only_previous_;
  tracking_adapter::ResetCounters velocity_only_last_reset_counters_;
  bool velocity_only_reset_counters_seen_{false};
  std::string velocity_only_last_reason_;
  std::uint64_t velocity_only_limited_count_{0U};

};

class NavigationModeExecutor final : public px4_ros2::ModeExecutorBase {
 public:
  explicit NavigationModeExecutor(px4_ros2::ModeBase& owned_mode);

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;
  void onFailsafeDeferred() override;

 private:
  void onOwnedModeCompleted(px4_ros2::Result result);
  void schedulePx4Hold(bool complete_navigation_failure);
  void onPx4HoldHandoverCompleted(px4_ros2::Result result,
                                  bool complete_navigation_failure);
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::UniquePtr& message);
  void checkHoldHandover();

  rclcpp::Node& node_;
  NavigationMode& navigation_mode_;
  SharedSubscriptionCallbackInstance vehicle_status_subscription_;
  rclcpp::TimerBase::SharedPtr handover_timer_;
  bool px4_hold_confirmed_{false};
  bool hold_handover_pending_{false};
  bool hold_handover_in_flight_{false};
  bool hold_handover_complete_navigation_failure_{false};
  std::uint32_t hold_handover_attempts_{0U};
  std::int64_t hold_handover_next_retry_steady_ns_{0};
};

}  // namespace px4_navigation_external_mode
