#include "uavnav/lio/pipeline.hpp"

#include <expected>
#include <iterator>
#include <utility>

namespace uavnav::lio {

LioPipeline::LioPipeline(std::unique_ptr<LioChannels> channels, std::unique_ptr<LioBackend> backend,
                         std::unique_ptr<LioFrontend> frontend) noexcept
    : channels_(std::move(channels)), backend_(std::move(backend)), frontend_(std::move(frontend)) {}

Result<std::unique_ptr<LioPipeline>, config::ConfigError> LioPipeline::create(
    const LioConfig& config, const Eigen::Isometry3d& base_T_imu, const Eigen::Isometry3d& imu_T_lidar,
    events::EventRecorder& events) {
  auto channels = std::make_unique<LioChannels>();  // heap: the queues hold their slots inline
  auto backend = LioBackend::create(backend_config(config), base_T_imu, imu_T_lidar, *channels, events);
  if (!backend) return std::unexpected(backend.error());
  auto frontend = std::make_unique<LioFrontend>(frontend_config(config), *channels, events);
  return std::unique_ptr<LioPipeline>(
      new LioPipeline(std::move(channels), std::move(*backend), std::move(frontend)));
}

void LioPipeline::drive(StepOutputs& out, const time::TimeSnapshot& now) {
  while (backend_->step(now)) {
  }
  StepOutputs polled = frontend_->poll_results(now);
  out.states.insert(out.states.end(), std::make_move_iterator(polled.states.begin()),
                    std::make_move_iterator(polled.states.end()));
  if (polled.odometry) out.odometry = std::move(polled.odometry);
  if (polled.health) out.health = std::move(polled.health);
  if (polled.restart) out.restart = std::move(polled.restart);
}

Result<StepOutputs, EstimatorReason> LioPipeline::push_imu(const ImuInput& imu, const time::TimeSnapshot& now) {
  auto r = frontend_->push_imu(imu, now);
  if (!r) return r;
  drive(*r, now);
  return r;
}

Result<StepOutputs, EstimatorReason> LioPipeline::push_scan(ScanInput&& scan, const time::TimeSnapshot& now) {
  auto r = frontend_->push_scan(std::move(scan), now);
  if (!r) return r;
  drive(*r, now);
  return r;
}

Result<ResetDelta, EstimatorReason> LioPipeline::restart(const SeedPose& seed, const time::TimeSnapshot& now) {
  const auto queued = frontend_->restart(seed, now);
  if (!queued) return std::unexpected(queued.error());
  StepOutputs out;
  drive(out, now);
  // The backend answers a queued restart within the drain above; a missing answer means it never ran.
  if (!out.restart) return std::unexpected(EstimatorReason::kBackendBusy);
  return *std::move(out.restart);
}

}  // namespace uavnav::lio
