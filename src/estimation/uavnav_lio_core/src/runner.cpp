#include "uavnav/lio/runner.hpp"

#include <exception>
#include <expected>
#include <iterator>
#include <utility>

#include "uavnav/core/event_builder.hpp"
#include "uavnav/lio/limits.hpp"

namespace uavnav::lio {

namespace {

/// Outputs of two consecutive applications merged "last wins" (outputs.hpp): `later`'s states follow
/// `acc`'s, and each of `later`'s non-empty optionals replaces `acc`'s.
void merge(StepOutputs& acc, StepOutputs&& later) {
  if (acc.states.empty()) {
    acc.states = std::move(later.states);
  } else {
    acc.states.insert(acc.states.end(), std::make_move_iterator(later.states.begin()),
                      std::make_move_iterator(later.states.end()));
  }
  if (later.odometry) acc.odometry = std::move(later.odometry);
  if (later.health) acc.health = std::move(later.health);
  if (later.restart) acc.restart = std::move(later.restart);
}

/// The caller's clock as the backend sees it (estimator thread only): a throw is caught, reported by a
/// rate-limited "LioClockFailed" event and answered with the last good snapshot, so the worker never dies.
class GuardedClock {
 public:
  GuardedClock(const LioClock& clock, events::EventRecorder& events) noexcept : clock_(clock), events_(events) {}

  time::TimeSnapshot operator()() noexcept {
    try {
      last_ = clock_();
      return last_;
    } catch (...) {
      ++failures_;
      if (failures_ == 1 || failures_ % limits::kEventSummaryEvery == 0) report();
      return last_;
    }
  }

  time::TimeSnapshot last() const noexcept { return last_; }

  /// End of run(): the final count, when failures happened since the last event.
  void flush() noexcept {
    if (failures_ > reported_) report();
  }

 private:
  void report() noexcept {
    reported_ = failures_;
    (void)events_.emit(events::EventBuilder(events::Component::kLio, "LioClockFailed", last_)
                           .reason(EstimatorEventReason::kClockFailed)
                           .value("count", static_cast<double>(failures_))
                           .build());
  }

  const LioClock& clock_;
  events::EventRecorder& events_;
  time::TimeSnapshot last_{};  // zero until the first good call
  std::uint64_t failures_{0};
  std::uint64_t reported_{0};  // failures_ at the last event
};

}  // namespace

LioRunner::LioRunner(std::unique_ptr<LioChannels> channels, std::unique_ptr<LioBackend> backend,
                     std::unique_ptr<LioFrontend> frontend, events::EventRecorder& events, LioClock clock)
    : channels_(std::move(channels)),
      backend_(std::move(backend)),
      frontend_(std::move(frontend)),
      events_(events),
      clock_(std::move(clock)) {}  // thread_ is started by create(), on the fully built object

LioRunner::~LioRunner() {
  stop();
  // Design table "Shutdown": joined, then the backend, the frontend and (as the first member) the channels.
  backend_.reset();
  frontend_.reset();
}

Result<std::unique_ptr<LioRunner>, config::ConfigError> LioRunner::create(const LioConfig& config,
                                                                          const Eigen::Isometry3d& base_T_imu,
                                                                          const Eigen::Isometry3d& imu_T_lidar,
                                                                          events::EventRecorder& events,
                                                                          LioClock backend_clock) {
  if (!backend_clock) {
    return std::unexpected(config::ConfigError{config::ConfigError::Kind::kMissingKey, "backend_clock",
                                               "LioRunner::create: the backend clock is empty"});
  }
  auto channels = std::make_unique<LioChannels>();  // heap: the queues hold their slots inline
  auto backend = LioBackend::create(backend_config(config), base_T_imu, imu_T_lidar, *channels, events);
  if (!backend) return std::unexpected(backend.error());
  auto frontend = std::make_unique<LioFrontend>(frontend_config(config), base_T_imu, *channels, events);
  std::unique_ptr<LioRunner> runner(new LioRunner(std::move(channels), std::move(*backend), std::move(frontend),
                                                  events, std::move(backend_clock)));
  // The one thread starts only now, on a fully built runner: the member order cannot expose a half-built object.
  // A std::system_error here destroys the runner (stop() on a thread that never ran is a no-op).
  LioRunner* const self = runner.get();
  runner->thread_ = std::jthread([self](std::stop_token stop) { self->run_backend(std::move(stop)); });
  return runner;
}

void LioRunner::run_backend(std::stop_token stop) noexcept {
  GuardedClock guarded(clock_, events_);
  // Captures one reference: held in std::function's local buffer, no allocation.
  const std::function<time::TimeSnapshot()> clock = [&guarded] { return guarded(); };
  bool std_exception = false;
  try {
    backend_->run(std::move(stop), clock);
    guarded.flush();
    return;
  } catch (const std::exception&) {
    std_exception = true;
  } catch (...) {
  }
  // AGENTS.md §2.6: never silent. The reason reaches the recorder (and a flush is attempted) before the process
  // is terminated, as an escaped exception would have done anyway (std::jthread).
  guarded.flush();
  (void)events_.emit(events::EventBuilder(events::Component::kLio, "LioBackendFailed", guarded.last())
                         .reason(EstimatorEventReason::kBackendFailed)
                         .value("std_exception", std_exception ? 1.0 : 0.0)
                         .build());
  try {
    events_.flush();
  } catch (...) {  // terminating anyway
  }
  std::terminate();
}

void LioRunner::stop() noexcept {
  if (!thread_.joinable()) return;
  (void)thread_.request_stop();
  thread_.join();
}

StepOutputs LioRunner::take_carried(StepOutputs&& later) {
  StepOutputs out = std::exchange(carried_, StepOutputs{});
  merge(out, std::move(later));
  return out;
}

Result<StepOutputs, EstimatorReason> LioRunner::push_imu(const ImuInput& imu, const time::TimeSnapshot& now) {
  auto r = frontend_->push_imu(imu, now);
  if (!r) return r;
  return take_carried(std::move(*r));
}

Result<StepOutputs, EstimatorReason> LioRunner::push_scan(ScanInput&& scan, const time::TimeSnapshot& now) {
  // Poll first: the frontend checks busy before it applies waiting results.
  merge(carried_, frontend_->poll_results(now));
  auto r = frontend_->push_scan(std::move(scan), now);
  if (!r) return r;  // carried_ goes out with the next successful call
  return take_carried(std::move(*r));
}

StepOutputs LioRunner::poll_results(const time::TimeSnapshot& now) { return take_carried(frontend_->poll_results(now)); }

}  // namespace uavnav::lio
