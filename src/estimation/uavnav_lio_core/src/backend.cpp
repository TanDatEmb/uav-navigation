#include "uavnav/lio/backend.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "backend_math.hpp"
#include "fast_lio_core/deskew/scan_deskewer.hpp"
#include "fast_lio_core/estimation/ikfom_estimator.hpp"
#include "fast_lio_core/geometry/frame_ids.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"
#include "fast_lio_core/initialization/imu_initializer.hpp"
#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"
#include "fast_lio_core/mapping/local_map_manager.hpp"
#include "fast_lio_core/preprocessing/point_cloud_preprocessor.hpp"
#include "uavnav/core/event_builder.hpp"
#include "uavnav/lio/degeneracy.hpp"
#include "uavnav/lio/limits.hpp"

// Moved from the S1a facade (estimator.cpp): IMU initialisation, prediction, rebase, deskew, preprocessing,
// correction, degeneracy, map insert/crop, base_link odometry and the restart seeding; the stateless
// fast_lio_core adapters live in backend_math.hpp. The facade's reads of the lifecycle are replaced by data
// decided at admission (ScanJob::map_policy, ScanJob::epoch) and by the map size (bootstrap); its predictor
// updates become results (kImuInitialized, kScanGood, kRestartSeeded).
namespace uavnav::lio {

namespace {

namespace fl = uav::nav::lio;
namespace bm = backend_math;
using bm::snapshot;
using bm::stamp;
using events::Component;
using events::EventBuilder;
using Reason = EstimatorEventReason;

double seconds(time::SensorTime t) { return static_cast<double>(t.ns) / 1e9; }
bool usable(const Eigen::Quaterniond& q) { return q.coeffs().allFinite() && q.norm() > 1e-9; }

/// What one processed scan amounted to.
struct ScanOutcome {
  Reason reason;
  std::optional<DegeneracyReport> report;  // set when a correction was attempted
  std::vector<Eigen::Vector3d> points;     // preprocessed points (LiDAR frame), for the map
};

/// Prediction/deskew failures and math exceptions are degenerate scans (S1a rule).
BackendResultKind kind_of(Reason r) {
  if (r == Reason::kScanGood) return BackendResultKind::kScanGood;
  if (r == Reason::kScanEmpty) return BackendResultKind::kScanEmpty;
  if (r == Reason::kMapBootstrap) return BackendResultKind::kMapReady;
  return BackendResultKind::kScanDegenerate;
}

}  // namespace

struct LioBackend::Impl {
  Impl(const LioConfig& c, const fl::RigidTransform& base_to_imu, const Eigen::Quaterniond& q_il, LioChannels& ch,
       events::EventRecorder& ev)
      : cfg(c),
        channels(ch),
        events(ev),
        q_imu_lidar(q_il),
        t_imu_lidar(c.math.t_imu_lidar_m),
        initializer(fl::ImuInitializerConfig{.minimum_imu_samples = c.math.imu_init_min_samples}),
        eskf(fl::IkfomEstimatorConfig{.maximum_iterations = c.math.registration_max_iterations,
                                      .maximum_integration_step_ns = c.math.imu_max_gap.ns},
             // The only OpenMP team of the LIO: forked and joined inside one correction (§3.5).
             fl::ResidualBuilderConfig{.parallel_thread_count = limits::kIcpThreads}),
        deskewer(fl::ScanDeskewerConfig{fl::DeskewMode::kAuto, fl::DeskewReference::kScanEnd}),
        preprocessor(fl::PointCloudPreprocessorConfig{
            fl::PointFilterConfig{c.math.preprocess_min_range_m, c.math.preprocess_max_range_m},
            fl::VoxelFilterConfig{c.math.preprocess_voxel_m}, /*enable_voxel_filter=*/true}),
        map(fl::IkdTreeRegistrationMapConfig{c.math.map_voxel_m, 0.5, 0.6, /*enable_asynchronous_rebuild=*/false}),
        local_map(fl::LocalMapManagerConfig{.half_extent_m = c.math.map_half_extent_m}),
        prior_applicator(base_to_imu),
        converter(base_to_imu),
        projector(base_to_imu),
        q_base_imu(base_to_imu.rotation()) {}

  EventBuilder event(std::string_view name, const time::TimeSnapshot& now) const {
    events::EventIdentity id;
    id.lio_epoch = epoch;
    EventBuilder b(Component::kLio, name, now);
    b.identity(id);
    return b;
  }
  void emit(const EventBuilder& b) { (void)events.emit(b.build()); }

  BackendResult result(BackendResultKind kind, std::uint32_t result_epoch, std::uint64_t seq,
                       time::SensorTime t) const {
    return BackendResult{kind, result_epoch, seq, t, std::nullopt, std::nullopt, position_sigma_m, std::nullopt};
  }

  /// Every result goes through here. A full result queue (impossible by construction, limits.hpp) drops the
  /// result with an event and never blocks: the frontend then misses that answer and fails closed.
  void publish(BackendResult&& r, const time::TimeSnapshot& now) {
    const BackendResultKind kind = r.kind;
    const std::uint64_t seq = r.seq;
    const time::SensorTime t = r.t;
    if (channels.results.try_push(std::move(r)).has_value()) return;
    ++stats.results_dropped;
    emit(event("LioQueueOverflow", now)
             .reason(Reason::kResultQueueFull)
             .value("kind", static_cast<double>(kind))
             .value("seq", static_cast<double>(seq))
             .value("sensor_time_s", seconds(t)));
  }

  void refresh_sigma() {
    const fl::ManifoldState::Covariance p = eskf.covariance();
    position_sigma_m = std::sqrt(p.diagonal().head<3>().maxCoeff());  // NaN stays NaN: lifecycle fails closed
  }

  /// After a failed prediction or a math exception: keep the state, move its time to `end` and inflate the
  /// covariance (bm::inflated_for_rebase), as main's pipeline did.
  void rebase_to(time::SensorTime start, time::SensorTime end, const time::TimeSnapshot& now) {
    eskf.rebase(eskf.stateView(), bm::inflated_for_rebase(eskf.covariance()));
    eskf_t = end;
    refresh_sigma();
    emit(event("EskfRebased", now)
             .reason(Reason::kEskfRebased)
             .value("sensor_time_s", seconds(end))
             .value("skipped_s", time::to_seconds(end - start))
             .value("position_sigma_m", position_sigma_m));
  }

  /// Keeps the newest sample at or before t (the next prediction's start bracket) and everything after.
  void trim_history(time::SensorTime t) {
    while (imu_history.size() >= 2 && imu_history[1].time.nanoseconds() <= t.ns) imu_history.pop_front();
  }

  // --- IMU -----------------------------------------------------------------------------------------
  void on_imu(const ImuInput& imu, const time::TimeSnapshot& now) {
    ++stats.imu_samples;
    const fl::ImuSample sample{stamp(imu.t), imu.gyro_rad_s, imu.accel_mps2};
    try {
      imu_history.push_back(sample);
    } catch (...) {  // never throws: the sample is dropped (a later prediction across it fails, rebases)
      emit(event("AllocationFailed", now).reason(Reason::kAllocationFailed).value("site", 0));
    }
    while (imu_history.size() > limits::kImuHistoryCapacity) imu_history.pop_front();
    if (!eskf_t) try_initialize(sample, imu.t, now);
  }

  /// Stationary gravity + bias initialisation (fast_lio_core ImuInitializer); answers kImuInitialized.
  void try_initialize(const fl::ImuSample& s, time::SensorTime t, const time::TimeSnapshot& now) {
    try {
      if (!initializer.addSample(s).ok() || !initializer.hasEnoughSamples()) return;
      const auto init = initializer.tryInitialize();
      if (!init.ok()) {
        if (!init_rejection_reported) {
          init_rejection_reported = true;  // once, not per sample (only epoch 1 initialises from the IMU)
          emit(event("ImuInitRejected", now).reason(Reason::kImuInitRejected).value("sensor_time_s", seconds(t)));
        }
        return;
      }
      fl::ManifoldState st;
      st.set_orientation_odom_imu(init.value().orientation_odom_imu);
      st.set_gyro_bias_rad_s(init.value().gyro_bias_rad_s);
      st.set_accel_bias_m_s2(init.value().accel_bias_m_s2);
      st.set_gravity_odom_m_s2(init.value().gravity_odom_m_s2);
      st.set_rotation_imu_lidar(q_imu_lidar);
      st.set_position_imu_lidar_m(t_imu_lidar);
      st.normalize();
      eskf.initialize(st);
      eskf_t = t;
      refresh_sigma();
      initializer.reset();
      trim_history(t);
      emit(event("ImuInitialized", now)
               .reason(Reason::kImuInitialized)
               .value("sensor_time_s", seconds(t))
               .value("gyro_bias_norm_rad_s", init.value().gyro_bias_rad_s.norm())
               .value("accel_bias_norm_mps2", init.value().accel_bias_m_s2.norm()));
      BackendResult r = result(BackendResultKind::kImuInitialized, epoch, 0, t);
      r.snapshot = snapshot(st, t);
      publish(std::move(r), now);
    } catch (...) {
      eskf_t.reset();
      emit(event("ImuInitRejected", now).reason(Reason::kMathException));
    }
  }

  // --- scan ----------------------------------------------------------------------------------------
  /// ESKF prediction from eskf_t to `end` over the IMU history (the FIFO delivered the IMU up to `end`).
  /// nullopt (state unchanged) when the history does not cover the interval or holds an IMU gap.
  std::optional<fl::ImuTrajectory> predict_to(time::SensorTime end) {
    const time::SensorTime start = *eskf_t;
    const std::vector<fl::ImuSample> span = bm::prediction_span(imu_history, start, end);
    if (span.empty()) return std::nullopt;
    auto trajectory = eskf.predict(span, stamp(start), stamp(end));
    if (!trajectory.ok()) return std::nullopt;
    eskf_t = end;
    trim_history(end);
    return std::move(trajectory).value();
  }

  /// predict -> deskew -> preprocess -> (map bootstrap | correct -> degeneracy). Throws only from the
  /// reused math; the caller turns that into kMathException.
  ScanOutcome process(ScanInput& in, const time::TimeSnapshot& now) {
    ScanOutcome o{Reason::kScanDegenerate, std::nullopt, {}};
    const time::SensorTime trajectory_start = *eskf_t;
    const auto trajectory = predict_to(in.end);
    if (!trajectory) {
      rebase_to(trajectory_start, in.end, now);
      trim_history(in.end);
      o.reason = Reason::kPredictionFailed;
      return o;
    }
    fl::LidarScan scan;
    scan.start_time = stamp(in.start);
    scan.end_time = stamp(in.end);
    scan.has_per_point_time = in.per_point_time;
    scan.points = std::move(in.points);
    if (in.per_point_time && in.start < trajectory_start) bm::drop_points_before(scan, in.start, trajectory_start);
    if (scan.points.empty()) {
      o.reason = Reason::kScanEmpty;
      return o;
    }
    const fl::RigidTransform imu_T_lidar(fl::imuFrame(), fl::lidarFrame(), q_imu_lidar, t_imu_lidar);
    const auto deskewed = deskewer.deskew(scan, *trajectory, imu_T_lidar);
    if (!deskewed.ok()) {
      o.reason = Reason::kDeskewFailed;
      return o;
    }
    const auto pre = preprocessor.process(deskewed.value().scan);
    if (!pre.ok()) {  // only fails when nothing survives the range / voxel filters (input already validated)
      o.reason = Reason::kScanEmpty;
      return o;
    }
    o.points.reserve(pre.value().scan.points.size());
    for (const fl::LidarPoint& p : pre.value().scan.points) o.points.push_back(p.position_lidar_m.cast<double>());

    if (map.size() == 0U) {
      o.reason = Reason::kMapBootstrap;  // first processed scan of the epoch: it becomes the map
      return o;
    }
    const fl::IkfomCorrectionResult c = eskf.correct(o.points, map);
    o.report = evaluate_degeneracy(c.information, cfg.degeneracy);
    o.reason = c.successful && !o.report->degenerate ? Reason::kScanGood : Reason::kScanDegenerate;
    return o;
  }

  void insert_into_map(const std::vector<Eigen::Vector3d>& points_lidar, const time::TimeSnapshot& now) {
    try {
      if (!local_map.insertionAllowed()) return;  // fast_lio_core's emergency point guard froze the map
      const fl::ManifoldState s = eskf.stateView();
      std::vector<Eigen::Vector3d> world;
      world.reserve(points_lidar.size());
      for (const auto& p : points_lidar) world.push_back(s.transformLidarPointToOdom(p));
      (void)map.insert(world);
      (void)local_map.update(map, s.position_odom_imu_m());
    } catch (...) {
      emit(event("LioScan", now).reason(Reason::kMathException));
    }
  }

  /// Corrected state at the scan end, at base_link; an OdometryInvalid event when the conversion fails.
  std::optional<BaseOdometry> odometry(time::SensorTime t, const time::TimeSnapshot& now) {
    try {
      const fl::StateEstimate est{stamp(t), eskf.stateView(), eskf.covariance()};
      // The IMU sample at or before t (trim_history keeps it first) gives the angular rate.
      const Eigen::Vector3d omega =
          imu_history.empty()
              ? Eigen::Vector3d::Zero()
              : Eigen::Vector3d(imu_history.front().angular_velocity_imu_rad_s - est.state.gyro_bias_rad_s());
      if (auto o = bm::base_odometry(converter, projector, est, omega)) return o;
    } catch (...) {
    }
    emit(event("OdometryInvalid", now).reason(Reason::kOdometryInvalid).value("sensor_time_s", seconds(t)));
    return std::nullopt;
  }

  void on_scan(ScanJob&& job, const time::TimeSnapshot& now) {
    ++stats.scans;
    const time::SensorTime t = job.scan.end;
    const std::size_t input_points = job.scan.points.size();
    BackendResult r = result(BackendResultKind::kScanNotProcessed, job.epoch, job.seq, t);
    if (job.epoch != epoch || !eskf_t || *eskf_t >= t) {  // another epoch, nothing to predict from, or too old
      const Reason why = job.epoch != epoch ? Reason::kStaleEpoch
                         : eskf_t           ? Reason::kBeforeEstimatorTime
                                            : Reason::kBeforeImuInit;
      emit(event("LioScan", now)
               .reason(why)
               .value("sensor_time_s", seconds(t))
               .value("scan_epoch", static_cast<double>(job.epoch)));
      publish(std::move(r), now);
      return;
    }

    ScanOutcome o{Reason::kMathException, std::nullopt, {}};
    const time::SensorTime start = *eskf_t;
    const fl::ManifoldState saved_state = eskf.stateView();
    const fl::ManifoldState::Covariance saved_cov = eskf.covariance();
    try {
      o = process(job.scan, now);
    } catch (...) {  // reused math threw: restore, rebase, degenerate scan + event; never an exception out
      try {
        eskf.rebase(saved_state, saved_cov);
        rebase_to(start, t, now);
      } catch (...) {
        eskf_t = t;
      }
    }
    refresh_sigma();

    EventBuilder scan_event = event("LioScan", now);
    scan_event.reason(o.reason)
        .value("sensor_time_s", seconds(t))
        .value("input_points", static_cast<double>(input_points))
        .value("points", static_cast<double>(o.points.size()));
    if (o.report) {
      scan_event.value("translation_min_eigenvalue", o.report->translation_min_eigenvalue)
          .value("rotation_min_eigenvalue", o.report->rotation_min_eigenvalue)
          .value("quality", o.report->quality)
          .value("degenerate", o.report->degenerate ? 1.0 : 0.0);
    }
    emit(scan_event);

    const bool good = o.reason == Reason::kScanGood;
    const bool insert = o.reason == Reason::kMapBootstrap || (good && job.map_policy == MapPolicy::kInsert);
    if (insert) insert_into_map(o.points, now);
    r.kind = kind_of(o.reason);
    r.snapshot = snapshot(eskf.stateView(), t);
    r.report = o.report;
    r.position_sigma_m = position_sigma_m;
    if (good) r.odometry = odometry(t, now);
    publish(std::move(r), now);
  }

  // --- restart -------------------------------------------------------------------------------------
  /// Seeds the ESKF at c.t in c.new_epoch (§3.4): clears the map and the local map, resets the ESKF, switches
  /// the epoch. Rejected (own epoch and state unchanged) before the IMU initialisation, for a non-finite
  /// command, or when the applicator refuses or throws.
  void on_restart(const RestartCommand& c, const time::TimeSnapshot& now) {
    BackendResult r = result(BackendResultKind::kRestartRejected, c.new_epoch, 0, c.t);
    const bool finite = c.seed.p_world_m.allFinite() && usable(c.seed.q_world_base) && usable(c.q_world_imu_old) &&
                        c.v_world_mps_old.allFinite();
    try {
      const std::optional<fl::ManifoldState> seeded =
          eskf_t && finite ? bm::seed_state(prior_applicator, eskf.stateView(), c, q_base_imu) : std::nullopt;
      if (seeded) {
        map.clear();
        local_map.reset();
        eskf.reset(*seeded);
        eskf_t = c.t;
        trim_history(c.t);
        refresh_sigma();
        epoch = c.new_epoch;
        ++stats.restarts;
        r.kind = BackendResultKind::kRestartSeeded;
        r.snapshot = snapshot(*seeded, c.t);
        r.position_sigma_m = position_sigma_m;
      }
    } catch (...) {  // the applicator or the ESKF reset threw: rejected, the frontend stays in RESTARTING
      r = result(BackendResultKind::kRestartRejected, c.new_epoch, 0, c.t);
    }
    publish(std::move(r), now);
  }

  LioConfig cfg;
  LioChannels& channels;
  events::EventRecorder& events;
  Eigen::Quaterniond q_imu_lidar;
  Eigen::Vector3d t_imu_lidar;

  fl::ImuInitializer initializer;
  fl::IkfomEstimator eskf;
  fl::ScanDeskewer deskewer;
  fl::PointCloudPreprocessor preprocessor;
  fl::IkdTreeRegistrationMap map;
  fl::LocalMapManager local_map;
  fl::InitialStatePriorApplicator prior_applicator;
  fl::BaseLinkStateConverter converter;
  fl::BaseLinkCovarianceProjector projector;
  Eigen::Quaterniond q_base_imu;

  std::deque<fl::ImuSample> imu_history;   // bounded by limits::kImuHistoryCapacity
  std::optional<time::SensorTime> eskf_t;  // ESKF state time; nullopt until the IMU initialiser succeeds
  BackendStats stats{0, 0, 0, 0, 0};       // map_points is read from the map in stats()
  double position_sigma_m{0.0};
  bool init_rejection_reported{false};  // ImuInitRejected latch
  std::uint32_t epoch{1};
};

// =====================================================================================================

LioBackend::LioBackend(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
LioBackend::~LioBackend() = default;

Result<std::unique_ptr<LioBackend>, config::ConfigError> LioBackend::create(const LioConfig& config,
                                                                            const Eigen::Isometry3d& base_T_imu,
                                                                            const Eigen::Isometry3d& imu_T_lidar,
                                                                            LioChannels& channels,
                                                                            events::EventRecorder& events) {
  using Kind = config::ConfigError::Kind;
  if (auto checked = bm::check_frames(config, base_T_imu, imu_T_lidar); !checked) {
    return std::unexpected(checked.error());
  }
  try {
    const fl::RigidTransform base_to_imu(fl::baseFrame(), fl::imuFrame(),
                                         Eigen::Quaterniond(base_T_imu.linear()).normalized(),
                                         base_T_imu.translation());
    auto impl = std::make_unique<Impl>(config, base_to_imu, Eigen::Quaterniond(imu_T_lidar.linear()).normalized(),
                                       channels, events);
    return std::unique_ptr<LioBackend>(new LioBackend(std::move(impl)));
  } catch (const std::exception& e) {  // a fast_lio_core component refused its configuration
    return std::unexpected(config::ConfigError{Kind::kOutOfRange, "", e.what()});
  } catch (...) {
    return std::unexpected(config::ConfigError{Kind::kOutOfRange, "", "unknown exception from fast_lio_core"});
  }
}

bool LioBackend::step(const time::TimeSnapshot& now) {
  std::optional<BackendRequest> request = impl_->channels.requests.try_pop();
  if (!request) return false;
  if (const auto* imu = std::get_if<ImuInput>(&*request)) impl_->on_imu(*imu, now);
  if (auto* job = std::get_if<ScanJob>(&*request)) impl_->on_scan(std::move(*job), now);
  if (const auto* restart = std::get_if<RestartCommand>(&*request)) impl_->on_restart(*restart, now);
  return true;
}

void LioBackend::run(std::stop_token stop, const std::function<time::TimeSnapshot()>& clock) {
  while (!stop.stop_requested()) {
    impl_->channels.requests.wait_nonempty(stop);
    // wait_nonempty also returns on a non-empty queue after a stop request: stop wins, the rest stays queued.
    if (stop.stop_requested()) return;
    (void)step(clock());
  }
}

BackendStats LioBackend::stats() const noexcept {
  BackendStats s = impl_->stats;
  try {
    s.map_points = impl_->map.size();
  } catch (...) {  // stats() must not throw; the vendor tree's size does not in practice
    s.map_points = 0;
  }
  return s;
}

}  // namespace uavnav::lio
