#include "uavnav/lio/estimator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "fast_lio_core/deskew/scan_deskewer.hpp"
#include "fast_lio_core/estimation/ikfom_estimator.hpp"
#include "fast_lio_core/estimation/state_estimate.hpp"
#include "fast_lio_core/geometry/frame_ids.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"
#include "fast_lio_core/initialization/imu_initializer.hpp"
#include "fast_lio_core/initialization/initial_state_prior.hpp"
#include "fast_lio_core/initialization/initial_state_prior_applicator.hpp"
#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"
#include "fast_lio_core/mapping/local_map_manager.hpp"
#include "fast_lio_core/navigation/base_link_covariance_projector.hpp"
#include "fast_lio_core/navigation/base_link_state_converter.hpp"
#include "fast_lio_core/preprocessing/point_cloud_preprocessor.hpp"
#include "uavnav/core/event_builder.hpp"
#include "uavnav/lio/degeneracy.hpp"
#include "uavnav/lio/limits.hpp"

// Call order per scan follows the deleted main pipeline (fast_lio_pipeline.cpp): predict -> deskew ->
// preprocess -> correct -> map insert/crop. Its lifecycle and tracking code is NOT reused: LioLifecycle
// owns the state (single writer), and the facade only classifies each scan and feeds it.
namespace uavnav::lio {

namespace {

namespace fl = uav::nav::lio;
using events::Component;
using events::EventBuilder;

/// imu_T_lidar's translation and the config's extrinsic_imu_lidar_*_m must agree to this (one source).
constexpr double kExtrinsicToleranceM = 1e-6;
/// Weight of the newest dt in the IMU period average used for the ImuRateTooHigh check.
constexpr double kImuPeriodEmaWeight = 0.1;
/// In-flight restart: the seed attitude is taken as is (§3.4: T^-1 x PX4 pose); this tilt limit is not
/// used by InitialStatePriorApplicator for the in-flight context.
constexpr double kUnusedTiltLimitRad = 0.0;
/// LioInputRejected "stream" value.
enum class Stream : int { kImu = 0, kScan = 1, kRestart = 2 };

fl::Timestamp stamp(time::SensorTime t) { return fl::Timestamp(t.ns); }
double seconds(time::SensorTime t) { return static_cast<double>(t.ns) / 1e9; }

double yaw_of(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d r = q.toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}

bool usable(const Eigen::Quaterniond& q) { return q.coeffs().allFinite() && q.norm() > 1e-9; }

EstimatorSnapshot snapshot(const fl::ManifoldState& s, time::SensorTime t) {
  return EstimatorSnapshot{t,
                           s.orientation_odom_imu(),
                           s.velocity_odom_imu_m_s(),
                           s.position_odom_imu_m(),
                           s.gyro_bias_rad_s(),
                           s.accel_bias_m_s2(),
                           s.gravity_odom_m_s2()};
}

/// What one scan amounted to, before the lifecycle sees it.
struct ScanOutcome {
  EstimatorEventReason reason;
  std::optional<DegeneracyReport> report;  // set when a correction was attempted
  std::vector<Eigen::Vector3d> points;     // preprocessed points (LiDAR frame), for the map
  std::size_t input_points{0};
};

LioEvent::Kind kind_of(EstimatorEventReason r) {
  switch (r) {
    case EstimatorEventReason::kScanGood:
      return LioEvent::Kind::kScanGood;
    case EstimatorEventReason::kScanEmpty:
      return LioEvent::Kind::kScanEmpty;
    case EstimatorEventReason::kMapBootstrap:
      return LioEvent::Kind::kMapReady;
    default:
      return LioEvent::Kind::kScanDegenerate;  // prediction/deskew failure, math exception
  }
}

}  // namespace

struct LioEstimator::Impl {
  Impl(const LioConfig& c, const fl::RigidTransform& base_to_imu, const Eigen::Quaterniond& q_il,
       events::EventRecorder& ev)
      : cfg(c),
        events(ev),
        q_imu_lidar(q_il),
        t_imu_lidar(c.math.t_imu_lidar_m),
        initializer(initializer_config(c)),
        eskf(ikfom_config(c), fl::ResidualBuilderConfig{}),
        deskewer(fl::ScanDeskewerConfig{fl::DeskewMode::kAuto, fl::DeskewReference::kScanEnd}),
        preprocessor(preprocessor_config(c)),
        map(fl::IkdTreeRegistrationMapConfig{c.math.map_voxel_m, 0.5, 0.6, /*enable_asynchronous_rebuild=*/false}),
        local_map(local_map_config(c)),
        prior_applicator(base_to_imu),
        converter(base_to_imu),
        projector(base_to_imu),
        q_base_imu(base_to_imu.rotation()),
        lifecycle(c.lifecycle),
        predictor(c.predictor) {}

  static fl::ImuInitializerConfig initializer_config(const LioConfig& c) {
    fl::ImuInitializerConfig i{};
    i.minimum_imu_samples = c.math.imu_init_min_samples;
    return i;
  }
  static fl::IkfomEstimatorConfig ikfom_config(const LioConfig& c) {
    fl::IkfomEstimatorConfig k{};
    k.maximum_iterations = c.math.registration_max_iterations;
    k.maximum_integration_step_ns = c.math.imu_max_gap.ns;
    return k;
  }
  static fl::PointCloudPreprocessorConfig preprocessor_config(const LioConfig& c) {
    fl::PointCloudPreprocessorConfig p{};
    p.point_filter = fl::PointFilterConfig{c.math.preprocess_min_range_m, c.math.preprocess_max_range_m};
    p.voxel_filter = fl::VoxelFilterConfig{c.math.preprocess_voxel_m};
    p.enable_voxel_filter = true;
    return p;
  }
  static fl::LocalMapManagerConfig local_map_config(const LioConfig& c) {
    fl::LocalMapManagerConfig l{};
    l.half_extent_m = c.math.map_half_extent_m;
    return l;
  }

  // --- events ------------------------------------------------------------------------------------
  EventBuilder event(std::string_view name, const time::TimeSnapshot& now) const {
    events::EventIdentity id;
    id.lio_epoch = epoch;
    EventBuilder b(Component::kLio, name, now);
    b.identity(id);
    return b;
  }
  void emit(const EventBuilder& b) { (void)events.emit(b.build()); }

  Result<StepOutputs, EstimatorReason> reject(EstimatorReason r, Stream s, time::SensorTime t,
                                              const time::TimeSnapshot& now) {
    emit(event("LioInputRejected", now)
             .reason(r)
             .value("stream", static_cast<int>(s))
             .value("sensor_time_s", seconds(t)));
    return std::unexpected(r);
  }

  // --- shared helpers ------------------------------------------------------------------------------
  void refresh_sigma() {
    const fl::ManifoldState::Covariance p = eskf.covariance();
    position_sigma_m = std::sqrt(p.diagonal().head<3>().maxCoeff());  // NaN stays NaN: lifecycle fails closed
  }

  HealthOutput health(time::SensorTime t) const {
    double age = 0.0;
    if (last_scan_event_t && last_imu) age = std::max(0.0, time::to_seconds(last_imu->t - *last_scan_event_t));
    return HealthOutput{t,
                        lifecycle.state(),
                        epoch,
                        last_reason,
                        last_report.translation_min_eigenvalue,
                        last_report.rotation_min_eigenvalue,
                        age,
                        predictor.tracking_error()};
  }

  /// One StateTransition event per change, and the health output of that call.
  void on_transition(const Transition& tr, time::SensorTime t, const time::TimeSnapshot& now, StepOutputs* out,
                     const std::optional<DegeneracyReport>& report) {
    if (!tr.changed) return;
    last_reason = tr.reason;
    EventBuilder b = event("StateTransition", now);
    b.states(tr.before, tr.after).reason(tr.reason).value("sensor_time_s", seconds(t));
    b.value("position_sigma_m", position_sigma_m);
    if (report) {
      b.value("quality", report->quality)
          .value("translation_min_eigenvalue", report->translation_min_eigenvalue)
          .value("rotation_min_eigenvalue", report->rotation_min_eigenvalue);
    }
    emit(b);
    if (out != nullptr) out->health = health(t);
  }

  /// Applies an ESKF snapshot to the output predictor. A snapshot newer than the output (the IMU has not
  /// reached the scan end yet) waits for push_imu; every other rejection is one event. Persistent
  /// rejection -> DEGRADED is an S1b carry-over (not decided here).
  void correct_predictor(const EstimatorSnapshot& s, const time::TimeSnapshot& now) {
    const auto r = predictor.on_correction(s);
    if (r) return;
    if (r.error() == PredictorReason::kNewerThanOutput) {
      if (pending_correction) {
        emit(event("PredictorCorrectionRejected", now).reason(PredictorReason::kNewerThanOutput));
      }
      pending_correction = s;
      return;
    }
    emit(event("PredictorCorrectionRejected", now).reason(r.error()).value("sensor_time_s", seconds(s.t)));
  }

  // --- IMU -----------------------------------------------------------------------------------------
  void track_rate(time::Duration dt, const time::TimeSnapshot& now) {
    if (dt.ns <= 0 || dt > limits::kPredictorBufferSpan) return;
    const double dt_s = time::to_seconds(dt);
    imu_period_s = imu_period_samples == 0 ? dt_s : imu_period_s + kImuPeriodEmaWeight * (dt_s - imu_period_s);
    ++imu_period_samples;
    if (rate_reported || imu_period_samples < limits::kPredictorBufferCapacity) return;
    // The predictor buffer must span kPredictorBufferSpan (§3.3: up to 300 ms scan delay).
    if (static_cast<double>(limits::kPredictorBufferCapacity) * imu_period_s <
        time::to_seconds(limits::kPredictorBufferSpan)) {
      rate_reported = true;  // once per epoch
      emit(event("ImuRateTooHigh", now)
               .reason(EstimatorEventReason::kImuRateTooHigh)
               .value("imu_period_s", imu_period_s));
    }
  }

  /// Stationary gravity + bias initialisation (fast_lio_core ImuInitializer); aligns the predictor.
  void try_initialize(const fl::ImuSample& s, time::SensorTime t, const time::TimeSnapshot& now) {
    try {
      if (!initializer.addSample(s).ok() || !initializer.hasEnoughSamples()) return;
      const auto init = initializer.tryInitialize();
      if (!init.ok()) {
        if (!init_rejection_reported) {
          init_rejection_reported = true;  // once, not per sample (only epoch 1 initialises from the IMU)
          emit(event("ImuInitRejected", now)
                   .reason(EstimatorEventReason::kImuInitRejected)
                   .value("sensor_time_s", seconds(t)));
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
      predictor.align(snapshot(st, t));
      initializer.reset();
      trim_history(t);
      emit(event("ImuInitialized", now)
               .reason(EstimatorEventReason::kImuInitialized)
               .value("sensor_time_s", seconds(t))
               .value("gyro_bias_norm_rad_s", init.value().gyro_bias_rad_s.norm())
               .value("accel_bias_norm_mps2", init.value().accel_bias_m_s2.norm()));
    } catch (...) {
      eskf_t.reset();
      emit(event("ImuInitRejected", now).reason(EstimatorEventReason::kMathException));
    }
  }

  /// Keeps the newest sample at or before t (the next prediction's start bracket) and everything after.
  void trim_history(time::SensorTime t) {
    while (imu_history.size() >= 2 && imu_history[1].time.nanoseconds() <= t.ns) imu_history.pop_front();
  }

  // --- scan ----------------------------------------------------------------------------------------
  /// ESKF prediction from eskf_t to `end` over the IMU history. The last sample may be held (zero-order)
  /// up to imu_max_gap when the IMU has not reached `end` yet. On failure (history does not cover the
  /// interval, or an IMU gap) the state is kept and its time moved to `end` (no integration).
  std::optional<fl::ImuTrajectory> predict_to(time::SensorTime end) {
    const time::SensorTime start = *eskf_t;
    eskf_t = end;
    std::vector<fl::ImuSample> span;
    for (std::size_t i = 0; i < imu_history.size(); ++i) {
      const std::int64_t ti = imu_history[i].time.nanoseconds();
      const bool start_bracket =
          ti <= start.ns && (i + 1 == imu_history.size() || imu_history[i + 1].time.nanoseconds() > start.ns);
      if (start_bracket || (!span.empty() && ti > start.ns)) span.push_back(imu_history[i]);
      if (!span.empty() && ti >= end.ns) break;
    }
    if (!span.empty() && span.back().time.nanoseconds() < end.ns &&
        end - time::SensorTime{span.back().time.nanoseconds()} <= cfg.math.imu_max_gap) {
      fl::ImuSample held = span.back();
      held.time = stamp(end);
      span.push_back(held);
    }
    trim_history(end);
    if (span.empty() || span.front().time.nanoseconds() > start.ns || span.back().time.nanoseconds() < end.ns) {
      return std::nullopt;
    }
    auto trajectory = eskf.predict(span, stamp(start), stamp(end));
    if (!trajectory.ok()) return std::nullopt;
    return std::move(trajectory).value();
  }

  /// predict -> deskew -> preprocess -> (map bootstrap | correct -> degeneracy). Throws only from the
  /// reused math; the caller turns that into kMathException.
  ScanOutcome process(ScanInput& in) {
    ScanOutcome o{EstimatorEventReason::kScanDegenerate, std::nullopt, {}, in.points.size()};
    const time::SensorTime trajectory_start = *eskf_t;
    const auto trajectory = predict_to(in.end);
    if (!trajectory) {
      o.reason = EstimatorEventReason::kPredictionFailed;
      return o;
    }
    fl::LidarScan scan;
    scan.start_time = stamp(in.start);
    scan.end_time = stamp(in.end);
    scan.has_per_point_time = in.per_point_time;
    scan.points = std::move(in.points);
    if (in.per_point_time && in.start < trajectory_start) {
      // Points stamped before the ESKF time cannot be deskewed (no trajectory there): drop them and
      // re-reference the rest to the ESKF time. Contiguous scans (start == previous end) lose nothing.
      const std::int64_t shift = (trajectory_start - in.start).ns;
      std::erase_if(scan.points,
                    [&](const fl::LidarPoint& p) { return static_cast<std::int64_t>(p.relative_time_ns) < shift; });
      // Every kept point has relative_time_ns >= shift, so the difference fits the uint32 field.
      for (auto& p : scan.points) p.relative_time_ns = static_cast<std::uint32_t>(p.relative_time_ns - shift);
      scan.start_time = stamp(trajectory_start);
    }
    if (scan.points.empty()) {
      o.reason = EstimatorEventReason::kScanEmpty;
      return o;
    }
    const fl::RigidTransform imu_T_lidar(fl::imuFrame(), fl::lidarFrame(), q_imu_lidar, t_imu_lidar);
    const auto deskewed = deskewer.deskew(scan, *trajectory, imu_T_lidar);
    if (!deskewed.ok()) {
      o.reason = EstimatorEventReason::kDeskewFailed;
      return o;
    }
    const auto pre = preprocessor.process(deskewed.value().scan);
    if (!pre.ok()) {  // only fails when nothing survives the range / voxel filters (input already validated)
      o.reason = EstimatorEventReason::kScanEmpty;
      return o;
    }
    o.points.reserve(pre.value().scan.points.size());
    for (const fl::LidarPoint& p : pre.value().scan.points) o.points.push_back(p.position_lidar_m.cast<double>());

    if (lifecycle.state() == LioState::kInitializing && map.size() == 0U) {
      o.reason = EstimatorEventReason::kMapBootstrap;  // first scan of the epoch: it becomes the map
      return o;
    }
    const fl::IkfomCorrectionResult c = eskf.correct(o.points, map);
    o.report = evaluate_degeneracy(c.information, cfg.degeneracy);
    o.reason =
        c.successful && !o.report->degenerate ? EstimatorEventReason::kScanGood : EstimatorEventReason::kScanDegenerate;
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
      emit(event("LioScan", now).reason(EstimatorEventReason::kMathException));
    }
  }

  /// Corrected state at the scan end, at base_link (fast_lio_core navigation converters).
  std::optional<OdometryOutput> odometry(time::SensorTime t, std::uint8_t quality, const time::TimeSnapshot& now) {
    try {
      const fl::StateEstimate est{stamp(t), eskf.stateView(), eskf.covariance()};
      // The IMU sample at or before t (trim_history keeps it first) gives the angular rate.
      const Eigen::Vector3d omega =
          imu_history.empty()
              ? Eigen::Vector3d::Zero()
              : Eigen::Vector3d(imu_history.front().angular_velocity_imu_rad_s - est.state.gyro_bias_rad_s());
      const auto base = converter.convert(est, omega);
      if (base.ok()) {
        const auto cov = projector.project(fl::KinematicStateEstimate{est, omega}, base.value());
        if (cov.ok()) {
          const Eigen::Matrix3d r = base.value().orientation_reference_body.toRotationMatrix();
          const Eigen::Matrix3d vel_cov = r * cov.value().twist_covariance_base.topLeftCorner<3, 3>() * r.transpose();
          OdometryOutput o{t,
                           epoch,
                           predictor.reset_counter(),
                           base.value().position_reference_body_m,
                           base.value().orientation_reference_body,
                           base.value().linear_velocity_reference_body_m_s,
                           cov.value().pose_covariance_odom,
                           vel_cov,
                           quality};
          if (o.p_world_m.allFinite() && o.q_world_base.coeffs().allFinite() && o.v_world_mps.allFinite() &&
              o.pose_cov.allFinite() && o.vel_cov.allFinite()) {
            return o;
          }
        }
      }
    } catch (...) {
    }
    emit(event("OdometryInvalid", now)
             .reason(EstimatorEventReason::kOdometryInvalid)
             .value("sensor_time_s", seconds(t)));
    return std::nullopt;
  }

  // --- data ----------------------------------------------------------------------------------------
  LioConfig cfg;
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
  LioLifecycle lifecycle;
  OutputPredictor predictor;

  std::deque<fl::ImuSample> imu_history;              // bounded by limits::kImuHistoryCapacity
  std::optional<ImuInput> last_imu;                   // newest accepted IMU sample
  std::optional<OutputSample> last_output;            // newest predictor output
  std::optional<time::SensorTime> eskf_t;             // ESKF state time; nullopt until the IMU initialiser succeeds
  std::optional<time::SensorTime> last_scan_end;      // newest accepted scan end (ordering check)
  std::optional<time::SensorTime> last_scan_event_t;  // newest scan end fed to the lifecycle
  std::optional<time::SensorTime> next_health_t;
  std::optional<EstimatorSnapshot> pending_correction;
  DegeneracyReport last_report{0.0, 0.0, true, 0};
  LioReason last_reason{LioReason::kNone};
  double position_sigma_m{0.0};
  double imu_period_s{0.0};
  std::size_t imu_period_samples{0};
  bool rate_reported{false};            // ImuRateTooHigh latch, reset per epoch
  bool init_rejection_reported{false};  // ImuInitRejected latch
  std::uint32_t epoch{1};
};

// =====================================================================================================

LioEstimator::LioEstimator(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
LioEstimator::~LioEstimator() = default;

Result<std::unique_ptr<LioEstimator>, config::ConfigError> LioEstimator::create(const LioConfig& config,
                                                                                const Eigen::Isometry3d& base_T_imu,
                                                                                const Eigen::Isometry3d& imu_T_lidar,
                                                                                events::EventRecorder& events) {
  using Kind = config::ConfigError::Kind;
  if (!base_T_imu.matrix().allFinite())
    return std::unexpected(config::ConfigError{Kind::kNotFinite, "base_T_imu", "not finite"});
  if (!imu_T_lidar.matrix().allFinite())
    return std::unexpected(config::ConfigError{Kind::kNotFinite, "imu_T_lidar", "not finite"});
  constexpr std::array<std::string_view, 3> kKeys{"extrinsic_imu_lidar_x_m", "extrinsic_imu_lidar_y_m",
                                                  "extrinsic_imu_lidar_z_m"};
  for (int a = 0; a < 3; ++a) {
    if (std::abs(imu_T_lidar.translation()[a] - config.math.t_imu_lidar_m[a]) > kExtrinsicToleranceM) {
      return std::unexpected(config::ConfigError{Kind::kOutOfRange, std::string(kKeys[static_cast<std::size_t>(a)]),
                                                 std::string(kKeys[static_cast<std::size_t>(a)]) +
                                                     " disagrees with imu_T_lidar (" +
                                                     std::to_string(config.math.t_imu_lidar_m[a]) + " vs " +
                                                     std::to_string(imu_T_lidar.translation()[a]) + ")"});
    }
  }
  try {
    const fl::RigidTransform base_to_imu(fl::baseFrame(), fl::imuFrame(),
                                         Eigen::Quaterniond(base_T_imu.linear()).normalized(),
                                         base_T_imu.translation());
    auto impl =
        std::make_unique<Impl>(config, base_to_imu, Eigen::Quaterniond(imu_T_lidar.linear()).normalized(), events);
    return std::unique_ptr<LioEstimator>(new LioEstimator(std::move(impl)));
  } catch (const std::exception& e) {  // a fast_lio_core component refused its configuration
    return std::unexpected(config::ConfigError{Kind::kOutOfRange, "", e.what()});
  } catch (...) {
    return std::unexpected(config::ConfigError{Kind::kOutOfRange, "", "unknown exception from fast_lio_core"});
  }
}

LioState LioEstimator::state() const noexcept { return impl_->lifecycle.state(); }
std::uint32_t LioEstimator::epoch() const noexcept { return impl_->epoch; }

Result<StepOutputs, EstimatorReason> LioEstimator::push_imu(const ImuInput& imu, const time::TimeSnapshot& now) {
  Impl& m = *impl_;
  if (!imu.gyro_rad_s.allFinite() || !imu.accel_mps2.allFinite()) {
    return m.reject(EstimatorReason::kNotFinite, Stream::kImu, imu.t, now);
  }
  if (m.last_imu && imu.t < m.last_imu->t) return m.reject(EstimatorReason::kOutOfOrder, Stream::kImu, imu.t, now);

  StepOutputs out;
  if (m.last_imu && imu.t == m.last_imu->t) {
    m.emit(m.event("ImuDuplicate", now)
               .reason(EstimatorEventReason::kImuDuplicate)
               .value("sensor_time_s", seconds(imu.t)));
  } else {
    if (m.last_imu) {
      const time::Duration dt = imu.t - m.last_imu->t;
      if (dt > m.cfg.math.imu_max_gap) {
        // Reported here; the LiDAR-gap rule (F22) is the lifecycle's kImuTick below, not duplicated.
        m.emit(m.event("ImuGap", now).reason(EstimatorEventReason::kImuGap).value("gap_s", time::to_seconds(dt)));
      }
      m.track_rate(dt, now);
    }
    const fl::ImuSample sample{stamp(imu.t), imu.gyro_rad_s, imu.accel_mps2};
    m.imu_history.push_back(sample);
    while (m.imu_history.size() > limits::kImuHistoryCapacity) m.imu_history.pop_front();

    if (!m.eskf_t) {
      m.try_initialize(sample, imu.t, now);
    } else if (m.last_imu) {
      const double dt_s = time::to_seconds(imu.t - m.last_imu->t);
      const ImuDelta delta{imu.t, 0.5 * (m.last_imu->gyro_rad_s + imu.gyro_rad_s) * dt_s,
                           0.5 * (m.last_imu->accel_mps2 + imu.accel_mps2) * dt_s, dt_s};
      if (const auto o = m.predictor.on_imu(delta)) {
        m.last_output = *o;
        if (m.pending_correction && m.pending_correction->t <= imu.t) {
          const EstimatorSnapshot s = *m.pending_correction;
          m.pending_correction.reset();
          m.correct_predictor(s, now);
        }
      }
    }
    m.last_imu = imu;
  }

  const Transition tr = m.lifecycle.on(LioEvent{LioEvent::Kind::kImuTick, imu.t, m.position_sigma_m});
  m.on_transition(tr, imu.t, now, &out, std::nullopt);
  if (m.last_output && m.last_output->t == imu.t)
    out.states.push_back(StateOutput{*m.last_output, m.epoch, m.lifecycle.state()});
  if (!m.next_health_t || imu.t >= *m.next_health_t) {
    out.health = m.health(imu.t);
    m.next_health_t = imu.t + limits::kHealthPeriod;
  }
  return out;
}

Result<StepOutputs, EstimatorReason> LioEstimator::push_scan(ScanInput&& in, const time::TimeSnapshot& now) {
  Impl& m = *impl_;
  if (in.points.size() > limits::kMaxScanPoints)
    return m.reject(EstimatorReason::kTooManyPoints, Stream::kScan, in.end, now);
  for (const auto& p : in.points) {
    if (!p.allFinite()) return m.reject(EstimatorReason::kNotFinite, Stream::kScan, in.end, now);
  }
  if (in.end < in.start || (m.last_scan_end && in.end < *m.last_scan_end)) {
    return m.reject(EstimatorReason::kOutOfOrder, Stream::kScan, in.end, now);
  }
  m.last_scan_end = in.end;

  StepOutputs out;
  const time::SensorTime t = in.end;
  if (!m.eskf_t || *m.eskf_t >= t) {  // nothing to predict from yet, or a scan older than a (re)start
    const auto why = m.eskf_t ? EstimatorEventReason::kBeforeEstimatorTime : EstimatorEventReason::kBeforeImuInit;
    m.emit(m.event("LioScan", now).reason(why).value("sensor_time_s", seconds(t)));
    return out;
  }

  ScanOutcome o{EstimatorEventReason::kMathException, std::nullopt, {}, in.points.size()};
  try {
    o = m.process(in);
  } catch (...) {  // reused math threw: a degenerate scan + event, never an exception out
    m.eskf_t = t;
  }
  m.refresh_sigma();

  EventBuilder scan_event = m.event("LioScan", now);
  scan_event.reason(o.reason)
      .value("sensor_time_s", seconds(t))
      .value("input_points", static_cast<double>(o.input_points));
  scan_event.value("points", static_cast<double>(o.points.size()));
  if (o.report) {
    m.last_report = *o.report;
    scan_event.value("translation_min_eigenvalue", o.report->translation_min_eigenvalue)
        .value("rotation_min_eigenvalue", o.report->rotation_min_eigenvalue)
        .value("quality", o.report->quality)
        .value("degenerate", o.report->degenerate ? 1.0 : 0.0);
  }
  m.emit(scan_event);

  const LioEvent::Kind kind = kind_of(o.reason);
  if (kind != LioEvent::Kind::kMapReady) m.last_scan_event_t = t;
  const Transition tr = m.lifecycle.on(LioEvent{kind, t, m.position_sigma_m});
  m.on_transition(tr, t, now, &out, o.report);

  const LioState s = m.lifecycle.state();
  const bool trusted = s == LioState::kInitializing || s == LioState::kTracking || s == LioState::kDegraded;
  if (o.reason == EstimatorEventReason::kMapBootstrap) {
    m.insert_into_map(o.points, now);
  } else if (o.reason == EstimatorEventReason::kScanGood && trusted) {
    m.insert_into_map(o.points, now);
  }
  // Only a LiDAR-corrected, non-degenerate state corrects the output; otherwise it dead-reckons on IMU.
  if (o.reason == EstimatorEventReason::kScanGood) m.correct_predictor(snapshot(m.eskf.stateView(), t), now);
  if (s == LioState::kTracking && o.report) out.odometry = m.odometry(t, o.report->quality, now);
  return out;
}

Result<void, EstimatorReason> LioEstimator::restart(const SeedPose& seed, const time::TimeSnapshot& now) {
  Impl& m = *impl_;
  const time::SensorTime t = m.last_imu ? m.last_imu->t : time::SensorTime{};
  if (m.lifecycle.state() != LioState::kRestarting || !m.eskf_t || !m.last_imu) {
    (void)m.reject(EstimatorReason::kWrongState, Stream::kRestart, t, now);
    return std::unexpected(EstimatorReason::kWrongState);
  }
  if (!seed.p_world_m.allFinite() || !usable(seed.q_world_base)) {
    (void)m.reject(EstimatorReason::kNotFinite, Stream::kRestart, t, now);
    return std::unexpected(EstimatorReason::kNotFinite);
  }
  const Eigen::Quaterniond q_seed = seed.q_world_base.normalized();
  try {
    // Biases, gravity and extrinsics carry over from the old epoch. The velocity is the predictor's
    // dead-reckoned one at the newest IMU time, rotated from the old world into the seed's.
    fl::ManifoldState carried = m.eskf.stateView();
    const Eigen::Quaterniond q_old = m.last_output ? m.last_output->q_world_imu : carried.orientation_odom_imu();
    const Eigen::Vector3d v_old = m.last_output ? m.last_output->v_world_mps : carried.velocity_odom_imu_m_s();
    const Eigen::Quaterniond q_new_imu = q_seed * m.q_base_imu;
    carried.set_velocity_odom_imu_m_s((q_new_imu * q_old.conjugate()) * v_old);

    fl::InitialStatePrior prior;
    prior.sample_time = stamp(t);
    prior.source = fl::InitialStatePriorSource::kTopic;
    prior.context = fl::InitialStatePriorContext::kInFlightReinitialization;
    prior.mask = fl::InitialStatePriorMask{true, false, fl::PriorAttitudeMode::kFull};
    prior.position_odom_base_m = seed.p_world_m;
    prior.orientation_odom_base = q_seed;
    prior.generation = m.epoch + 1;
    prior.provenance = "restart";
    fl::ManifoldState seeded;
    if (!m.prior_applicator.apply(prior, carried, kUnusedTiltLimitRad, seeded).ok()) {
      (void)m.reject(EstimatorReason::kNotFinite, Stream::kRestart, t, now);
      return std::unexpected(EstimatorReason::kNotFinite);
    }
    m.map.clear();
    m.local_map.reset();
    m.eskf.reset(seeded);
    m.eskf_t = t;
    m.trim_history(t);
    m.refresh_sigma();
    // Stamped at the newest IMU time, so the predictor buffer matches it (no rewind, output_predictor.hpp).
    (void)m.predictor.reset_to(snapshot(seeded, t));
  } catch (...) {
    (void)m.reject(EstimatorReason::kNotFinite, Stream::kRestart, t, now);
    return std::unexpected(EstimatorReason::kNotFinite);
  }

  const std::uint32_t old_epoch = m.epoch;
  ++m.epoch;
  m.pending_correction.reset();
  m.last_output.reset();
  m.last_report = DegeneracyReport{0.0, 0.0, true, 0};
  m.rate_reported = false;
  m.imu_period_samples = 0;
  m.next_health_t.reset();  // restart() returns no outputs: the next IMU sample carries the health
  const Transition tr = m.lifecycle.on(LioEvent{LioEvent::Kind::kRestartSeeded, t, m.position_sigma_m});
  m.emit(m.event("LioRestart", now)
             .reason(LioReason::kRestartSeeded)
             .value("old_epoch", old_epoch)
             .value("new_epoch", m.epoch)
             .value("seed_x_m", seed.p_world_m.x())
             .value("seed_y_m", seed.p_world_m.y())
             .value("seed_z_m", seed.p_world_m.z())
             .value("seed_yaw_rad", yaw_of(q_seed)));
  m.on_transition(tr, t, now, nullptr, std::nullopt);
  return {};
}

}  // namespace uavnav::lio
