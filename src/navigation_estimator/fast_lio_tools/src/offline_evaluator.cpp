#include "fast_lio_tools/offline_evaluator.hpp"

#include <stdexcept>
#include <utility>

#include "fast_lio_core/configuration/estimator_config.hpp"
#include "fast_lio_core/pipeline/fast_lio_pipeline.hpp"
#include "fast_lio_tools/dataset_reader.hpp"
#include "fast_lio_tools/report_writer.hpp"

namespace uav::nav::lio {

OfflineEvaluator::OfflineEvaluator(EvaluationConfig config) : config_(std::move(config)) {}

EvaluationMetrics OfflineEvaluator::run() {
  EstimatorConfig estimator_config;
  // Auto is confined to the offline boundary as required by the M1 contract.
  estimator_config.deskew.mode = DeskewMode::kAuto;
  FastLioPipeline pipeline{estimator_config};
  DatasetReader reader{config_.dataset_directory};
  EvaluationMetrics metrics;
  const auto drain = [&]() {
    while (const auto result = pipeline.processNext()) {
      if (result->has_corrected_odometry) {
        ++metrics.successful_correction_count;
      } else if (!result->rejection_reason.empty()) {
        ++metrics.rejected_scan_count;
      }
      metrics.residual_rms = result->diagnostics.registration.residual_rms_m;
      metrics.maximum_imu_gap_ms =
          static_cast<double>(result->diagnostics.synchronization.imu_gap_max_ns) * 1e-6;
    }
  };
  reader.replay(
      [&](const ImuSample& sample) {
        ++metrics.imu_sample_count;
        const auto status = pipeline.pushImu(sample);
        if (!status.ok()) {
          throw std::runtime_error(status.message());
        }
        drain();
      },
      [&](const LidarScan& scan) {
        ++metrics.lidar_scan_count;
        const auto status = pipeline.pushLidar(scan);
        if (!status.ok()) {
          throw std::runtime_error(status.message());
        }
        drain();
      });
  drain();
  ReportWriter::write(config_.output_directory, metrics);
  return metrics;
}

}  // namespace uav::nav::lio
