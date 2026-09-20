#include "fast_lio_core/preprocessing/point_cloud_preprocessor.hpp"

namespace uav::nav::lio {

PointCloudPreprocessor::PointCloudPreprocessor(PointCloudPreprocessorConfig config)
    : config_(config), point_filter_(config.point_filter), voxel_filter_(config.voxel_filter) {}

Result<PreprocessingResult> PointCloudPreprocessor::process(const LidarScan& scan) const {
  const Status scan_status = scan.validate();
  if (!scan_status.ok()) {
    return scan_status;
  }
  PreprocessingResult output;
  output.scan = scan;
  output.stats.input_point_count = scan.points.size();
  output.scan.points = point_filter_.filter(scan.points);
  output.stats.range_filtered_point_count =
      output.stats.input_point_count - output.scan.points.size();
  if (config_.enable_voxel_filter) {
    output.scan.points = voxel_filter_.filter(output.scan.points);
  }
  output.stats.output_point_count = output.scan.points.size();
  if (output.scan.points.empty()) {
    return Status(StatusCode::kInsufficientData, "Point preprocessing removed every LiDAR point");
  }
  return output;
}

}  // namespace uav::nav::lio
