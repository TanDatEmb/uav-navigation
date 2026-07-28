#include "fast_lio_tools/report_writer.hpp"

#include <array>
#include <fstream>
#include <stdexcept>

namespace uav::nav::lio {

void ReportWriter::write(const std::filesystem::path& directory, const EvaluationMetrics& m) {
  std::filesystem::create_directories(directory);
  std::ofstream summary(directory / "summary.json");
  summary << "{\n  \"imu_sample_count\": " << m.imu_sample_count
          << ",\n  \"lidar_scan_count\": " << m.lidar_scan_count
          << ",\n  \"successful_correction_count\": " << m.successful_correction_count
          << ",\n  \"rejected_scan_count\": " << m.rejected_scan_count << "\n}\n";
  std::ofstream report(directory / "report.md");
  report << "# M1 offline evaluation\n\n"
         << "- IMU samples: " << m.imu_sample_count << "\n"
         << "- LiDAR scans: " << m.lidar_scan_count << "\n"
         << "- Successful corrections: " << m.successful_correction_count
         << "\n- Rejected scans: " << m.rejected_scan_count << "\n";
  constexpr std::array<const char*, 6> kCsvFiles{"timing.csv",    "synchronization.csv",
                                                 "state.csv",     "trajectory.csv",
                                                 "residuals.csv", "deskew.csv"};
  for (const char* name : kCsvFiles) {
    std::ofstream(directory / name) << "status\nno_samples\n";
  }
  std::ofstream(directory / "map.pcd") << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
                                          "TYPE F F F\nCOUNT 1 1 1\nWIDTH 0\nHEIGHT 1\nPOINTS 0\n"
                                          "DATA ascii\n";
  if (!summary || !report) {
    throw std::runtime_error("failed to write evaluation report");
  }
}

}  // namespace uav::nav::lio
