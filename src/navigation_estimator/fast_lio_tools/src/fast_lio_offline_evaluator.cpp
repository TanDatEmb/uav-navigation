#include <exception>
#include <filesystem>
#include <iostream>

#include "fast_lio_tools/offline_evaluator.hpp"

int main(int argc, char** argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: fast_lio_offline_evaluator DATASET_DIR [OUTPUT_DIR]\n";
    return 2;
  }
  try {
    uav::nav::lio::EvaluationConfig config;
    config.dataset_directory = argv[1];
    if (argc == 3) {
      config.output_directory = argv[2];
    }
    const auto metrics = uav::nav::lio::OfflineEvaluator{config}.run();
    std::cout << "processed " << metrics.lidar_scan_count << " scans; "
              << metrics.successful_correction_count << " corrected updates\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "offline evaluation failed: " << error.what() << '\n';
    return 1;
  }
}
