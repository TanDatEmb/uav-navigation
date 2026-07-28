#pragma once

#include <filesystem>

#include "fast_lio_tools/evaluation_metrics.hpp"

namespace uav::nav::lio {

class ReportWriter {
 public:
  static void write(const std::filesystem::path& output_directory,
                    const EvaluationMetrics& metrics);
};

}  // namespace uav::nav::lio
