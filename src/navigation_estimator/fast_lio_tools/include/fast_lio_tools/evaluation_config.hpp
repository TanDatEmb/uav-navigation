#pragma once

#include <filesystem>

namespace uav::nav::lio {

struct EvaluationConfig {
  std::filesystem::path dataset_directory;
  std::filesystem::path output_directory{"evaluation"};
};

}  // namespace uav::nav::lio
