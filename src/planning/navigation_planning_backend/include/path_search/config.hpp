/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */


#pragma once

#include "utils/header/yaml_loader.hpp"
#include <cmath>
#include <string>
#include <stdexcept>

namespace path_search {
    class PathSearchConfig {
    public:
        bool visual_process{false};
        bool debug_visualization_en{false};
        bool allow_diag{false};
        int heu_type{0};
        double heuristic_weight{1.00001};

        PathSearchConfig() = default;

        PathSearchConfig(const std::string& cfg_path,
                         const std::string name_space = "astar")
            : PathSearchConfig(yaml_loader::YamlLoader(cfg_path), name_space) {}

        PathSearchConfig(const yaml_loader::YamlLoader& loader,
                         const std::string name_space = "astar") {
            loader.LoadParam(name_space + "/allow_diag", allow_diag, false);
            loader.LoadParam(name_space + "/debug_visualization_en", debug_visualization_en, false);
            loader.LoadParam(name_space + "/heu_type", heu_type, 0);
            loader.LoadParam(name_space + "/heuristic_weight", heuristic_weight, 1.00001);
            loader.LoadParam(name_space + "/visual_process", visual_process, false);
            if (!std::isfinite(heuristic_weight) || heuristic_weight < 1.0 ||
                heuristic_weight > 5.0) {
                throw std::invalid_argument("astar/heuristic_weight must be within [1, 5]");
            }
        }

    };
}
