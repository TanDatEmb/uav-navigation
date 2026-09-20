#pragma once

#include <string_view>

namespace navigation_planning_backend {

inline std::string_view solveStageName(const int stage) noexcept {
    switch (stage) {
        case 0: return "idle";
        case 1: return "setup";
        case 2: return "astar";
        case 3: return "corridor";
        case 4: return "main_minco";
        case 5: return "backup";
        case 31: return "corridor_setup";
        case 32: return "corridor_seed";
        case 33: return "corridor_iris";
        case 34: return "corridor_validate";
        case 35: return "corridor_finalize";
        case 36: return "corridor_complete";
        // Nominal corridor setup details are diagnostic sub-stages of the
        // historical stage 4 entry point.  They distinguish a pre-MINCO
        // rejection from an optimizer failure without changing planner
        // admission, timing, or safety behavior.
        case 41: return "corridor_setup_input";
        case 42: return "corridor_setup_guide";
        case 43: return "corridor_overlap";
        case 44: return "corridor_vertex_parameterization";
        case 45: return "corridor_vertex_finite";
        case 46: return "corridor_duration";
        case 47: return "corridor_path";
        default: return "unknown";
    }
}

inline int nominalSetupSolveStage(const int setup_failure_stage) noexcept {
    switch (setup_failure_stage) {
        case 1: return 41;
        case 2: return 42;
        case 3: return 43;
        case 4: return 44;
        case 5: return 45;
        case 6: return 46;
        case 7: return 47;
        default: return 4;
    }
}

}  // namespace navigation_planning_backend
