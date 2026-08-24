#pragma once

#include <string_view>

namespace super_planner {

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
        default: return "unknown";
    }
}

}  // namespace super_planner
