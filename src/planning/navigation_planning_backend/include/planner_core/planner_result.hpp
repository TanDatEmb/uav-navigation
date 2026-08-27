/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#ifndef PlannerResultCode_HPP
#define PlannerResultCode_HPP

#include <cstring>
#include <string>
#include <vector>

namespace navigation_planning_backend {
    enum PlannerResultCode {
        PLANNER_SUCCESS_WITH_BACKUP = 3,
        PLANNER_SUCCESS_NO_BACKUP = 2,
        PLANNER_SUCCESS = 1,
        PLANNER_UNDEFINED = 0,
        PLANNER_NO_ODOM = -1,
        PLANNER_NO_START_POINT = -2,
        PLANNER_BACKUP_FAILED = -3,
        PLANNER_SOLVE_TIMEOUT = -4,
        PLANNER_SOLVE_CANCELLED = -5,
        PLANNER_EXP_FAILED = -6,
        PLANNER_CANDIDATE_REJECTED = -7,
        PLANNER_BACKUP_OPTIMIZATION_FAILED = -8,
        PLANNER_BACKUP_INITIALIZATION_FAILED = -9,
        PLANNER_BACKUP_NO_PATH = -10,

    };

    inline std::string PlannerResultCode_STR(const int& ret) {
        switch (ret) {
        case PLANNER_SUCCESS_WITH_BACKUP:
            return "Success, with backup trajectory also success";
        case PLANNER_SUCCESS_NO_BACKUP:
            return "Success, without need of backup";
        case PLANNER_SUCCESS:
            return "Success";
        case PLANNER_UNDEFINED:
            return "Undefined";
        case PLANNER_NO_ODOM:
            return "No odom, return at the start of the replan";
        case PLANNER_NO_START_POINT:
            return "Cannot find a start point in the local map";
        case PLANNER_BACKUP_FAILED:
            return "Backup trajectory required but generation failed";
        case PLANNER_SOLVE_TIMEOUT:
            return "Solve deadline exhausted before a candidate could be committed";
        case PLANNER_SOLVE_CANCELLED:
            return "Solve was cancelled before a candidate could be committed";
        case PLANNER_EXP_FAILED:
            return "Main trajectory generation failed before a candidate could be committed";
        case PLANNER_CANDIDATE_REJECTED:
            return "Generated candidate failed construction or world validation";
        case PLANNER_BACKUP_OPTIMIZATION_FAILED:
            return "Backup optimizer failed without exhausting the solve deadline";
        case PLANNER_BACKUP_INITIALIZATION_FAILED:
            return "Backup trajectory initialization failed";
        case PLANNER_BACKUP_NO_PATH:
            return "Backup path search found no admissible path";
        }
        return "Unknown planner return code (" + std::to_string(ret) + ")";
    };
}
#endif
