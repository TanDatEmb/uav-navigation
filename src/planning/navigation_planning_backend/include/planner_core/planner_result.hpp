/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
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

    };

    static std::string PlannerResultCode_STR(const int& ret) {
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
        }
        return "Unknown planner return code (" + std::to_string(ret) + ")";
    };
}
#endif
