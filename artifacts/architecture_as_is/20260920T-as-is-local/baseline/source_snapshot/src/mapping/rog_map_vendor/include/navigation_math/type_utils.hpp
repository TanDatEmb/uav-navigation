//
// Created by yunfan on 11/8/24.
//

#ifndef NAVIGATION_MATH_TYPE_UTILS_HPP
#define NAVIGATION_MATH_TYPE_UTILS_HPP

// comment or uncomment the following line to enable or disable the utils
#include <navigation_math/color_text.hpp>
#include <navigation_math/eigen_alias.hpp>

namespace navigation_math{

    using std::vector;
    using std::string;
    using std::cout;
    using std::endl;


    enum RET_CODE {
        /// FOR Planner
        FAILED = 0,
        NO_NEED = 1,
        SUCCESS = 2,
        FINISH = 3,
        NEW_TRAJ = 4,
        EMER = 5,
        OPT_FAILED = 6,
        INIT_ERROR = 7,

        /// FOR path search
        REACH_HORIZON,
        REACH_GOAL,
        NO_PATH,
        TIME_OUT
    };

    static const std::vector<std::string> RET_CODE_STR{"FAILED", "NO_NEED", "SUCCESS",
                                                       "FINISH", "NEW_TRAJ", "EMER",
                                                       "OPT_FAILED","INIT_ERROR",
                                                        "REACH_HORIZON", "REACH_GOAL", "NO_PATH", "TIME_OUT"};

    enum GridType {
        UNDEFINED = 0,
        UNKNOWN = 1,
        OUT_OF_MAP,
        OCCUPIED,
        KNOWN_FREE,
        FRONTIER, // The frontier is an unknown grid which is adjacent to the known free grid
    };

    const static std::vector<std::string> GridTypeStr{"UNDEFINED",
                                                      "UNKNOWN",
                                                      "OUT_OF_MAP",
                                                      "OCCUPIED",
                                                      "KNOWN_FREE",
                                                      "FRONTIER"};

    template<typename T>
    std::ostream &operator<<(std::ostream &out, const std::vector<T> &v) {
        out << "[";
        for (typename std::vector<T>::const_iterator it = v.begin(); it != v.end(); ++it) {
            out << *it;
            if (it != v.end() - 1) {
                out << ", ";
            }
        }
        out << "]";
        return out;
    }

    struct RobotState {
        Vec3f p{Vec3f::Zero()};
        Vec3f v{Vec3f::Zero()};
        Vec3f a{Vec3f::Zero()};
        Vec3f j{Vec3f::Zero()};
        double yaw{0.0};
        double rcv_time{0.0};
        bool rcv{false};
        Quatf q{Quatf::Identity()};
    };




}


#endif //NAVIGATION_MATH_TYPE_UTILS_HPP
