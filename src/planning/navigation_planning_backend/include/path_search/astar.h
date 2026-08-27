/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */


#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "Eigen/Dense"
#include "vector"
#include <navigation_world_model/world_model_view.hpp>
#include "queue"
#include "path_search/config.hpp"
#include "utils/header/type_utils.hpp"
#include <planner_runtime_context/planner_runtime_context.hpp>


namespace path_search {
    using namespace navigation_math;

    constexpr double inf = std::numeric_limits<double>::infinity();
    struct GridNode;
    typedef GridNode *GridNodePtr;

    struct GridNode {
        enum enum_state {
            OPENSET = 1,
            CLOSEDSET = 2,
            UNDEFINED = 3
        } state{UNDEFINED};

        int rounds{0};
        Vec3i id_g;
        double total_score{inf}, distance_score{inf};
        double distance_to_goal{inf};
        GridNodePtr father_ptr{nullptr};
        std::uint64_t frontier_sequence{0U};
    };

    struct OpenSetEntry {
        GridNodePtr node{nullptr};
        double total_score{inf};
        int rounds{0};
    };

    class OpenSetComparator {
    public:
        bool operator()(const OpenSetEntry &entry1, const OpenSetEntry &entry2) const {
            return entry1.total_score > entry2.total_score;
        }
    };

    struct FrontierEntry {
        GridNodePtr node{nullptr};
        double distance_to_goal{inf};
        std::uint64_t sequence{0U};
    };

    class FrontierComparator {
    public:
        bool operator()(const FrontierEntry &entry1, const FrontierEntry &entry2) const {
            if (entry1.distance_to_goal != entry2.distance_to_goal) {
                return entry1.distance_to_goal > entry2.distance_to_goal;
            }
            return entry1.sequence > entry2.sequence;
        }
    };

    const int ON_INF_MAP = (1 << 0);
    const int ON_PROB_MAP = (1 << 1);
    const int UNKNOWN_AS_OCCUPIED = (1 << 3);
    const int UNKNOWN_AS_FREE = (1 << 4);
    const int USE_INF_NEIGHBOR = (1 << 5);
    const int DONT_USE_INF_NEIGHBOR = (1 << 6);

    class Astar {

        navigation_world_model::WorldModelViewPtr map_ptr_;
        navigation_planner_context::PlannerRuntimeContext::Ptr planner_context_;

        PathSearchConfig cfg_;
        double search_time_limit_s_{0.0};
        vec_Vec3i neighbor_list;

        // Search state is sparse: a solve usually visits a small fraction of
        // the map. Allocate only nodes touched by the current search.
        std::unordered_map<std::size_t, std::unique_ptr<GridNode>> visited_nodes_;

        int rounds_{0};
        std::uint64_t frontier_sequence_{0U};

        static constexpr int DIAG = 0;
        static constexpr int MANH = 1;
        static constexpr int EUCL = 2;

        struct MissionData {
            Vec3f start_pt;
            Vec3f goal_pt;
            double searching_horizon;
            bool use_inf_map{false};
            bool use_prob_map{false};
            bool unknown_as_occ{false};
            bool unknown_as_free{false};
            bool use_inf_neighbor{false};
            double resolution;
            Vec3i local_map_center_id_g;
            Vec3f local_map_center_d;
            double mission_rcv_WT{0};
            Vec3f local_map_max_d, local_map_min_d;
            Vec3i local_global_min_index{Vec3i::Zero()};
            Vec3i local_global_max_index{Vec3i::Zero()};
            Vec3i local_lower_extent_i{Vec3i::Zero()};
            Vec3i local_upper_extent_i{Vec3i::Zero()};
            Vec3i local_voxel_count{Vec3i::Zero()};
            std::mutex mission_mtx;
        } md_;



        double getHeu(GridNodePtr node1, GridNodePtr node2, int type = DIAG) const;

        std::size_t getLocalIndexHash(const Vec3i &id_in) const;

        void posToGlobalIndex(const Vec3f &pos, Vec3i &id_g) const ;

        void globalIndexToPos(const Vec3i &id_g, Vec3f &pos) const;

        bool insideLocalMap(const Vec3f &pos) const;

        bool insideLocalMap(const Vec3i &id_g) const;

        bool neighborHaveOne(const navigation_world_model::CellState &type, const Vec3i &src_id);

        RET_CODE setup(const Vec3f &start_pt, const Vec3f &goal_pt, const int &flag,
                       const double &searching_horizon = 9999);

        void retrievePath(GridNodePtr current, vector<GridNodePtr> &path);

        void ConvertNodePathToPointPath(const vector<GridNodePtr> &node_path, vec_Vec3f &point_path);

        GridNodePtr nodeAt(const std::size_t local_index) {
            auto& slot = visited_nodes_[local_index];
            if (!slot) {
                slot = std::make_unique<GridNode>();
            }
            return slot.get();
        }

    public:

        Astar(const PathSearchConfig& config,
              const navigation_planner_context::PlannerRuntimeContext::Ptr &planner_context,
              navigation_world_model::WorldModelViewPtr rm,
              double search_time_limit_s);

        ~Astar();
        Astar(const Astar&) = delete;
        Astar& operator=(const Astar&) = delete;
        Astar(Astar&&) = delete;
        Astar& operator=(Astar&&) = delete;

        typedef std::shared_ptr<Astar> Ptr;

        void setVisualProcessEn(const bool &en);

        void setFineInfNeighbors(const int & neighbor_step);

        void setWorldModelView(navigation_world_model::WorldModelViewPtr view) {
            map_ptr_ = std::move(view);
        }

        RET_CODE pointToPointPathSearch(const Vec3f &start_pt, const Vec3f &end_pt,
                                        const int &flag,
                                        const double &searching_horizon,
                                        vec_Vec3f &out_path,
                                        const double &time_out = -1.0,
                                        const bool prefer_start_goal_altitude = false);

        /// @ brief: The escape path only for path search from prob map to inf map. from non-occupied point to
        ///          inf map free (or known freee) point . Aim to find a path from current point to (known) free point
        /// @ param:
        RET_CODE escapePathSearch(const Vec3f &start_pt, const int flag,
                                  vec_Vec3f &out_path,
                                  const bool prefer_start_altitude = false,
                                  const double time_out = -1.0);


    };
}
