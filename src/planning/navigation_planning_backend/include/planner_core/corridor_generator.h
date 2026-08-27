/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#pragma once

#include <atomic>
#include <algorithm>
#include <cstddef>
#include "memory"
#include <stdexcept>

#include <planner_core/config.hpp>
#include <planner_core/ciri.h>
#include <planner_core/absolute_deadline.hpp>

#include <data_structure/base/polytope.h>
#include <data_structure/base/trajectory.h>

#include <navigation_world_model/world_model_view.hpp>
#include <utils/header/fmt_eigen.hpp>

#include <planner_runtime_context/planner_runtime_context.hpp>



namespace navigation_planning_backend {
    using namespace geometry_utils;
    using navigation_math::Vec3i;
    using navigation_math::vec_E;
    using navigation_math::Line;
    using namespace color_text;
    using navigation_math::OCCUPIED;

    class CorridorGenerator {
    private:
        navigation_planner_context::PlannerRuntimeContext::Ptr planner_context_;
        double bound_dis_;
        double seed_line_max_length_;
        double min_overlap_threshold_;
        double robot_r_;
        int iris_iter_num_;
        double virtual_groud_height_ = 0.0;
        double virtual_ceil_height_ = 0.0;
        navigation_world_model::WorldModelViewPtr map_ptr_;
        navigation_world_model::UnknownPolicy unknown_policy_{
            navigation_world_model::UnknownPolicy::kAllowUnknown};
        vec_E<Vec3i> line_seed_neighbor_list;
        CIRI::Ptr ciri_;
        std::ofstream failed_traj_log;

        vec_Vec3f latest_pc;
        static constexpr std::size_t kMaximumDiagnosticPoints = 4096U;

        void appendDiagnosticPoints(const vec_Vec3f& points) {
            const std::size_t remaining =
                kMaximumDiagnosticPoints > latest_pc.size()
                    ? kMaximumDiagnosticPoints - latest_pc.size() : 0U;
            const std::size_t count = std::min(remaining, points.size());
            latest_pc.insert(latest_pc.end(), points.begin(), points.begin() + count);
        }

        double ciri_t{0};
        int ciri_cnt{0};
        // Detailed, lock-free progress for the outer planner watchdog.
        // 0 idle, 1 segment/ray checks, 2 line box query, 3 line CIRI,
        // 4 overlap LP, 5 point box query, 6 point CIRI.
        std::atomic<int> solve_stage_{0};
        std::atomic<std::size_t> solve_point_count_{0};
        void refreshVerticalBounds();
    public:
        vec_Vec3f getLatestCloud() {
            vec_Vec3f out = latest_pc;
            latest_pc.clear();
            return out;
        }

        CorridorGenerator(const navigation_planner_context::PlannerRuntimeContext::Ptr &planner_context,
                          navigation_world_model::WorldModelViewPtr map_ptr,
                          const double bound_dis,
                          const double seed_line_max_dis,
                          const double min_overlap_threshold,
                          const double virtual_groud_height,
                          const double virtual_ceil_height,
                          const double robot_r,
                          const int iris_iter_num,
                          navigation_world_model::UnknownPolicy unknown_policy);

        ~CorridorGenerator() = default;

        void SetLineNeighborList(const vec_E<Vec3i> &line_seed_neighbor_list);

        typedef std::shared_ptr<CorridorGenerator> Ptr;

        int solveStage() const noexcept { return solve_stage_.load(); }
        std::size_t solvePointCount() const noexcept { return solve_point_count_.load(); }
        void setWorldModelView(navigation_world_model::WorldModelViewPtr view) {
            if (!view) {
                throw std::invalid_argument("CorridorGenerator requires a world-model view");
            }
            map_ptr_ = std::move(view);
            refreshVerticalBounds();
        }

        bool SearchPolytopeOnPath(const vec_Vec3f &path, PolytopeVec &sfcs,
                                  Vec3f & shifted_start_pt,
                                  bool cut_first_poly = false,
                                  const AbsoluteDeadline* deadline = nullptr);

        void getSeedBBox(const Vec3f &p1, const Vec3f &p2,
                         Vec3f &box_min, Vec3f &box_max);

        bool GeneratePolytopeFromPoint(const Vec3f &pt, Polytope &polytope,
                                       const AbsoluteDeadline* deadline = nullptr);

        bool GenerateEmptyPolytope(const navigation_math::Vec3f &pt,
                                   const double & dis,
                                   Polytope & polytope);

        bool GeneratePolytopeFromLine(Line &line, Polytope &polytope,
                                      const AbsoluteDeadline* deadline = nullptr);

        double getCiriComputationTime() {
            if (ciri_cnt == 0) {
                return -1;
            }
            double aver_T = ciri_t / ciri_cnt;
            ciri_t = 0;
            ciri_cnt = 0;
            return aver_T;
        }


        void setIterNum(int iter){
            iris_iter_num_ = iter;
        }

    };
}
