/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */


#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include <utils/header/type_utils.hpp>
#include <utils/geometry/geometry_utils.h>
#include <data_structure/base/ellipsoid.h>

namespace geometry_utils {

    using navigation_math::Mat3f;
    using navigation_math::Vec3f;
    using navigation_math::vec_Vec3f;
    using navigation_math::Mat3Df;
    using navigation_math::MatD4f;
    using navigation_math::vec_E;



    class Polytope {
        bool undefined{true};
        bool is_known_free{false};

        MatD4f planes;
        bool have_seed_line{false};
    public:
        template <class Archive>
        void serialize(Archive& archive) {
            archive(undefined, is_known_free, planes, have_seed_line,
                    route_boundary_gate_,
                    route_boundary_point_, route_boundary_radius_m_,
                    overlap_depth_with_last_one, interior_pt_with_last_one,
                    ellipsoid_, seed_line.first,seed_line.second, robot_r);
        }


        double overlap_depth_with_last_one{0};
        Vec3f interior_pt_with_last_one{};
        Ellipsoid ellipsoid_{};
        std::pair<Vec3f, Vec3f> seed_line{};
        double robot_r{};

        Polytope() = default;

        explicit Polytope(MatD4f _planes);

        bool empty() const;

        bool HaveSeedLine() const;

        void SetSeedLine(const std::pair<Vec3f, Vec3f> &_seed_line, double r = 0);

        int SurfNum() const;

        Polytope CrossWith(const Polytope &b) const ;

        Vec3f CrossCenter(const Polytope &b) const ;

        bool HaveOverlapWith(Polytope cmp, double eps = 1e-6);

        MatD4f GetPlanes() const;

        void Reset();

        bool IsKnownFree();

        void SetKnownFree(bool is_free);

        // A route-boundary gate is a mission-geometry contract. It prevents
        // SFC simplification from removing the corridor cell whose adjacent
        // trajectory junction must enter the active waypoint acceptance
        // region before continuing onto the next leg.
        void SetRouteBoundaryGate(bool enabled) noexcept {
            route_boundary_gate_ = enabled;
            if (!enabled) {
                route_boundary_point_.setConstant(
                    std::numeric_limits<float>::quiet_NaN());
                route_boundary_radius_m_ =
                    std::numeric_limits<double>::quiet_NaN();
            }
        }

        bool IsRouteBoundaryGate() const noexcept {
            return route_boundary_gate_;
        }

        void SetRouteBoundaryContract(const Vec3f& point,
                                      const double radius_m) noexcept {
            if (!point.allFinite() || !std::isfinite(radius_m) || radius_m <= 0.0) {
                SetRouteBoundaryGate(false);
                return;
            }
            route_boundary_point_ = point;
            route_boundary_radius_m_ = radius_m;
            route_boundary_gate_ = true;
        }

        const Vec3f& GetRouteBoundaryPoint() const noexcept {
            return route_boundary_point_;
        }

        double GetRouteBoundaryRadius() const noexcept {
            return route_boundary_radius_m_;
        }

        void SetPlanes(MatD4f _planes);

        void SetEllipsoid(const Ellipsoid &ellip);

        bool PointIsInside(const Vec3f &pt, const double & margin = 0.01) const;

        double GetVolume() const ;

        double volume() const {
            return GetVolume();
        }

    private:
        bool route_boundary_gate_{false};
        Vec3f route_boundary_point_{
            Vec3f::Constant(std::numeric_limits<float>::quiet_NaN())};
        double route_boundary_radius_m_{
            std::numeric_limits<double>::quiet_NaN()};

    };

    typedef std::vector<Polytope> PolytopeVec;

    inline bool SimplifySFC(const Vec3f& head_p, const Vec3f& tail_p,
                                 geometry_utils::PolytopeVec& sfcs) {
        // A generic simplification may remove an intermediate corridor when
        // the first and a later corridor overlap. That would allow MINCO to
        // cut a genuine pass-through corner while remaining collision-free.
        if (std::any_of(sfcs.begin(), sfcs.end(),
                        [](const Polytope& polytope) {
                            return polytope.IsRouteBoundaryGate();
                        })) {
            return true;
        }
        vec_Vec3f path{head_p, tail_p};
        int start_id{-1}, end_id{-1};
        if (sfcs.size() > 2) {
            for (std::size_t i = 0; i < sfcs.size(); ++i) {
                if (sfcs[i].PointIsInside(path.front())) {
                    start_id = i;
                }
                const std::size_t reverse_index = sfcs.size() - 1 - i;
                if (sfcs[reverse_index].PointIsInside(path.back())) {
                    end_id = static_cast<int>(reverse_index);
                }
            }
            if (start_id < 0 || end_id < 0) {
                double best_head_violation = std::numeric_limits<double>::infinity();
                double best_tail_violation = std::numeric_limits<double>::infinity();
                for (const auto &sfc : sfcs) {
                    const MatD4f planes = sfc.GetPlanes();
                    best_head_violation = std::min(
                            best_head_violation,
                            (planes.leftCols<3>() * head_p + planes.col(3)).maxCoeff());
                    best_tail_violation = std::min(
                            best_tail_violation,
                            (planes.leftCols<3>() * tail_p + planes.col(3)).maxCoeff());
                }
                std::cout << color_text::RED
                          << " -- [EXPTrajOpt] Ill corridor: count=" << sfcs.size()
                          << " head=" << head_p.transpose()
                          << " head_best_violation=" << best_head_violation
                          << " tail=" << tail_p.transpose()
                          << " tail_best_violation=" << best_tail_violation
                          << " start_id=" << start_id << " end_id=" << end_id
                          << color_text::RESET << std::endl;
                return false;
            }
            if (start_id >= end_id) {
                end_id = start_id;
            }
            PolytopeVec sfcs_new(sfcs.begin() + start_id, sfcs.begin() + end_id + 1);
            if (sfcs_new.size() > 2) {
                Polytope check_cand = sfcs_new[0], last_overlapped = sfcs_new[1];
                PolytopeVec sfcs_final;
                sfcs_final.push_back(sfcs_new[0]);
                for (std::size_t i = 2; i < sfcs_new.size(); ++i) {
                    Polytope cross_poly = check_cand.CrossWith(sfcs_new[i]);
                    Vec3f interior_pt;
                    bool is_overlapped = geometry_utils::findInterior(cross_poly.GetPlanes(), interior_pt);
                    if (is_overlapped) {
                        last_overlapped = sfcs_new[i];
                        if (last_overlapped.PointIsInside(path.back())) {
                            sfcs_final.push_back(last_overlapped);
                            break;
                        }
                    }
                    else {
                        sfcs_final.push_back(last_overlapped);
                        check_cand = last_overlapped;
                            --i;
                    }
                }
                sfcs = sfcs_final;
            }
            else {
                sfcs = sfcs_new;
            }
        }
        return true;
    }


}
