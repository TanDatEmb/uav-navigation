/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#include <planner_core/planner.hpp>
#include <planner_core/route_regression_certificate.hpp>
#include <planner_core/route_backbone.hpp>
#include <planner_core/hot_replan_recovery.hpp>
#include <planner_core/guide_vertical_envelope.hpp>
#include <planner_core/altitude_route_projection.hpp>
#include <planner_core/absolute_deadline.hpp>
#include <planner_core/backup_braking.hpp>
#include <planner_core/evidence_speed_governor.hpp>
#include <planner_core/command_time.hpp>
#include <planner_core/guide_endpoint.hpp>
#include <planner_core/kinematic_state_boundary.hpp>
#include <planner_core/replan_contract.hpp>
#include <planner_core/trajectory_world_validator.hpp>
#include <navigation_world_model/continuous_clearance.hpp>
#include <navigation_world_model/goal_contract.hpp>
#include <navigation_common/time.hpp>
#include <navigation_planning/planning_timing.hpp>
#include <traj_opt/trajectory_dynamics.hpp>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <navigation_math/scope_timer.hpp>
#include <fmt/color.h>

using namespace navigation_math;
using std::isnan;

namespace navigation_planning_backend {

namespace {

std::string trajectoryDurationSummary(const Trajectory& trajectory) {
    std::ostringstream output;
    output << '[';
    for (int index = 0; index < trajectory.getPieceNum(); ++index) {
        if (index != 0) output << ',';
        output << trajectory[index].getDuration();
    }
    output << ']';
    return output.str();
}

double knownFreeGuideSupport(
        const navigation_world_model::WorldModelView& world,
        const vec_Vec3f& guide) {
    if (guide.size() < 2U) return 0.0;
    double support_m = 0.0;
    for (std::size_t index = 1; index < guide.size(); ++index) {
        const Eigen::Vector3d begin = guide[index - 1U].cast<double>();
        const Eigen::Vector3d end = guide[index].cast<double>();
        const double segment_m = (end - begin).norm();
        if (!std::isfinite(segment_m) || segment_m <= 1.0e-9 ||
            !world.isSegmentTraversable(
                begin, end, navigation_world_model::GridLayer::kInflated,
                navigation_world_model::UnknownPolicy::kRequireKnownFree)) {
            break;
        }
        support_m += segment_m;
    }
    return support_m;
}

std::optional<double> firstRouteBoundaryEntryTime(
        const Trajectory& trajectory,
        const Eigen::Vector3d& boundary_point,
        const double boundary_radius_m) {
    if (trajectory.empty() || trajectory.getPieceNum() < 1 ||
        !boundary_point.allFinite() || !std::isfinite(boundary_radius_m) ||
        boundary_radius_m <= 0.0) {
        return std::nullopt;
    }

    const double total_duration_s = trajectory.getTotalDuration();
    if (!std::isfinite(total_duration_s) || total_duration_s <= 0.0) {
        return std::nullopt;
    }

    // The route contract is an entry into the mission acceptance ball, not a
    // requirement that the optimizer place a polynomial junction exactly at
    // the ball centre.  Check polynomial junctions first.  A fixed sampling
    // grid alone is not a certificate: a fast trajectory can enter and leave
    // the ball between two 10 ms samples even though the optimizer's hard
    // route-boundary junction is valid.
    double junction_time_s = 0.0;
    for (int junction_index = 0;
         junction_index <= trajectory.getPieceNum(); ++junction_index) {
        const Eigen::Vector3d position = trajectory.getJuncPos(junction_index);
        if (!position.allFinite()) return std::nullopt;
        const double distance_m = (position - boundary_point).norm();
        if (std::isfinite(distance_m) &&
            distance_m <= boundary_radius_m + 1.0e-6) {
            return junction_time_s;
        }
        if (junction_index < trajectory.getPieceNum()) {
            const double duration_s = trajectory[junction_index].getDuration();
            if (!std::isfinite(duration_s) || duration_s <= 0.0 ||
                !std::isfinite(junction_time_s + duration_s)) {
                return std::nullopt;
            }
            junction_time_s += duration_s;
        }
    }

    // The junction check above covers the hard optimizer contract.  Keep a
    // conservative fixed-time witness for a valid interior crossing in a
    // trajectory representation that does not put the crossing at a junction.
    constexpr double kBoundaryWitnessDtS = 0.01;
    const double sample_count = std::ceil(total_duration_s / kBoundaryWitnessDtS);
    if (!std::isfinite(sample_count) || sample_count > 10000000.0) {
        return std::nullopt;
    }
    for (std::uint64_t sample = 0;
         sample <= static_cast<std::uint64_t>(sample_count); ++sample) {
        const double time_s = std::min(
            total_duration_s,
            static_cast<double>(sample) * kBoundaryWitnessDtS);
        const Eigen::Vector3d position = trajectory.getPos(time_s);
        if (!position.allFinite()) return std::nullopt;
        const double distance_m = (position - boundary_point).norm();
        if (std::isfinite(distance_m) &&
            distance_m <= boundary_radius_m + 1.0e-6) {
            return time_s;
        }
    }
    return std::nullopt;
}

}  // namespace

    PlannerResultCode Planner::classifySolveFailure(
            const AbsoluteDeadline &solve_deadline,
            const bool elapsed_budget_exceeded,
            const PlannerResultCode fallback) const {
        if (solve_cancelled_.load(std::memory_order_relaxed)) {
            return PLANNER_SOLVE_CANCELLED;
        }
        if (elapsed_budget_exceeded ||
            solve_deadline.expired(planner_context_->getSimTime()) ||
            solve_deadline.steadyExpired()) {
            return PLANNER_SOLVE_TIMEOUT;
        }
        return fallback;
    }

    void Planner::setRecoveryVelocityScale(const double scale) noexcept {
        const double bounded_scale =
            std::isfinite(scale) && scale > 0.0 && scale <= 1.0 ? scale : 1.0;
        const double exp_limit = nominal_exp_max_velocity_mps_ * bounded_scale;
        const double backup_limit = nominal_backup_max_velocity_mps_ * bounded_scale;
        if (!std::isfinite(exp_limit) || exp_limit <= 0.0 ||
            !std::isfinite(backup_limit) || backup_limit <= 0.0) {
            cfg_.exp_traj_cfg.max_vel = nominal_exp_max_velocity_mps_;
            cfg_.back_traj_cfg.max_vel = nominal_backup_max_velocity_mps_;
            exp_traj_opt_->setMaximumVelocity(nominal_exp_max_velocity_mps_);
            back_traj_opt_->setMaximumVelocity(nominal_backup_max_velocity_mps_);
            return;
        }
        cfg_.exp_traj_cfg.max_vel = exp_limit;
        cfg_.back_traj_cfg.max_vel = backup_limit;
        exp_traj_opt_->setMaximumVelocity(exp_limit);
        back_traj_opt_->setMaximumVelocity(backup_limit);
    }

    Vec3f Planner::resolveGoalForPlanning(const Vec3f& requested_goal) {
        requested_goal_p_ = requested_goal;
        planning_goal_p_ = requested_goal;
        goal_endpoint_adjusted_ = false;
        requested_goal_inflated_state_ = navigation_world_model::CellState::kOutOfMap;
        planning_goal_inflated_state_ = navigation_world_model::CellState::kOutOfMap;

        if (!map_ptr_ || !requested_goal.allFinite()) return requested_goal;

        requested_goal_inflated_state_ = map_ptr_->classify(
            requested_goal.cast<double>(), navigation_world_model::GridLayer::kInflated);
        planning_goal_inflated_state_ = requested_goal_inflated_state_;

        // Reserve the existing tracking certificate before projecting a
        // blocked terminal waypoint. The mission acceptance gate remains the
        // full radius, but a projected endpoint at that outer boundary has
        // no room for the unchanged 0.25 m execution-anchor budget.
        const double usable_projection_radius =
            goal_acceptance_radius_m_ - cfg_.tracking_error_budget_m;
        const bool usable_projection_radius_valid =
            std::isfinite(usable_projection_radius) && usable_projection_radius > 0.0;
        if (terminal_stop_required_ && usable_projection_radius_valid &&
            acceptance_endpoint_lease_.has_value() && route_snapshot_.has_value() &&
            acceptance_endpoint_lease_request_id_ == route_snapshot_->request_id &&
            acceptance_endpoint_lease_route_revision_ == route_snapshot_->route_revision &&
            acceptance_endpoint_lease_waypoint_index_ ==
                route_snapshot_->active_waypoint_index) {
            const auto& leased = *acceptance_endpoint_lease_;
            const double lease_error =
                (leased.cast<double>() - requested_goal.cast<double>()).norm();
            const bool lease_valid = leased.allFinite() &&
                std::isfinite(lease_error) && lease_error <=
                    usable_projection_radius + 1.0e-6 && map_ptr_->contains(leased) &&
                map_ptr_->classify(
                    leased.cast<double>(), navigation_world_model::GridLayer::kInflated) ==
                    navigation_world_model::CellState::kKnownFree &&
                map_ptr_->isSegmentTraversable(
                    leased.cast<double>(), leased.cast<double>(),
                    navigation_world_model::GridLayer::kInflated,
                    navigation_world_model::UnknownPolicy::kRequireKnownFree);
            if (lease_valid) {
                planning_goal_p_ = leased;
                planning_goal_inflated_state_ = map_ptr_->classify(
                    leased.cast<double>(), navigation_world_model::GridLayer::kInflated);
                goal_endpoint_adjusted_ = lease_error > 1.0e-6;
                return planning_goal_p_;
            }
            acceptance_endpoint_lease_.reset();
        }

        // A STOP waypoint is an acceptance region, not a requirement to stop
        // at the exact centre of a point that may have only one voxel of
        // clearance.  When the requested voxel is already free, choose the
        // highest-clearance known-free lattice point in the same acceptance
        // ball.  This protects the terminal stop against the measured
        // tracking residual that can otherwise turn a legal nominal endpoint
        // into an occupied emergency-brake boundary.  The endpoint remains
        // owned by the mission acceptance contract and the complete candidate
        // is still checked by authorizeAndStage().
        if (terminal_stop_required_ &&
            requested_goal_inflated_state_ ==
                navigation_world_model::CellState::kKnownFree &&
            std::isfinite(goal_acceptance_radius_m_) &&
            goal_acceptance_radius_m_ > 0.0) {
            // A known-free requested endpoint already carries the authoritative
            // inflated-layer safety certificate. Do not replace it solely
            // because raw occupied samples produce a larger heuristic
            // clearance score elsewhere in the acceptance ball: that moves a
            // terminal waypoint without improving the contract, can make the
            // following route regression harder, and may create a needless
            // MINCO/corridor failure at the handoff. Keep the conservative
            // acceptance-ball projection below only for the inconsistent case
            // where the point classification is free but the exact continuous
            // oracle cannot certify the point.
            if (map_ptr_->isSegmentTraversable(
                    requested_goal.cast<double>(), requested_goal.cast<double>(),
                    navigation_world_model::GridLayer::kInflated,
                    navigation_world_model::UnknownPolicy::kRequireKnownFree)) {
                return planning_goal_p_;
            }
            const auto geometry = map_ptr_->geometry();
            const double step = geometry.inflated_resolution_m;
            const double obstacle_radius = geometry.occupied_inflation_radius_m;
            const bool geometry_valid = std::isfinite(step) && step > 0.0 &&
                std::isfinite(obstacle_radius) && obstacle_radius >= 0.0;
            navigation_world_model::AxisAlignedBox query;
            const double query_radius = usable_projection_radius_valid
                ? usable_projection_radius + obstacle_radius + step
                : 0.0;
            if (geometry_valid && std::isfinite(query_radius) && query_radius > 0.0) {
                query.minimum = requested_goal.cast<double>() -
                    Eigen::Vector3d::Constant(query_radius);
                query.maximum = requested_goal.cast<double>() +
                    Eigen::Vector3d::Constant(query_radius);
            }
            const auto occupied = query.valid()
                ? map_ptr_->observedOccupiedPoints(query)
                : navigation_world_model::PointVector{};
            const auto clearance_score = [&occupied](const Eigen::Vector3d& point) {
                double score = std::numeric_limits<double>::infinity();
                for (const auto& occupied_point : occupied) {
                    if (!occupied_point.allFinite()) {
                        return -std::numeric_limits<double>::infinity();
                    }
                    score = std::min(score, (point - occupied_point).norm());
                }
                return score;
            };
            Eigen::Vector3d best = requested_goal.cast<double>();
            double best_score = clearance_score(best);
            const double clearance_trigger = obstacle_radius +
                (usable_projection_radius_valid ? usable_projection_radius : 0.0);
            // Do not move an otherwise well-separated STOP merely because a
            // far-away occupied sample happens to score slightly better at a
            // different point in the acceptance ball.  The endpoint search
            // is for a terminal clearance deficit, not general goal shaping.
            if (!std::isfinite(clearance_trigger) ||
                best_score >= clearance_trigger) {
                return planning_goal_p_;
            }
            const int extent = geometry_valid && usable_projection_radius_valid
                ? static_cast<int>(std::ceil(usable_projection_radius / step)) : -1;
            if (extent >= 0 && extent <= 64) {
                for (int dx = -extent; dx <= extent; ++dx) {
                    for (int dy = -extent; dy <= extent; ++dy) {
                        const Eigen::Vector3d candidate = requested_goal.cast<double>() +
                            Eigen::Vector3d{dx * step, dy * step, 0.0};
                        const double distance =
                            (candidate - requested_goal.cast<double>()).norm();
                        if (!candidate.allFinite() || !std::isfinite(distance) ||
                            distance > usable_projection_radius + 1.0e-6 ||
                            !map_ptr_->contains(candidate) ||
                            map_ptr_->classify(
                                candidate, navigation_world_model::GridLayer::kInflated) !=
                                navigation_world_model::CellState::kKnownFree ||
                            !map_ptr_->isSegmentTraversable(
                                candidate, candidate,
                                navigation_world_model::GridLayer::kInflated,
                                navigation_world_model::UnknownPolicy::kRequireKnownFree)) {
                            continue;
                        }
                        const double score = clearance_score(candidate);
                        const bool higher_clearance = score > best_score + 1.0e-9;
                        const bool equal_clearance_closer =
                            std::abs(score - best_score) <= 1.0e-9 &&
                            distance < (best - requested_goal.cast<double>()).norm();
                        if (higher_clearance || equal_clearance_closer) {
                            best = candidate;
                            best_score = score;
                        }
                    }
                }
            }
            planning_goal_p_ = best;
            planning_goal_inflated_state_ = map_ptr_->classify(
                best, navigation_world_model::GridLayer::kInflated);
            goal_endpoint_adjusted_ =
                (best - requested_goal.cast<double>()).norm() > 1.0e-6;
            if (goal_endpoint_adjusted_) {
                planner_context_->info(
                    " -- [planner] selected clearance-maximizing STOP endpoint "
                    "requested=({:.3f},{:.3f},{:.3f}) planning=({:.3f},{:.3f},{:.3f}) "
                    "offset={:.3f} clearance_score={:.3f}",
                    requested_goal.x(), requested_goal.y(), requested_goal.z(),
                    planning_goal_p_.x(), planning_goal_p_.y(), planning_goal_p_.z(),
                    (best - requested_goal.cast<double>()).norm(), best_score);
            }
            if (best.allFinite() && usable_projection_radius_valid &&
                (best.cast<double>() - requested_goal.cast<double>()).norm() <=
                    usable_projection_radius + 1.0e-6 && map_ptr_->contains(best) &&
                map_ptr_->classify(
                    best.cast<double>(), navigation_world_model::GridLayer::kInflated) ==
                    navigation_world_model::CellState::kKnownFree &&
                map_ptr_->isSegmentTraversable(
                    best.cast<double>(), best.cast<double>(),
                    navigation_world_model::GridLayer::kInflated,
                    navigation_world_model::UnknownPolicy::kRequireKnownFree)) {
                acceptance_endpoint_lease_ = best;
                acceptance_endpoint_lease_request_id_ = route_snapshot_.has_value()
                    ? route_snapshot_->request_id : 0U;
                acceptance_endpoint_lease_route_revision_ = route_snapshot_.has_value()
                    ? route_snapshot_->route_revision : 0U;
                acceptance_endpoint_lease_waypoint_index_ = route_snapshot_.has_value()
                    ? route_snapshot_->active_waypoint_index : 0U;
            }
            return planning_goal_p_;
        }

        // UNKNOWN remains governed by the configured policy. OCCUPIED and
        // OUT_OF_MAP terminal points may be projected into the mission
        // acceptance ball; all other states retain the exact-goal contract.
        // OUT_OF_MAP is included because a rolling local map can place a
        // reachable waypoint just beyond its current window even though a
        // certified acceptance point is already available inside it.
        if ((requested_goal_inflated_state_ !=
                 navigation_world_model::CellState::kOccupied &&
             requested_goal_inflated_state_ !=
                 navigation_world_model::CellState::kOutOfMap) ||
            !usable_projection_radius_valid) {
            return requested_goal;
        }

        const auto candidate = map_ptr_->nearestNotOccupied(
            requested_goal.cast<double>(), navigation_world_model::GridLayer::kInflated,
            usable_projection_radius);
        if (!candidate || !candidate->allFinite() || !map_ptr_->contains(*candidate) ||
            (*candidate - requested_goal.cast<double>()).norm() >
                usable_projection_radius + 1.0e-6) {
            return requested_goal;
        }

        planning_goal_inflated_state_ = map_ptr_->classify(
            *candidate, navigation_world_model::GridLayer::kInflated);
        if (!navigation_world_model::isCellTraversable(
                planning_goal_inflated_state_, cfg_.unknown_space_policy)) {
            planning_goal_inflated_state_ = requested_goal_inflated_state_;
            return requested_goal;
        }

        planning_goal_p_ = *candidate;
        // The nearest free voxel is only a topological escape from the
        // occupied terminal voxel.  It can still leave the vehicle exactly
        // on the inflated frontier, where the certificate tube or normal
        // tracking error moves the command back into OCCUPIED on the next
        // solve.  Walk farther in the same certified escape direction while
        // the point remains inside the mission acceptance ball.  The loop is
        // bounded by that finite ball and every added segment is checked by
        // the same inflated-layer/UNKNOWN oracle; it never enlarges the
        // acceptance radius or relaxes collision validation.
        const auto geometry = map_ptr_->geometry();
        const auto escape = candidate->cast<double>() - requested_goal.cast<double>();
        const double escape_distance = escape.norm();
        const double interior_step = geometry.inflated_resolution_m;
        std::size_t interior_steps = 0U;
        if (std::isfinite(interior_step) && interior_step > 0.0 &&
            std::isfinite(escape_distance) && escape_distance > 1.0e-9) {
            const auto escape_direction = escape / escape_distance;
            while (true) {
                const auto interior_candidate = planning_goal_p_.cast<double>() +
                    escape_direction * interior_step;
                const double interior_error =
                    (interior_candidate - requested_goal.cast<double>()).norm();
                if (!interior_candidate.allFinite() || !map_ptr_->contains(interior_candidate) ||
                    !std::isfinite(interior_error) ||
                    interior_error > usable_projection_radius + 1.0e-6) {
                    break;
                }
                const auto interior_state = map_ptr_->classify(
                    interior_candidate, navigation_world_model::GridLayer::kInflated);
                if (!navigation_world_model::isCellTraversable(
                        interior_state, cfg_.unknown_space_policy) ||
                    !map_ptr_->isSegmentTraversable(
                        planning_goal_p_.cast<double>(), interior_candidate,
                        navigation_world_model::GridLayer::kInflated,
                        cfg_.unknown_space_policy)) {
                    break;
                }
                planning_goal_p_ = interior_candidate;
                planning_goal_inflated_state_ = interior_state;
                ++interior_steps;
            }
        }
        goal_endpoint_adjusted_ = true;
        if (planning_goal_p_.allFinite() &&
            (planning_goal_p_.cast<double>() - requested_goal.cast<double>()).norm() <=
                usable_projection_radius + 1.0e-6 && map_ptr_->contains(planning_goal_p_) &&
            planning_goal_inflated_state_ ==
                navigation_world_model::CellState::kKnownFree &&
            map_ptr_->isSegmentTraversable(
                planning_goal_p_.cast<double>(), planning_goal_p_.cast<double>(),
                navigation_world_model::GridLayer::kInflated,
                navigation_world_model::UnknownPolicy::kRequireKnownFree)) {
            acceptance_endpoint_lease_ = planning_goal_p_;
            acceptance_endpoint_lease_request_id_ = route_snapshot_.has_value()
                ? route_snapshot_->request_id : 0U;
            acceptance_endpoint_lease_route_revision_ = route_snapshot_.has_value()
                ? route_snapshot_->route_revision : 0U;
            acceptance_endpoint_lease_waypoint_index_ = route_snapshot_.has_value()
                ? route_snapshot_->active_waypoint_index : 0U;
        }
        planner_context_->warn(
            " -- [planner] requested goal voxel is occupied or out-of-map; planning to acceptance-safe "
            "endpoint requested=({:.3f},{:.3f},{:.3f}) planning=({:.3f},{:.3f},{:.3f}) "
            "offset={:.3f} radius={:.3f} usable_radius={:.3f}",
            requested_goal.x(), requested_goal.y(), requested_goal.z(),
            planning_goal_p_.x(), planning_goal_p_.y(), planning_goal_p_.z(),
            (planning_goal_p_.cast<double>() - requested_goal.cast<double>()).norm(),
            goal_acceptance_radius_m_, usable_projection_radius);
        if (interior_steps > 0U) {
            planner_context_->info(
                " -- [planner] moved acceptance-safe endpoint inward by {} inflated steps",
                interior_steps);
        }
        return planning_goal_p_;
    }

    Planner::Planner
            (const std::string &cfg_path,
             const navigation_planner_context::PlannerRuntimeContext::Ptr &planner_context,
             navigation_world_model::WorldModelViewPtr map_ptr,
             const std::optional<navigation_planning::DynamicLimits> &mission_limits,
             navigation_world_model::WorldCommitAuthorizer& commit_authorizer
            ) : cfg_(Config(cfg_path, mission_limits)), map_ptr_(std::move(map_ptr)),
                commit_authorizer_(&commit_authorizer), planner_context_(planner_context) {

        const auto world_geometry = map_ptr_->geometry();
        cfg_.bindWorldGeometry(world_geometry);
        planner_context_->setResolution(cfg_.resolution);
        planner_context_->setVisualizationEn(cfg_.visualization_en);
        exp_traj_opt_ = std::make_shared<traj_opt::ExpTrajOpt>(cfg_.exp_traj_cfg, planner_context_);
        back_traj_opt_ = std::make_shared<traj_opt::BackupTrajOpt>(cfg_.back_traj_cfg, planner_context_);
        nominal_exp_max_velocity_mps_ = cfg_.exp_traj_cfg.max_vel;
        nominal_backup_max_velocity_mps_ = cfg_.back_traj_cfg.max_vel;
        yaw_traj_opt_ = std::make_shared<traj_opt::YawTrajOpt>(
            cfg_.yaw_rate_max_rad_s, cfg_.yaw_acceleration_max_rad_s2);
        const double occupied_inflation_radius = world_geometry.occupied_inflation_radius_m;
        if (occupied_inflation_radius + 1.0e-9 < cfg_.robot_r) {
            throw std::invalid_argument(
                    "mapping inflation radius is smaller than planner robot radius");
        }
        astar_ptr_ = std::make_shared<path_search::Astar>(
            cfg_.astar_cfg, planner_context_, map_ptr_, cfg_.astar_search_time_limit_s);
        cg_ptr_ = std::make_shared<CorridorGenerator>(planner_context_, map_ptr_,
                                                      cfg_.corridor_bound_distance_m,
                                                      cfg_.corridor_segment_max_length_m,
                                                      cfg_.resolution,
                                                      cfg_.robot_r,
                                                      cfg_.iris_iter_num,
                                                      unknownPolicy());
        cg_ptr_->SetLineNeighborList(cfg_.seed_line_neighbour);


        time_consuming_.resize(8);

        robot_state_.rcv = false;
        planner_process_start_WT_ = planner_context_->getSimTime();
        fov_checker_ = std::make_shared<FOVChecker>(FOVType::OMNI,
                                                    -1.0,
                                                    -35.0,
                                                    35.0);

        const int neighbor_step = static_cast<int>(std::ceil(
            static_cast<long double>(cfg_.robot_r) /
            static_cast<long double>(cfg_.resolution)));
        astar_ptr_->setFineInfNeighbors(neighbor_step);
    }

    bool Planner::authorizeAndStage(CandidateCommandBundle&& candidate) {
        if (commit_authorizer_ == nullptr || !map_ptr_) {
            latest_commit_decision_.store(static_cast<int>(
                navigation_world_model::WorldCommitDecision::kCandidateRejected));
            planner_context_->error(" -- [planner] command rejected: no WorldModel commit authorizer");
            return false;
        }
        const double maximum_yaw_rate = candidate.yaw.getMaxVelRate();
        const double maximum_yaw_acceleration = candidate.yaw.getMaxAccRate();
        latest_candidate_maximum_yaw_rate_rad_s_ = maximum_yaw_rate;
        latest_candidate_maximum_yaw_acceleration_rad_s2_ =
            maximum_yaw_acceleration;
        if (!candidateYawRateWithinLimit(candidate, cfg_.yaw_rate_max_rad_s)) {
            latest_commit_decision_.store(static_cast<int>(
                navigation_world_model::WorldCommitDecision::kCandidateRejected));
            planner_context_->warn(
                " -- [planner] command rejected by final yaw-rate certificate: "
                "maximum={} limit={}",
                maximum_yaw_rate, cfg_.yaw_rate_max_rad_s);
            return false;
        }
        if (!std::isfinite(maximum_yaw_acceleration) ||
            maximum_yaw_acceleration > cfg_.yaw_acceleration_max_rad_s2 + 1.0e-6) {
            latest_commit_decision_.store(static_cast<int>(
                navigation_world_model::WorldCommitDecision::kCandidateRejected));
            planner_context_->warn(
                " -- [planner] command rejected by final yaw-acceleration certificate: "
                "maximum={} limit={}",
                maximum_yaw_acceleration, cfg_.yaw_acceleration_max_rad_s2);
            return false;
        }
        const auto command_identity = commandIdentitySnapshot();
        if (!command_identity.valid()) {
            latest_commit_decision_.store(static_cast<int>(
                navigation_world_model::WorldCommitDecision::kCandidateRejected));
            planner_context_->error(
                " -- [planner] command rejected: no mission command identity was set");
            return false;
        }
        candidate.localization_epoch = command_identity.localization_epoch;
        candidate.goal_epoch = command_identity.goal_epoch;
        candidate.request_id = command_identity.request_id;
        const double authorization_wall_time = planner_context_->getSimTime();
        if (route_snapshot_.has_value()) {
            const double begin_tt = std::max(
                0.0, authorization_wall_time - candidate.start_wall_time);
            const double route_regression_tolerance_m =
                navigation_mission::RouteProgressConfig{}.backtrack_tolerance_m;
            bool bounded_terminal_stop_recovery = false;
            if (candidate.terminal_stop &&
                route_snapshot_->active_waypoint_index <
                    route_snapshot_->waypoints.size()) {
                const auto& active_waypoint = route_snapshot_->waypoints[
                    route_snapshot_->active_waypoint_index];
                const Eigen::Vector3d candidate_start =
                    candidate.position.getState(
                        std::clamp(begin_tt, 0.0,
                                   candidate.position.getTotalDuration())).col(0);
                const Eigen::Vector3d candidate_end =
                    candidate.position.getState(
                        candidate.position.getTotalDuration()).col(0);
                const double recovery_radius =
                    active_waypoint.acceptance_radius_m + cfg_.tracking_error_budget_m;
                const bool candidate_is_bounded_correction =
                    active_waypoint.behavior ==
                        navigation_mission::MissionWaypoint::Behavior::Stop &&
                    candidate_start.allFinite() &&
                    candidate_end.allFinite() &&
                    std::isfinite(recovery_radius) && recovery_radius > 0.0 &&
                    (candidate_start - active_waypoint.position_enu).norm() <=
                        recovery_radius + 1.0e-9 &&
                    (candidate_end - active_waypoint.position_enu).norm() <=
                        active_waypoint.acceptance_radius_m + 1.0e-9;

                // An emergency brake can stop beyond the waypoint acceptance
                // ball. A later PlanFromRest correction back to that same STOP
                // is legitimate only when the previous command was an actual
                // emergency bundle and its analytic endpoint bounds the
                // measured recovery start. This is a proof from the execution
                // history, not a wider route-regression tolerance: the route
                // fold gate is bypassed only for this bounded STOP correction;
                // world, dynamic, yaw, anchor, and handoff certificates still
                // authorize the candidate independently.
                const auto previous = cmd_traj_info_.snapshot();
                const bool previous_was_emergency =
                    !previous.empty && previous.emergency_brake &&
                    previous.identity.localization_epoch ==
                        command_identity.localization_epoch &&
                    previous.identity.goal_epoch == command_identity.goal_epoch &&
                    previous.identity.request_id == command_identity.request_id;
                Eigen::Vector3d previous_endpoint =
                    Eigen::Vector3d::Constant(
                        std::numeric_limits<double>::quiet_NaN());
                if (previous_was_emergency) {
                    previous_endpoint = previous.position.getState(
                        previous.position.getTotalDuration()).col(0);
                }
                const double emergency_endpoint_distance =
                    previous_endpoint.allFinite()
                        ? (previous_endpoint - active_waypoint.position_enu).norm()
                        : std::numeric_limits<double>::quiet_NaN();
                const bool emergency_bounded_correction =
                    previous_was_emergency && candidate_start.allFinite() &&
                    candidate_end.allFinite() &&
                    std::isfinite(emergency_endpoint_distance) &&
                    std::isfinite(cfg_.tracking_error_budget_m) &&
                    cfg_.tracking_error_budget_m >= 0.0 &&
                    (candidate_start - active_waypoint.position_enu).norm() <=
                        emergency_endpoint_distance +
                        cfg_.tracking_error_budget_m + 1.0e-9 &&
                    (candidate_end - active_waypoint.position_enu).norm() <=
                        active_waypoint.acceptance_radius_m + 1.0e-9;
                bounded_terminal_stop_recovery =
                    candidate_is_bounded_correction || emergency_bounded_correction;
                if (emergency_bounded_correction && !candidate_is_bounded_correction) {
                    planner_context_->info(
                        " -- [planner] allowing bounded STOP correction after "
                        "certified emergency endpoint distance={} start_distance={} "
                        "acceptance_radius={}",
                        emergency_endpoint_distance,
                        (candidate_start - active_waypoint.position_enu).norm(),
                        active_waypoint.acceptance_radius_m);
                }
            }
            const auto route_certificate = certifyMainRouteRegression(
                candidate, *route_snapshot_, begin_tt,
                route_regression_tolerance_m, bounded_terminal_stop_recovery);
            if (route_certificate.applicable && !route_certificate.valid) {
                latest_commit_decision_.store(static_cast<int>(
                    navigation_world_model::WorldCommitDecision::kCandidateRejected));
                planner_context_->warn(
                    " -- [planner] command rejected by MAIN route-regression "
                    "certificate: maximum={} tolerance={} first_time={}",
                    route_certificate.maximum_regression_m,
                    route_regression_tolerance_m,
                    route_certificate.first_violation_time_s);
                return false;
            }
        }
        const auto pinned_identity = map_ptr_->identity();
        const auto lease = commit_authorizer_->latest();
        if (!lease) {
            latest_commit_decision_.store(static_cast<int>(
                navigation_world_model::WorldCommitDecision::kNoPublishedWorld));
            planner_context_->warn(" -- [planner] command rejected: no published WorldModel");
            return false;
        }
        const auto certificate_policy = candidateCertificatePolicy(candidate, unknownPolicy());
        // Validate against the newest immutable view before entering the
        // publication gate. The certificate also carries the conservative
        // swept region; the authorizer may retain it across unrelated map
        // revisions only when immutable change provenance proves disjointness.
        const auto validation = validateExecutableCandidate(
            *lease.view, candidate, authorization_wall_time, certificate_policy);
        if (!validation.valid || !validation.protected_region.valid()) {
            latest_commit_decision_.store(static_cast<int>(
                navigation_world_model::WorldCommitDecision::kCandidateRejected));
            planner_context_->warn(
                " -- [planner] command rejected by latest WorldModel: "
                "time={} reason={} role={} cell_state={} position=({}, {}, {})",
                validation.first_blocked_tt,
                sweptValidationFailureName(validation.failure),
                static_cast<int>(validation.blocked_role),
                static_cast<int>(validation.blocked_cell_state),
                validation.blocked_position.x(), validation.blocked_position.y(),
                validation.blocked_position.z());
            return false;
        }
        CommandCertificate certificate{
            pinned_identity, lease.identity, validation.begin_tt,
            validation.protected_region};
        const auto decision = commit_authorizer_->commitIfCurrentOrUnaffected(
            lease.identity, validation.protected_region,
            [&](const navigation_world_model::WorldValidationLease& commit_lease) {
                if (!commit_lease) return false;
                certificate.validated_world = commit_lease.identity;
                std::lock_guard<std::mutex> commit_guard(solve_commit_mutex_);
                if (solve_cancelled_.load()) return false;
                if (!cmd_traj_info_.canCommitCandidate(candidate)) return false;
                staged_command_candidate_ = StagedCommandCandidate{
                    std::move(candidate), certificate, cmd_traj_info_.nextGeneration(),
                    std::nullopt, false};
                return true;
            });
        latest_commit_decision_.store(static_cast<int>(decision));
        if (decision != navigation_world_model::WorldCommitDecision::kCommitted) {
            planner_context_->warn(" -- [planner] command authorization rejected with reason {}",
                           static_cast<int>(decision));
            return false;
        }
        return true;
    }

    bool Planner::stageCommandHistoryForCandidate(const ExpTraj& exp_traj) {
        if (exp_traj.empty()) return false;
        std::lock_guard<std::mutex> guard(solve_commit_mutex_);
        if (!staged_command_candidate_) return false;
        // The EXP trajectory is only a warm-start hint for the next solve.
        // Promote it together with the execution timeline activation.
        staged_command_candidate_->pending_exp_history = exp_traj;
        staged_command_candidate_->clear_new_goal_on_ack = true;
        return true;
    }

    void Planner::onExecutionTimelineActivated(
            const std::uint64_t generation) noexcept {
        std::lock_guard<std::mutex> guard(solve_commit_mutex_);
        if (!staged_command_candidate_ ||
            staged_command_candidate_->generation != generation) {
            return;
        }
        auto staged = std::move(*staged_command_candidate_);
        if (!cmd_traj_info_.commitCandidate(
                std::move(staged.command), staged.certificate)) {
            planner_context_->error(
                " -- [planner] execution activated generation={} but warm-start "
                "cache synchronization failed; execution remains authoritative",
                generation);
            staged_command_candidate_.reset();
            return;
        }
        if (staged.pending_exp_history.has_value()) {
            last_exp_traj_info_ = std::move(*staged.pending_exp_history);
        }
        if (staged.clear_new_goal_on_ack) gi_.new_goal = false;
        staged_command_candidate_.reset();
    }

    void Planner::discardCommandCandidate() noexcept {
        std::lock_guard<std::mutex> guard(solve_commit_mutex_);
        staged_command_candidate_.reset();
    }

    std::optional<navigation_planning::CandidateBundle>
    Planner::exportStagedCommandCandidate(
            const CandidateCommandBundle& command,
            const CommandCertificate& certificate,
            const std::uint64_t generation,
            const std::uint64_t localization_epoch,
            const std::uint64_t goal_epoch,
            const std::uint64_t request_id,
            const std::int64_t valid_from_ns,
            const std::int64_t valid_until_ns) const {
        if (localization_epoch == 0U || goal_epoch == 0U || request_id == 0U ||
            generation == 0U || valid_from_ns <= 0 || valid_until_ns < valid_from_ns ||
            certificate.validated_world.localization_epoch == 0U ||
            certificate.validated_world.generation == 0U ||
            certificate.validated_world.revision == 0U ||
            certificate.validated_world.observation_stamp_ns <= 0 ||
            command.localization_epoch != localization_epoch ||
            command.goal_epoch != goal_epoch || command.request_id != request_id ||
            command.position.empty() || command.yaw.empty() ||
            !std::isfinite(command.position.start_WT) ||
            !std::isfinite(command.position.getTotalDuration()) ||
            command.position.getTotalDuration() < 0.0) {
            return std::nullopt;
        }

        const double start_wall_time_s = command.position.start_WT;
        const double end_wall_time_s = start_wall_time_s +
            command.position.getTotalDuration();
        if (!std::isfinite(end_wall_time_s) || end_wall_time_s < start_wall_time_s) {
            return std::nullopt;
        }
        const auto start_ns = navigation_common::secondsToNanoseconds(start_wall_time_s);
        const auto end_ns = navigation_common::secondsToNanoseconds(end_wall_time_s);
        const auto declared_end_ns = navigation_common::secondsSumToNanoseconds(
            start_wall_time_s, command.position.getTotalDuration());
        if (!start_ns || !end_ns || !declared_end_ns ||
            *end_ns < *start_ns || *declared_end_ns < *start_ns) return std::nullopt;

        navigation_planning::CandidateBundle candidate;
        candidate.pinned_world_identity = certificate.pinned_world;
        candidate.world_identity = certificate.validated_world;
        candidate.localization_epoch = localization_epoch;
        candidate.goal_epoch = goal_epoch;
        candidate.request_id = request_id;
        candidate.bundle_generation = generation;
        candidate.activation_stamp_ns = valid_from_ns;
        // The trajectory object is the authoritative producer of the wall
        // clock used by both the evaluator and the execution lease. Do not
        // copy the construction-time mirror: a stale/default mirror would
        // make the exported candidate lose its declared endpoint metadata.
        candidate.start_wall_time_s = start_wall_time_s;
        candidate.duration_s = command.position.getTotalDuration();
        candidate.declared_start_ns = *start_ns;
        candidate.declared_end_ns = *declared_end_ns;
        candidate.backup_start_time_s = command.backup_start_tt;
        candidate.backup_available = command.backup_suffix_available;
        candidate.terminal_stop = command.terminal_stop;
        const bool emergency_candidate =
            command.backup_disposition == BackupDisposition::EMERGENCY;
        candidate.kind = emergency_candidate
            ? navigation_planning::CandidateBundleKind::kEmergencyBrake
            : command.backup_suffix_available
                ? navigation_planning::CandidateBundleKind::kMainWithBackup
                : navigation_planning::CandidateBundleKind::kTerminalStop;
        candidate.source = emergency_candidate
            ? navigation_planning::CandidateSource::kEmergency
            : navigation_planning::CandidateSource::kRefined;
        candidate.certificates.dynamics = true;
        candidate.certificates.flatness = true;
        candidate.certificates.world = true;
        // A main-only finite endpoint remains a terminal executable bundle
        // for admission even when the mission behavior is PASS_THROUGH. The
        // explicit semantic bit above is reserved for the STOP mission
        // contract used by runtime renewal/recovery decisions.
        candidate.certificates.terminal_stop =
            candidate.kind == navigation_planning::CandidateBundleKind::kTerminalStop ||
            command.terminal_stop;
        candidate.protected_region = certificate.protected_region;
        candidate.role = emergency_candidate
            ? navigation_planning::CandidateRole::kEmergency
            : navigation_planning::CandidateRole::kMain;
        candidate.role_schedule.reserve(command.roles.size());
        for (const auto& interval : command.roles) {
            candidate.role_schedule.push_back({
                interval.begin_tt, interval.end_tt,
                emergency_candidate
                    ? navigation_planning::CandidateRole::kEmergency
                    : interval.role == CandidateTrajectoryRole::BACKUP
                        ? navigation_planning::CandidateRole::kBackup
                        : navigation_planning::CandidateRole::kMain});
        }
        candidate.valid_from_ns = std::max(valid_from_ns, *start_ns);
        candidate.valid_until_ns = std::min(valid_until_ns, *declared_end_ns);
        if (candidate.valid_until_ns < candidate.valid_from_ns) return std::nullopt;
        candidate.evaluator = [position = command.position,
                               yaw = command.yaw,
                               roles = command.roles,
                               emergency_candidate,
                               start_ns = *start_ns,
                               end_ns = *declared_end_ns] (
                                  const std::int64_t stamp_ns,
                                  navigation_planning::TrajectoryPoint& point) {
            const auto command_time = commandTrajectoryTime(
                stamp_ns, start_ns, end_ns, position.getTotalDuration());
            if (command_time.trajectory_time_s < 0.0) return false;
            const auto state = position.getState(command_time.trajectory_time_s);
            const auto yaw_state = yaw.getState(command_time.trajectory_time_s);
            if (!state.allFinite() || !yaw_state.allFinite()) return false;
            point.position_world = state.col(0);
            point.velocity_world = state.col(1);
            point.acceleration_world = state.col(2);
            point.jerk_world = state.col(3);
            point.yaw = yaw_state(0, 0);
            point.yaw_rate = yaw_state(0, 1);
            point.finished = command_time.finished;
            point.trajectory_time_s = command_time.trajectory_time_s;
            point.role = emergency_candidate
                ? navigation_planning::CandidateRole::kEmergency
                : navigation_planning::CandidateRole::kMain;
            if (emergency_candidate) return true;
            const __int128 elapsed_ns = static_cast<__int128>(stamp_ns) -
                                        static_cast<__int128>(start_ns);
            const __int128 duration_ns = static_cast<__int128>(end_ns) -
                                         static_cast<__int128>(start_ns);
            for (const auto& interval : roles) {
                const auto begin_ns = static_cast<__int128>(std::llround(
                    static_cast<long double>(interval.begin_tt) * 1.0e9L));
                const auto interval_end_ns = static_cast<__int128>(std::llround(
                    static_cast<long double>(interval.end_tt) * 1.0e9L));
                const bool at_declared_endpoint = elapsed_ns == duration_ns &&
                    &interval == &roles.back();
                if (elapsed_ns >= begin_ns &&
                    (elapsed_ns < interval_end_ns || at_declared_endpoint) &&
                    interval.role == CandidateTrajectoryRole::BACKUP) {
                    point.role = navigation_planning::CandidateRole::kBackup;
                    break;
                }
            }
            return true;
        };
        bool route_boundary_required = false;
        if (route_snapshot_.has_value() &&
            route_snapshot_->active_waypoint_index < route_snapshot_->waypoints.size()) {
            const auto& waypoint =
                route_snapshot_->waypoints[route_snapshot_->active_waypoint_index];
            const bool is_pass_through = waypoint.behavior ==
                navigation_mission::MissionWaypoint::Behavior::PassThrough;
            const bool boundary_reached = command.connected_goal || command.terminal_stop;
            route_boundary_required = is_pass_through && boundary_reached;
            if (boundary_reached && (is_pass_through || command.terminal_stop)) {
                Eigen::Vector3d incoming = Eigen::Vector3d::Zero();
                Eigen::Vector3d outgoing = Eigen::Vector3d::Zero();
                for (const auto& segment : route_snapshot_->segments) {
                    if (segment.end_waypoint_index ==
                            route_snapshot_->active_waypoint_index &&
                        segment.tangent.allFinite() && segment.tangent.norm() > 1.0e-9) {
                        incoming = segment.tangent.normalized();
                    }
                    if (segment.start_waypoint_index ==
                            route_snapshot_->active_waypoint_index &&
                        segment.tangent.allFinite() && segment.tangent.norm() > 1.0e-9) {
                        outgoing = segment.tangent.normalized();
                    }
                }
                if (incoming.norm() <= 1.0e-9) incoming = outgoing;
                if (outgoing.norm() <= 1.0e-9) outgoing = incoming;
                const double radius = std::max(
                    navigation_world_model::kGoalCompletionToleranceM,
                    waypoint.acceptance_radius_m);
                if (incoming.norm() > 1.0e-9 && outgoing.norm() > 1.0e-9 &&
                    std::isfinite(radius) && radius > 0.0) {
                    navigation_planning::RouteBoundaryConstraint constraint;
                    constraint.admissible_volume.minimum = waypoint.position_enu -
                        Eigen::Vector3d::Constant(radius);
                    constraint.admissible_volume.maximum = waypoint.position_enu +
                        Eigen::Vector3d::Constant(radius);
                    constraint.junction_index =
                        route_snapshot_->active_waypoint_index;
                    constraint.incoming_tangent = incoming;
                    constraint.outgoing_tangent = outgoing;
                    constraint.corner_speed_mps = command.terminal_stop ? 0.0 :
                        std::max(0.0, pass_through_next_target_.has_value()
                            ? nominal_exp_max_velocity_mps_ : 0.0);
                    candidate.route_boundary_constraint = constraint;
                    candidate.route_boundary_event = navigation_planning::RouteBoundaryEvent{
                        command.terminal_stop
                            ? navigation_planning::RouteBoundaryEventKind::kTerminalStop
                            : navigation_planning::RouteBoundaryEventKind::kPassThrough,
                        route_snapshot_->active_waypoint_index, *declared_end_ns,
                        waypoint.position_enu, incoming, outgoing,
                        constraint.corner_speed_mps};
                }
            }
        }
        if (route_boundary_required &&
            (!candidate.route_boundary_constraint.has_value() ||
             !candidate.route_boundary_event.has_value())) {
            planner_context_->warn(
                " -- [planner] pass-through endpoint reached without a complete "
                "RouteBoundaryConstraint/Event; rejecting candidate");
            return std::nullopt;
        }
        return candidate.valid() ? std::optional<navigation_planning::CandidateBundle>{
            std::move(candidate)} : std::nullopt;
    }

    bool Planner::updateRouteYawReference() noexcept {
        if (!route_snapshot_.has_value()) {
            route_yaw_reference_ = {};
            return false;
        }
        route_yaw_reference_ = computeRouteYawReference(
            *route_snapshot_, solve_state_.p, solve_state_.v, solve_state_.yaw,
            cfg_.route_yaw_config);
        if (!route_yaw_reference_.valid) return false;
        if (route_yaw_reference_.source == RouteYawSource::kRouteLookahead ||
            route_yaw_reference_.source == RouteYawSource::kRouteTurnInPlace) {
            last_route_yaw_target_rad_ = route_yaw_reference_.target_yaw_rad;
        } else if (last_route_yaw_target_rad_.has_value()) {
            route_yaw_reference_.target_yaw_rad = solve_state_.yaw + std::remainder(
                *last_route_yaw_target_rad_ - solve_state_.yaw, 2.0 * M_PI);
        } else {
            last_route_yaw_target_rad_ = route_yaw_reference_.target_yaw_rad;
        }
        return route_yaw_reference_.valid;
    }

    RET_CODE
    Planner::planInitialFromStoppedState(const Vec3f &goal_p,
                                         const double &goal_yaw,
                                         const bool &new_goal) {
        std::lock_guard<std::mutex> guard(replan_lock_);
        {
            std::lock_guard<std::mutex> state_guard(drone_state_mutex_);
            solve_state_ = robot_state_;
            solve_acceleration_estimated_ = robot_acceleration_estimated_;
            solve_jerk_estimated_ = robot_jerk_estimated_;
        }
        const AbsoluteDeadline solve_deadline(
                planner_context_->getSimTime(), cfg_.solve_deadline_s);
        candidate_terminal_stop_active_ = false;
        solve_stage_.store(1);
        latest_commit_decision_.store(static_cast<int>(
            navigation_world_model::WorldCommitDecision::kNotAttempted));
        latest_replan.reset();
        latest_replan.setGoal(goal_p, goal_yaw, solve_state_);
        const Vec3f planning_goal = resolveGoalForPlanning(goal_p);
        if (solve_state_.rcv == false) {
            planner_context_->warn(" -- [planner] in [PlanFromRest]: No odom, force return.");
            latest_replan.setRetCode(PlannerResultCode::PLANNER_NO_ODOM);
            return FAILED;
        }
        if (!updateRouteYawReference()) {
            planner_context_->warn(
                " -- [planner] in [PlanFromRest]: invalid semantic route-yaw reference");
            latest_replan.setRetCode(PlannerResultCode::PLANNER_INVALID_ROUTE);
            return FAILED;
        }
        gi_.goal_p = planning_goal;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        gi_.goal_valid = true;
        vec_Vec3f viz_pts{goal_p, solve_state_.p};

        {
            TimeConsuming t_viz("viz goal path", false);
            planner_context_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        /// 1) First, shift the start_point to free space only when necessary.
        // Keep the measured pose as the planner start when it is already
        // traversable. Snapping a valid pose to the centre of its nearest
        // inflated voxel can move the command toward a nearby obstacle and
        // make the backup CIRI seed fail even though the measured pose has
        // clearance. The fallback remains on the inflated layer so a truly
        // occupied or out-of-map pose is still corrected before planning.
        const auto measured_start_cell = map_ptr_->classify(
                solve_state_.p.cast<double>(), navigation_world_model::GridLayer::kInflated);
        const auto measured_start_is_traversable =
                map_ptr_->contains(solve_state_.p) &&
                navigation_world_model::isCellTraversable(
                        measured_start_cell,
                        unknownPolicy());
        Vec3f local_star_pt = solve_state_.p;
        if (!measured_start_is_traversable) {
            const auto nearest_start = map_ptr_->nearestNotOccupied(
                    solve_state_.p, navigation_world_model::GridLayer::kInflated, 3.0);
            if (!nearest_start) {
                planner_context_->error(
                        " -- [planner] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
                latest_replan.setRetCode(PlannerResultCode::PLANNER_NO_START_POINT);
                return FAILED;
            }
            local_star_pt = *nearest_start;
            planner_context_->warn(
                    " -- [planner] PlanFromRest shifted non-traversable measured start: "
                    "cell_state={} measured=({:.3f},{:.3f},{:.3f}) local_start=({:.3f},{:.3f},{:.3f}) "
                    "shift={} traversable={}",
                    static_cast<int>(measured_start_cell),
                    solve_state_.p.x(), solve_state_.p.y(), solve_state_.p.z(),
                    local_star_pt.x(), local_star_pt.y(), local_star_pt.z(),
                    (local_star_pt - solve_state_.p).norm(),
                    measured_start_is_traversable ? 1 : 0);
        }
        latest_replan.setLocalStartP(local_star_pt);

        /// 2) Generate Exp traj
        ExpTraj exp_traj_info;
        BackupTraj back_traj_info;
        last_exp_traj_info_.setEmpty();
        local_start_p_ = local_star_pt;
        ExpTraj previous_exp_snapshot = last_exp_traj_info_;
        RET_CODE exp_ret_code = generateExpTraj(
                previous_exp_snapshot, exp_traj_info, solve_deadline);
        //GenerateRestToRestExpTraj(local_star_pt, exp_traj_info);
        if (exp_ret_code == FAILED) {
            latest_replan.setRetCode(classifySolveFailure(
                solve_deadline, false, PlannerResultCode::PLANNER_EXP_FAILED));
            planner_context_->warn(" -- [planner] in [PlanFromRest] GenerateExpTrajectory failed with {}.",
                           RET_CODE_STR[exp_ret_code].c_str());
            return FAILED;
        } else {
            planner_context_->info(" -- [planner] in [PlanFromRest] GenerateExpTrajectory SUCCESS.");
        }

        back_traj_info.setEmpty();
        solve_stage_.store(5);
        if (solve_deadline.expired(planner_context_->getSimTime()) ||
            solve_deadline.steadyExpired()) {
            planner_context_->warn(" -- [planner] solve deadline exhausted before backup stage");
            latest_replan.setRetCode(classifySolveFailure(solve_deadline));
            return FAILED;
        }
        RET_CODE back_ret_code = generateBackupTrajectory(
                exp_traj_info, back_traj_info, solve_deadline);

        if (solve_cancelled_.load() ||
            solve_deadline.expired(planner_context_->getSimTime()) ||
            solve_deadline.steadyExpired()) {
            if (!solve_cancelled_.load()) {
                planner_context_->warn(" -- [planner] solve deadline exhausted during backup stage");
            }
            latest_replan.setRetCode(classifySolveFailure(solve_deadline));
            return FAILED;
        }

        if (!backupResultMayBuildCommandCandidate(back_ret_code)) {
            latest_replan.setRetCode(classifySolveFailure(
                solve_deadline, false, classifyBackupResult(back_ret_code)));
            planner_context_->warn(
                " -- [planner] in [PlanFromRest]: backup result is not executable; "
                "leaving CmdTraj unchanged");
            return FAILED;
        }

        if (back_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                planner_context_->info(" -- [planner] in [PlanFromRest] generateBackupTrajectory SUCCESS.");
            }

            auto candidate = CmdTraj::buildCandidate(
                exp_traj_info, &back_traj_info, BackupDisposition::SUCCESS,
                candidate_terminal_stop_active_);
            if (!candidate || !authorizeAndStage(std::move(*candidate))) {
                latest_replan.setRetCode(classifySolveFailure(
                    solve_deadline, false, PlannerResultCode::PLANNER_CANDIDATE_REJECTED));
                return FAILED;
            }
            if (!stageCommandHistoryForCandidate(exp_traj_info)) {
                planner_context_->error(" -- [planner] failed to stage EXP history with candidate");
                discardCommandCandidate();
                latest_replan.setRetCode(PlannerResultCode::PLANNER_CANDIDATE_REJECTED);
                return FAILED;
            }

            // For visualization
            {
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                planner_context_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
                latest_replan.setRetCode(PlannerResultCode::PLANNER_SUCCESS_WITH_BACKUP);
            }

            return SUCCESS;
        } else if (back_ret_code == FINISH || back_ret_code == NO_NEED) {
            if (cfg_.print_log) {
                planner_context_->info(" -- [planner] in [PlanFromRest] generateBackupTrajectory Finish or NO_NEED.");
            }
            const auto disposition = back_ret_code == FINISH
                ? BackupDisposition::FINISH : BackupDisposition::NO_NEED;
            auto candidate = CmdTraj::buildCandidate(
                exp_traj_info, nullptr, disposition,
                candidate_terminal_stop_active_);
            if (!candidate || !authorizeAndStage(std::move(*candidate))) {
                latest_replan.setRetCode(classifySolveFailure(
                    solve_deadline, false, PlannerResultCode::PLANNER_CANDIDATE_REJECTED));
                return FAILED;
            }
            if (!stageCommandHistoryForCandidate(exp_traj_info)) {
                planner_context_->error(" -- [planner] failed to stage EXP history with candidate");
                discardCommandCandidate();
                latest_replan.setRetCode(PlannerResultCode::PLANNER_CANDIDATE_REJECTED);
                return FAILED;
            }

            // For visualization
            TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
            {
                planner_context_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }
            latest_replan.setRetCode(PlannerResultCode::PLANNER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        // A candidate is not safe to commit without a feasible backup.  Keep
        // the previously committed atomic bundle so the runtime can drain its
        // existing safety suffix or enter its fail-closed handover path.
        planner_context_->warn(
                " -- [planner] in [PlanFromRest] backup generation returned [{}]; "
                "rejecting candidate because backup is required",
                RET_CODE_STR[back_ret_code].c_str());
        latest_replan.setRetCode(classifySolveFailure(
            solve_deadline, false, classifyBackupResult(back_ret_code)));
        return FAILED;
    }


    RET_CODE
    Planner::planSuccessorFromExecutionAnchor(const Vec3f &goal_p,
                                              const double &goal_yaw,
                                              const bool &new_goal) {
        TimeConsuming replan_total_t("ReplanOnce", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        {
            std::lock_guard<std::mutex> state_guard(drone_state_mutex_);
            solve_state_ = robot_state_;
            solve_acceleration_estimated_ = robot_acceleration_estimated_;
            solve_jerk_estimated_ = robot_jerk_estimated_;
        }
        const AbsoluteDeadline solve_deadline(
                planner_context_->getSimTime(), cfg_.solve_deadline_s);
        candidate_terminal_stop_active_ = false;
        solve_stage_.store(1);
        latest_commit_decision_.store(static_cast<int>(
            navigation_world_model::WorldCommitDecision::kNotAttempted));

        const Vec3f planning_goal = resolveGoalForPlanning(goal_p);
        gi_.goal_p = planning_goal;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        gi_.goal_valid = true;
        latest_replan.reset();
        latest_replan.setGoal(goal_p, goal_yaw, solve_state_);
        if (!updateRouteYawReference()) {
            planner_context_->warn(
                " -- [planner] in [ReplanOnce]: invalid semantic route-yaw reference");
            latest_replan.setRetCode(PlannerResultCode::PLANNER_INVALID_ROUTE);
            return FAILED;
        }

        vec_Vec3f viz_pts{goal_p, solve_state_.p};

        {
            TimeConsuming t_viz("tviz", false);
            planner_context_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        /// 1) Replan EXP traj
        ExpTraj exp_traj_info;
        TimeConsuming t_exp("t_exp", false);
        ExpTraj previous_exp_snapshot = last_exp_traj_info_;
        RET_CODE exp_ret_code = generateExpTraj(
                previous_exp_snapshot, exp_traj_info, solve_deadline);
        time_consuming_[GENERATE_EXP_TRAJ] = t_exp.stop();

        if (exp_ret_code == FAILED) {
            latest_replan.setRetCode(classifySolveFailure(
                solve_deadline, false, PlannerResultCode::PLANNER_EXP_FAILED));
            planner_context_->warn(" -- [planner] in [ReplanOnce]: GenerateExpTrajectory failed, force return");
            return FAILED;
        } else if (exp_ret_code == NEW_TRAJ) {
            if (cfg_.print_log) {
                planner_context_->info(" -- [planner] in [ReplanOnce]: Last exp traj end, switch to new traj.");
            }
            return NEW_TRAJ;
        } else if (exp_ret_code == EMER) {
            planner_context_->warn(" -- [planner] in [ReplanOnce]: Replan failed, switch to emer.");
            return EMER;
        } else if (exp_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                planner_context_->info(" -- [planner] in [ReplanOnce]: Replan a new exp traj success.");
            }
        } else if (exp_ret_code == NO_NEED &&
                   !expResultMayBuildCommandCandidate(exp_ret_code)) {
            // generateExpTraj returns the historical EXP snapshot for NO_NEED.
            // It is planner history, not a new executable candidate.  Keep the
            // currently committed immutable command bundle unchanged and let
            // the runtime revalidate/expose that bundle as the retained command.
            if (cfg_.print_log) {
                planner_context_->info(
                    " -- [planner] in [ReplanOnce]: No new EXP trajectory is needed; "
                    "retain the committed command without generating or committing backup.");
            }
            latest_replan.setRetCode(PlannerResultCode::PLANNER_SUCCESS);
            return NO_NEED;
        }

        {
            TimeConsuming t_viz("tviz", false);
            planner_context_->vizYawTraj(exp_traj_info.posTraj(), exp_traj_info.yawTraj());
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        BackupTraj back_traj_info;
        // 2）生成back轨迹
        solve_stage_.store(5);
        TimeConsuming t_back("t_back", false);
        if (solve_deadline.expired(planner_context_->getSimTime()) ||
            solve_deadline.steadyExpired()) {
            planner_context_->warn(" -- [planner] solve deadline exhausted before backup stage");
            latest_replan.setRetCode(classifySolveFailure(solve_deadline));
            return FAILED;
        }
        RET_CODE back_ret_code = generateBackupTrajectory(
                exp_traj_info, back_traj_info, solve_deadline);
        time_consuming_[GENERATE_BACK_TRAJ] = t_back.stop();

        {
            ft += time_consuming_[EPX_TRAJ_FRONTEND] + time_consuming_[BACK_TRAJ_FRONTEND];
            ft_cnt++;
            bt += time_consuming_[BACK_TRAJ_OPT] + time_consuming_[EXP_TRAJ_OPT];
            bt_cnt++;
        }

        double replan_dt = replan_total_t.stop();
        if (solve_deadline.expired(planner_context_->getSimTime()) ||
            solve_deadline.steadyExpired() ||
            replan_dt > cfg_.solve_deadline_s) {
            planner_context_->warn(" -- [planner] in [ReplanOnce]: Replan overtime, check parameters, replan dt = {}.", replan_dt);
            latest_replan.setRetCode(classifySolveFailure(
                solve_deadline, replan_dt > cfg_.solve_deadline_s));
            return FAILED;
        }

        if (solve_cancelled_.load()) {
            latest_replan.setRetCode(classifySolveFailure(solve_deadline));
            return FAILED;
        }

        if (!backupResultMayBuildCommandCandidate(back_ret_code)) {
            latest_replan.setRetCode(classifySolveFailure(
                solve_deadline, false, classifyBackupResult(back_ret_code)));
            planner_context_->warn(
                " -- [planner] in [ReplanOnce]: backup result is not executable; "
                "leaving CmdTraj unchanged");
            return FAILED;
        }

        // A map revision can make a previously exploratory trajectory unsafe
        // immediately after this solve. If backup visibility says the new EXP
        // is entirely known-free, blindly committing it as a main-only bundle
        // would erase the older bundle's braking suffix at exactly the
        // boundary where it is needed. Retain that suffix until the runtime
        // validates the old bundle and/or a later solve produces another
        // atomic backup bundle. A new goal is excluded because the old suffix
        // belongs to a different command boundary; an active backup is kept
        // until its finite stop endpoint.
        const double command_time_now = planner_context_->getSimTime() -
                cmd_traj_info_.getStartWallTime();
        const bool retain_backup_capable_command =
                shouldRetainBackupCapableCommand(
                    new_goal, cmd_traj_info_.backupTrajectoryAvailable());
        if ((back_ret_code == NO_NEED || back_ret_code == FINISH) &&
            retain_backup_capable_command) {
            latest_replan.setRetCode(PlannerResultCode::PLANNER_SUCCESS);
            planner_context_->info(
                " -- [planner] retaining backup-capable command instead of replacing it "
                "with a main-only candidate elapsed={} backup_start={} result={}",
                command_time_now, cmd_traj_info_.getBackupTrajStartTT(),
                RET_CODE_STR[back_ret_code].c_str());
            return NO_NEED;
        }

        if (back_ret_code == SUCCESS) {
            auto candidate = CmdTraj::buildCandidate(
                exp_traj_info, &back_traj_info, BackupDisposition::SUCCESS,
                candidate_terminal_stop_active_);
            if (!candidate) {
                planner_context_->warn(
                    " -- [planner] rejected malformed main+backup candidate: "
                    "exp_durations={} backup_durations={} backup_start={}",
                    trajectoryDurationSummary(exp_traj_info.posTraj()),
                    trajectoryDurationSummary(back_traj_info.posTraj()),
                    back_traj_info.getStartTT());
                latest_replan.setRetCode(classifySolveFailure(
                    solve_deadline, false, PlannerResultCode::PLANNER_CANDIDATE_REJECTED));
                return FAILED;
            }
            if (!authorizeAndStage(std::move(*candidate))) {
                latest_replan.setRetCode(classifySolveFailure(
                    solve_deadline, false, PlannerResultCode::PLANNER_CANDIDATE_REJECTED));
                return FAILED;
            }
            if (!stageCommandHistoryForCandidate(exp_traj_info)) {
                planner_context_->error(" -- [planner] failed to stage EXP history with candidate");
                discardCommandCandidate();
                latest_replan.setRetCode(PlannerResultCode::PLANNER_CANDIDATE_REJECTED);
                return FAILED;
            }

            {
                // For visualization
                TimeConsuming t_viz("tviz", false);
                planner_context_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            latest_replan.setRetCode(PLANNER_SUCCESS_WITH_BACKUP);
            if (cfg_.print_log)
                planner_context_->info(" -- [planner] in [ReplanOnce]: Replan a new back traj success, all replan success.");
            return SUCCESS;
        } else if (back_ret_code == NO_NEED) {
            // 这次生成backup轨迹的点没有意义,
            auto candidate = CmdTraj::buildCandidate(
                exp_traj_info, nullptr, BackupDisposition::NO_NEED,
                candidate_terminal_stop_active_);
            if (!candidate || !authorizeAndStage(std::move(*candidate))) {
                latest_replan.setRetCode(classifySolveFailure(
                    solve_deadline, false, PlannerResultCode::PLANNER_CANDIDATE_REJECTED));
                return FAILED;
            }
            if (!stageCommandHistoryForCandidate(exp_traj_info)) {
                planner_context_->error(" -- [planner] failed to stage EXP history with candidate");
                discardCommandCandidate();
                latest_replan.setRetCode(PlannerResultCode::PLANNER_CANDIDATE_REJECTED);
                return FAILED;
            }


            {
                TimeConsuming t_viz("tviz", false);
                planner_context_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();

            }

            if (cfg_.print_log)
                planner_context_->info(" -- [planner] in [ReplanOnce]: No need back traj success, all replan success.");
            latest_replan.setRetCode(PLANNER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        } else if (back_ret_code == FINISH) {
            // Which means the exp traj is all in known free, no need for backup traj
            auto candidate = CmdTraj::buildCandidate(
                exp_traj_info, nullptr, BackupDisposition::FINISH,
                candidate_terminal_stop_active_);
            if (!candidate || !authorizeAndStage(std::move(*candidate))) {
                latest_replan.setRetCode(classifySolveFailure(
                    solve_deadline, false, PlannerResultCode::PLANNER_CANDIDATE_REJECTED));
                return FAILED;
            }
            if (!stageCommandHistoryForCandidate(exp_traj_info)) {
                planner_context_->error(" -- [planner] failed to stage EXP history with candidate");
                discardCommandCandidate();
                latest_replan.setRetCode(PlannerResultCode::PLANNER_CANDIDATE_REJECTED);
                return FAILED;
            }

            {
                TimeConsuming t_viz("tviz", false);
                planner_context_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            if (cfg_.print_log)
                planner_context_->info(" -- [planner] in [ReplanOnce]: No need back traj success, all replan success.");
            latest_replan.setRetCode(PLANNER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        // Main and backup form one atomic safety bundle.  A failed backup
        // must leave CmdTraj untouched so the runtime can retain/drain the
        // previously committed suffix instead of executing main-only EXP.
        planner_context_->warn(
                " -- [planner] in [ReplanOnce]: backup generation returned {}; "
                "rejecting candidate because backup is required",
                RET_CODE_STR[back_ret_code].c_str());
        latest_replan.setRetCode(classifySolveFailure(
            solve_deadline, false, classifyBackupResult(back_ret_code)));
        return FAILED;
    }

    navigation_planning::PlanningOutcome Planner::plan(
            const navigation_planning::PlanningRequest& request) {
        const auto started = std::chrono::steady_clock::now();
        navigation_planning::PlanningOutcome outcome;
        outcome.failure_stage = navigation_planning::PlanningFailureStage::kInput;
        outcome.failure_reason = navigation_planning::PlanningFailureReason::kInvalidInput;
        const auto finish = [&]() {
            outcome.trace.elapsed_steady_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - started).count();
            return outcome;
        };
        if (!request.valid()) return finish();

        // The request owns the immutable solve inputs. Route geometry is set
        // by the runtime immediately before submission and is checked by the
        // request key/goal identity; this call only binds the same world and
        // kinematic snapshot to the backend solve.
        setWorldModelView(request.world);
        if (!setState(request.start_state)) return finish();
        setCommandIdentity(CommandIdentity{
            request.key.localization_epoch, request.key.goal_epoch,
            request.key.request_id});
        const auto result = request.key.start_mode ==
                navigation_planning::PlanningStartMode::kStoppedMeasuredState
            ? planInitialFromStoppedState(
                request.goal.target_world, 0.0, true)
            : request.key.start_mode ==
                navigation_planning::PlanningStartMode::kMeasuredEmergencyBrake
            ? ([&]() {
                StatePVAJ measured = StatePVAJ::Zero();
                measured.col(0) = request.start_state.position_world;
                measured.col(1) = request.start_state.velocity_world;
                measured.col(2) = request.start_state.acceleration_world;
                measured.col(3) = request.start_state.jerk_world;
                return commitEmergencyBrake(
                    measured, request.start_state.yaw_rad, 0.0,
                    planner_context_->getSimTime(), std::nullopt)
                    ? RET_CODE::SUCCESS : RET_CODE::EMER;
              })()
            : planSuccessorFromExecutionAnchor(
                request.goal.target_world, 0.0, false);
        const auto status = result == SUCCESS || result == FINISH || result == NO_NEED
            ? navigation_planning::PlanningFailureStage::kNone
            : navigation_planning::PlanningFailureStage::kNominalRefinement;
        if (result == NO_NEED) {
            outcome.outcome = navigation_planning::CompletePlanningOutcome::
                kRetainedCommittedBundle;
            outcome.failure_stage = navigation_planning::PlanningFailureStage::kNone;
            outcome.failure_reason = navigation_planning::PlanningFailureReason::kNone;
            return finish();
        }
        if (result != SUCCESS && result != FINISH) {
            outcome.outcome = result == EMER
                ? navigation_planning::CompletePlanningOutcome::kNoCompleteBundle
                : navigation_planning::CompletePlanningOutcome::kNoCompleteBundle;
            outcome.failure_stage = status;
            outcome.failure_reason = result == EMER
                ? navigation_planning::PlanningFailureReason::kBackupDynamics
                : navigation_planning::PlanningFailureReason::kNoCompleteBundleAtDeadline;
            return finish();
        }
        const auto candidate = exportCommandCandidate(
            request.key.localization_epoch, request.key.goal_epoch,
            request.key.request_id, request.key.anchor_stamp_ns,
            std::numeric_limits<std::int64_t>::max());
        if (!candidate) {
            outcome.outcome = navigation_planning::CompletePlanningOutcome::kNoCompleteBundle;
            outcome.failure_stage = navigation_planning::PlanningFailureStage::kCommitRecertification;
            outcome.failure_reason = navigation_planning::PlanningFailureReason::kNoCompleteBundleAtDeadline;
            return finish();
        }
        outcome.candidate = *candidate;
        outcome.outcome = result == SUCCESS
            ? navigation_planning::CompletePlanningOutcome::kRefinedCompleteBundle
            : navigation_planning::CompletePlanningOutcome::kBaselineCompleteBundle;
        outcome.failure_stage = navigation_planning::PlanningFailureStage::kNone;
        outcome.failure_reason = navigation_planning::PlanningFailureReason::kNone;
        return finish();
    }

    RET_CODE Planner::PlanFromRest(const Vec3f &goal_p,
                                   const double &goal_yaw,
                                   const bool &new_goal) {
        return planInitialFromStoppedState(goal_p, goal_yaw, new_goal);
    }

    RET_CODE Planner::ReplanOnce(const Vec3f &goal_p,
                                 const double &goal_yaw,
                                 const bool &new_goal) {
        return planSuccessorFromExecutionAnchor(goal_p, goal_yaw, new_goal);
    }

    void Planner::getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish) {
        const auto committed = cmd_traj_info_.snapshot();
        double eval_t = (planner_context_->getSimTime() -
                         committed.position.start_WT);
        traj_finish = false;
        const double total_dur = committed.position.getTotalDuration();
        if (eval_t > total_dur) {
            traj_finish = true;
            eval_t = total_dur;
        }
        start_WT_pos = committed.position.start_WT;
    }

    Trajectory Planner::getCommittedPositionTrajectory() {
        return cmd_traj_info_.posTraj();
    }

    Trajectory Planner::getCommittedYawTrajectory() {
        return cmd_traj_info_.yawTraj();
    }


    void Planner::getOneCommandFromTraj(StatePVAJ &pvaj,
                                             double &yaw,
                                             double &yaw_dot,
                                             bool &on_backup_traj,
                                             bool &traj_finish) {
        const auto sample = sampleCommand();
        pvaj = sample.pvaj;
        yaw = sample.yaw;
        yaw_dot = sample.yaw_rate;
        on_backup_traj = sample.role == TrajectoryRole::BACKUP;
        traj_finish = sample.finished;
    }

    Planner::CommandSample Planner::sampleCommand() {
        CommandSample sample;
        const auto committed = cmd_traj_info_.snapshot();
        const double cur_t = planner_context_->getSimTime();
        const double cmd_start_WT = committed.position.start_WT;
        const double total_dur = committed.position.getTotalDuration();

        const auto command_time = commandTrajectoryTime(cur_t, cmd_start_WT, total_dur);
        sample.finished = command_time.finished;
        const double eval_t = command_time.trajectory_time_s;
        sample.trajectory_time_s = eval_t;
        for (const auto& interval : committed.roles) {
            if (interval.role == CandidateTrajectoryRole::BACKUP &&
                eval_t >= interval.begin_tt && eval_t <= interval.end_tt) {
                sample.role = TrajectoryRole::BACKUP;
                break;
            }
        }
        sample.pvaj = committed.position.getState(eval_t);
        sample.yaw = committed.yaw.getPos(eval_t)[0];
        sample.yaw_rate = committed.yaw.getVel(eval_t)[0];
        sample.generation = committed.generation;
        sample.certificate_world = committed.certificate.validated_world;
        sample.valid = sample.pvaj.allFinite() && std::isfinite(sample.yaw) &&
                       std::isfinite(sample.yaw_rate) &&
                       std::isfinite(sample.trajectory_time_s);
        return sample;
    }

    std::optional<navigation_planning::CandidateBundle> Planner::exportCommandCandidate(
            const std::uint64_t localization_epoch,
            const std::uint64_t goal_epoch,
            const std::uint64_t request_id,
            const std::int64_t valid_from_ns,
            const std::int64_t valid_until_ns) const {
        std::optional<StagedCommandCandidate> staged;
        {
            std::lock_guard<std::mutex> guard(solve_commit_mutex_);
            if (!staged_command_candidate_) return std::nullopt;
            staged = staged_command_candidate_;
        }
        return exportStagedCommandCandidate(
            staged->command, staged->certificate, staged->generation,
            localization_epoch, goal_epoch, request_id,
            valid_from_ns, valid_until_ns);
    }

    bool Planner::commitEmergencyBrake(const StatePVAJ &initial_command_state,
                                            const double initial_command_yaw,
                                            const double initial_command_yaw_dot,
                                            const double start_WT,
                                            const std::optional<double>
                                                terminal_altitude_override_m) {
        if (!initial_command_state.allFinite() ||
            !std::isfinite(initial_command_yaw) ||
            !std::isfinite(initial_command_yaw_dot) || !std::isfinite(start_WT)) {
            planner_context_->error(" -- [planner] emergency brake rejected: non-finite initial state");
            return false;
        }

        StatePVAJ bounded_state = initial_command_state;
        // Preserve the complete live command boundary at handover. Do not
        // clamp acceleration or jerk here: saturation would itself create the
        // discontinuity this emergency transition is meant to eliminate. The
        // seed and final flatness certificates reject an infeasible boundary.

        // A measured-state emergency brake is also the last command that can
        // protect the vehicle while a replacement solve is unavailable.  A
        // free-end stop preserves P/V/J continuity but lets an existing
        // downward velocity finish below the current altitude.  Repeating
        // that recovery during a long obstacle detour accumulates altitude
        // loss and can drive the vehicle into the ground even though every
        // individual polynomial is dynamically valid.  The runtime may
        // provide a bounded terminal altitude from the retained certified
        // command; this changes neither the hard dynamic limits nor the
        // world/command certificates.
        const double terminal_altitude_m =
            terminal_altitude_override_m.has_value() &&
                    std::isfinite(*terminal_altitude_override_m)
                ? *terminal_altitude_override_m
                : bounded_state.col(0).z();
        const auto seed = makeBackupBrakingSeedWithTerminalAltitude(
                0.0, bounded_state,
                cfg_.back_traj_cfg.max_vel,
                cfg_.back_traj_cfg.max_acc,
                cfg_.back_traj_cfg.max_jerk,
                cfg_.sample_traj_dt_s,
                0.0, terminal_altitude_m);
        if (!seed.feasible || !seed.terminal_altitude_preserved ||
            !std::isfinite(seed.duration_s) || seed.duration_s <= 0.0) {
            planner_context_->error(
                " -- [planner] emergency brake rejected: no feasible altitude-preserving PVAJ stop seed "
                "altitude={} preserved={}",
                terminal_altitude_m, seed.terminal_altitude_preserved);
            return false;
        }

        // The translational braking seed does not account for the independent
        // yaw-acceleration limit.  A short emergency stop can therefore be
        // translationally feasible but rejected at the final yaw certificate
        // (especially after a waypoint handoff). Extend the same stop
        // polynomial until the existing flatness/yaw limits accept it; this
        // changes duration only and does not relax any safety gate.
        double stop_duration_s = seed.duration_s;
        Trajectory position_trajectory;
        Trajectory yaw_trajectory;
        traj_opt::TrajectoryDynamicReport dynamic_report;
        bool dynamic_certificate_valid = false;
        for (int attempt = 0; attempt < 24; ++attempt) {
            position_trajectory = Trajectory{};
            position_trajectory.emplace_back(
                    minimumSnapStopPieceWithTerminalAltitude(
                        bounded_state, stop_duration_s, terminal_altitude_m));
            position_trajectory.start_WT = start_WT;

            StatePVAJ yaw_state = StatePVAJ::Zero();
            yaw_state(0, 0) = initial_command_yaw;
            yaw_state(0, 1) = initial_command_yaw_dot;
            yaw_trajectory = Trajectory{};
            yaw_trajectory.emplace_back(
                    minimumSnapStopPiece(yaw_state, stop_duration_s));
            yaw_trajectory.start_WT = start_WT;

            const double emergency_yaw_rate = yaw_trajectory.getMaxVelRate();
            const double emergency_yaw_acceleration =
                yaw_trajectory.getMaxAccRate();
            if (std::isfinite(emergency_yaw_rate) &&
                emergency_yaw_rate <= cfg_.yaw_rate_max_rad_s + 1.0e-6 &&
                std::isfinite(emergency_yaw_acceleration) &&
                emergency_yaw_acceleration <=
                    cfg_.yaw_acceleration_max_rad_s2 + 1.0e-6 &&
                traj_opt::trajectorySatisfiesFlatnessEnvelope(
                    position_trajectory, cfg_.back_traj_cfg, &dynamic_report,
                    0.005, &yaw_trajectory)) {
                dynamic_certificate_valid = true;
                break;
            }
            stop_duration_s *= 1.15;
            if (!std::isfinite(stop_duration_s) || stop_duration_s <= 0.0) {
                break;
            }
        }
        if (!dynamic_certificate_valid) {
            planner_context_->error(
                    " -- [planner] emergency brake dynamic gate failed: finite={} "
                    "first_nonfinite_time={} nonfinite_mask={} body_rate={} thrust=[{},{}] "
                    "initial_v={} initial_a={} initial_j={} yaw={} yaw_rate={}",
                    dynamic_report.finite,
                    dynamic_report.first_nonfinite_time_s,
                    dynamic_report.nonfinite_mask,
                    dynamic_report.maximum_body_rate_rad_s,
                    dynamic_report.minimum_thrust_n,
                    dynamic_report.maximum_thrust_n,
                    bounded_state.col(1).transpose(),
                    bounded_state.col(2).transpose(),
                    bounded_state.col(3).transpose(),
                    initial_command_yaw, initial_command_yaw_dot);
            return false;
        }

        auto candidate = CmdTraj::buildEmergencyCandidate(
            position_trajectory, yaw_trajectory);
        if (!candidate || !authorizeAndStage(std::move(*candidate))) {
            planner_context_->error(" -- [planner] emergency brake atomic commit rejected");
            return false;
        }
        planner_context_->warn(
                " -- [planner] committed command-continuous emergency brake: "
                "initial_speed={} speed_limit={} initial_overspeed={} duration={} distance={} "
                "peak_vel={} peak_acc={} peak_jerk={}",
                seed.initial_velocity_mps, cfg_.back_traj_cfg.max_vel,
                seed.initial_overspeed,
                stop_duration_s,
                (seed.endpoint - bounded_state.col(0)).norm(),
                seed.maximum_velocity_mps, seed.maximum_acceleration_mps2,
                seed.maximum_jerk_mps3);
        return true;
    }


    void Planner::getModuleTimeConsuming(vector<double> &time) {
        time = time_consuming_;
        std::fill(time_consuming_.begin(), time_consuming_.end(), 0);
    }

    vector<double> Planner::moduleTimeConsumingSnapshot() const {
        std::lock_guard<std::mutex> guard(replan_lock_);
        return time_consuming_;
    }


    RET_CODE Planner::generateExpTraj(
            ExpTraj &last_exp_traj_info, ExpTraj &out_exp_traj_info,
            const AbsoluteDeadline &solve_deadline) {
        /* 1) Log the exp traj frontend time*/
        TimeConsuming t_exp_frontend("t_exp_frontend", false);

        // use hot init or not, just prepare a guide path, a guide t, init and fina state and sfc for exp traj opt
        StatePVAJ pos_init_state, pos_fina_state;
        PolytopeVec sfc;
        vec_Vec3f guide_path;
        // the guide_stamp saves a TT
        vector<double> guide_stamp;
        double guide_path_end_vel{0.0};
        // Preserve the mission-resolved endpoint before receding-horizon
        // construction is allowed to replace gi_.goal_p with a bounded local
        // prefix.  Local endpoint connectivity is useful for geometry, but it
        // must not activate terminal STOP semantics for a remote mission goal.
        const Vec3f mission_planning_goal = gi_.goal_p;
        int reserve_size = cfg_.local_window_m / cfg_.resolution * 1.2;
        guide_path.reserve(reserve_size);
        guide_stamp.reserve(reserve_size);
        latest_guide_path_length_m_ = std::numeric_limits<double>::quiet_NaN();
        latest_guide_duration_s_ = std::numeric_limits<double>::quiet_NaN();
        required_lookahead_m_ = std::numeric_limits<double>::quiet_NaN();
        certified_lookahead_m_ = std::numeric_limits<double>::quiet_NaN();
        lookahead_complete_ = false;

        Vec4f init_yaw{solve_state_.yaw, 0, 0, 0};
        // alias for last_exp_traj_info
        Trajectory guide_pos_traj, guide_yaw_traj, last_exp_traj;

        // record the wall time (WT) and the trajectory time (TT) at the start of the replan.
        const double replan_process_start_WT = planner_context_->getSimTime();
        double replan_process_start_TT, replan_state_TT;
        HotReplanWindow replan_window;
        // A hot replan normally preserves a short prefix of the currently
        // committed command so PVAJ remains continuous.  That prefix is not
        // authoritative when the vehicle has fallen behind it.  A measured
        // P/V rebase with estimated A/J reset to zero is not a continuous hot
        // splice either, so this solve must not publish such a replacement.
        // Retain the previous immutable command while its normal runtime
        // certificates remain valid; the bounded anchor-recovery supervisor
        // owns any explicit measured-state transition.
        if (!last_exp_traj_info.empty()) {
            const auto committed = cmd_traj_info_.snapshot();
            const double committed_start_WT = committed.position.start_WT;
            const double committed_duration = committed.position.getTotalDuration();
            const double committed_tt = std::clamp(
                    replan_process_start_WT - committed_start_WT,
                    0.0, std::max(0.0, committed_duration));
            const StatePVAJ committed_state = committed.position.getState(committed_tt);
            const double committed_future_tt = std::clamp(
                    committed_tt + cfg_.replan_forward_dt_s,
                    committed_tt, std::max(committed_tt, committed_duration));
            const StatePVAJ committed_future_state =
                    committed.position.getState(committed_future_tt);
            StatePVAJ committed_yaw_state;
            const bool committed_yaw_state_valid =
                    committed.yaw.getState(committed_tt, committed_yaw_state);
            const double command_anchor_error =
                    (committed_state.col(0) - solve_state_.p).norm();
            const double splice_horizon_s = committed_future_tt - committed_tt;
            const Vec3f measured_constant_velocity_future =
                    solve_state_.p + splice_horizon_s * solve_state_.v;
            const double future_position_error =
                    (committed_future_state.col(0) -
                     measured_constant_velocity_future).norm();
            const double future_velocity_error =
                    (committed_future_state.col(1) - solve_state_.v).norm();
            const auto splice_compatibility = assessHotReplanSpliceCompatibility(
                    command_anchor_error, future_position_error,
                    future_velocity_error, splice_horizon_s,
                    cfg_.exp_traj_cfg.max_acc, cfg_.tracking_error_budget_m);
            const bool measured_yaw_valid = std::isfinite(solve_state_.yaw);
            const bool committed_yaw_valid = committed_yaw_state_valid &&
                    committed_yaw_state.allFinite() &&
                    std::isfinite(committed_yaw_state(0, 0));
            const double yaw_anchor_error = measured_yaw_valid && committed_yaw_valid
                    ? std::abs(std::remainder(
                          solve_state_.yaw - committed_yaw_state(0, 0), 2.0 * M_PI))
                    : std::numeric_limits<double>::infinity();
            const bool position_rebase_required =
                    !committed_state.allFinite() ||
                    !committed_future_state.allFinite() ||
                    splice_compatibility.requiresMeasuredStateRestart();
            const bool yaw_rebase_required =
                    !measured_yaw_valid || !committed_yaw_valid ||
                    !std::isfinite(yaw_anchor_error) ||
                    yaw_anchor_error > cfg_.yaw_tracking_error_budget_rad;
            const bool tracking_budget_exceeded =
                    position_rebase_required || yaw_rebase_required;
            if (tracking_budget_exceeded) {
                const bool measured_start_traversable =
                        solve_state_.p.allFinite() && map_ptr_->contains(solve_state_.p) &&
                        navigation_world_model::isCellTraversable(
                                map_ptr_->classify(
                                        solve_state_.p,
                                        navigation_world_model::GridLayer::kInflated),
                                unknownPolicy());
                const auto recovery = classifyHotReplanTrackingRecovery(
                        tracking_budget_exceeded, measured_start_traversable);
                if (recovery == HotReplanTrackingRecovery::kFailClosed) {
                    planner_context_->warn(
                            " -- [planner] cannot restart hot replan from measured state: "
                            "measured start is not traversable position_error={} "
                            "future_position_error={} future_velocity_error={} yaw_error={} "
                            "position_budget={} future_position_allowance={} "
                            "future_velocity_allowance={} yaw_budget={}",
                            command_anchor_error, future_position_error,
                            future_velocity_error, yaw_anchor_error,
                            cfg_.tracking_error_budget_m,
                            splice_compatibility.future_position_allowance_m,
                            splice_compatibility.future_velocity_allowance_mps,
                            cfg_.yaw_tracking_error_budget_rad);
                    return FAILED;
                }
                planner_context_->warn(
                        " -- [planner] hot splice outside compatibility envelope; "
                        "retaining certified command for bounded runtime recovery: "
                        "position_error={} future_position_error={} "
                        "future_velocity_error={} yaw_error={} position_budget={} "
                        "future_position_allowance={} future_velocity_allowance={} "
                        "yaw_budget={}",
                        command_anchor_error, future_position_error,
                        future_velocity_error, yaw_anchor_error,
                        cfg_.tracking_error_budget_m,
                        splice_compatibility.future_position_allowance_m,
                        splice_compatibility.future_velocity_allowance_mps,
                        cfg_.yaw_tracking_error_budget_rad);
                return FAILED;
            }
        }

        /* 2) Check last exp traj */
        if (last_exp_traj_info.empty()) {
            /* 2.1) Generate from the latest measured state. */
            // A route reset must not silently become a kinematic reset. The
            // execution state may still carry motion when a local trajectory
            // ends at a sensing frontier; preserve measured P/V and any
            // genuinely measured A/J, while the boundary helper excludes
            // finite-difference estimates from the immutable command state.
            // Only position is shifted to the collision-free map start.
            pos_init_state = makeCommandBoundaryPVAJ(
                solve_state_, solve_acceleration_estimated_, solve_jerk_estimated_);
            pos_init_state.col(0) = local_start_p_;
            replan_process_start_TT = -1;
            replan_state_TT = -1;
        } else {
            guide_pos_traj = cmd_traj_info_.posTraj(); // last_exp_traj;
            guide_yaw_traj = cmd_traj_info_.yawTraj(); //last_exp_traj_info.exp_yaw_traj;
            last_exp_traj = last_exp_traj_info.posTraj();

            replan_window = hotReplanWindow(
                    replan_process_start_WT, guide_pos_traj.start_WT,
                    cfg_.replan_forward_dt_s, cmd_traj_info_.getTotalDuration());
            if (!replan_window.valid) {
                planner_context_->error(
                        " -- [generateExpTraj] invalid executable-command clock for hot replan");
                return FAILED;
            }
            replan_process_start_TT = replan_window.start_tt_s;
            replan_state_TT = replan_window.state_tt_s;
            /* 2.2) Perform collision check on last exp traj*/
            vector<TimePosPair> last_exp_traj_time_pos;
            vector<double> last_exp_traj_vel;


            // Check the executable command boundary, not the planner-history
            // boundary. If the command has actually ended while still on its
            // MAIN portion, ask the runtime to restart from the measured
            // state; retaining an ended command would only let its finite
            // execution lease expire.
            if (replan_window.reaches_command_end) {
                out_exp_traj_info = last_exp_traj_info;

                if (cmd_traj_info_.isTTOnBackupTraj(replan_process_start_TT)) {
                    if (cfg_.print_log)
                        planner_context_->warn(
                                " -- [planner] Replan emergency stop; return FAILED and wait for a rest-state plan.");
                    return FAILED;
                }

                if (cfg_.print_log) {
                    planner_context_->warn(
                            " -- [generateExpTraj] committed command ended before hot replan; "
                            "requesting PlanFromRest.");
                }
                return NEW_TRAJ;
            }

            if (!last_exp_traj_info.empty()) {
                // last_exp_traj_info is only the previous EXP planning
                // history; it deliberately does not include the committed
                // backup suffix.  Its endpoint can therefore be earlier than
                // cmd_traj_info_'s executable endpoint.  Treating that
                // historical endpoint as a completed command returns NO_NEED
                // while a valid committed bundle is still running, preventing
                // renewal of its finite command lease and causing a false
                // execution-boundary expiry.  The command trajectory check
                // above is the sole completion boundary for hot replanning.

                // Do not return NO_NEED merely because the previous EXP was
                // connected to the goal or geometrically near it.  The
                // committed command may still have a backup suffix and its
                // caller-owned validity window must be renewed by a fresh,
                // latest-world-certified candidate.  The mission controller
                // observes completion from the committed endpoint and can
                // then advance the waypoint.
            }
            /// Ready for replan.
            out_exp_traj_info.setGoalConnectedFlag(false);

            // The guide is always sampled from the committed command. This is
            // required while a BACKUP suffix is active because planner history
            // may end before the executable command; the command is the only
            // valid continuity boundary for this replan.

            // Sample the committed guide after the measured replan state and
            // retain only its known-traversable continuity prefix.
            double eval_t = replan_state_TT;
            double guide_pos_traj_total_time = guide_pos_traj.getTotalDuration();

            Vec3f temp_pt, last_sample_pt;
            last_exp_traj_time_pos.clear();
            last_sample_pt = guide_pos_traj.getPos(eval_t);
            eval_t += cfg_.sample_traj_dt_s;
            for (; eval_t < guide_pos_traj_total_time; eval_t += cfg_.sample_traj_dt_s) {
                temp_pt = guide_pos_traj.getPos(eval_t);
                if ((temp_pt - last_sample_pt).norm() < cfg_.resolution * 0.8) {
                    continue;
                }

                const auto temp_grid = map_ptr_->classify(
                        temp_pt, navigation_world_model::GridLayer::kInflated);

                if (!navigation_world_model::isCellTraversable(temp_grid, unknownPolicy())) {
                    break;
                }
                last_exp_traj_time_pos.emplace_back(eval_t, temp_pt);
                last_exp_traj_vel.emplace_back(guide_pos_traj.getVel(eval_t).norm());
                last_sample_pt = temp_pt;
            }


            // Keep only the configured known-free continuity prefix; A* owns
            // the remaining route on every planning cycle, including when the
            // sampled command is entirely free.
            const double split_dis = cfg_.receding_distance_m;
            // Do not turn a collision-free committed trajectory into an
            // infinite immutable guide.  That upstream shortcut recursively
            // feeds optimizer drift back into every later replan and lets the
            // displayed normal path advance independently of newly sensed
            // geometry. Keep only the configured continuity prefix; A* owns
            // the rest of the route on every planning cycle.


            // Begin the replan from the committed trajectory's measured-state
            // boundary.
            if (!guide_pos_traj.getState(replan_state_TT, pos_init_state)) {
                planner_context_->warn(" -- [planner] Invalid traj or eval t");
                return FAILED;
            }
            // Build the hot-start guide path. Stamps are relative to its first
            // retained point.
            guide_stamp.clear();
            guide_path.clear();
            if (split_dis <= 0 || last_exp_traj_time_pos.empty()) {
                /// No need receding, just path search.
                guide_path.push_back(pos_init_state.col(0));
                guide_stamp.push_back(0.0);
                last_exp_traj_time_pos.clear();
                last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                guide_path_end_vel = solve_state_.v.norm();
            } else {
                temp_pt = last_exp_traj_time_pos.back().second;
                // Drop sampled points beyond the configured continuity prefix
                // or points that are no longer traversable.
                while (!navigation_world_model::isCellTraversable(
                           map_ptr_->classify(temp_pt,
                                              navigation_world_model::GridLayer::kInflated),
                           unknownPolicy()) ||
                       (temp_pt - pos_init_state.col(0)).norm() > split_dis) {
                    last_exp_traj_time_pos.pop_back();
                    last_exp_traj_vel.pop_back();
                    if (last_exp_traj_time_pos.empty()) {
                        planner_context_->warn(" -- [planner] WARN, all traj is collide in INF2");
                        break;
                    }
                    temp_pt = last_exp_traj_time_pos.back().second;
                }
                if (!last_exp_traj_time_pos.empty()) {
                    for (long unsigned int i = 0; i < last_exp_traj_time_pos.size(); i++) {
                        guide_path.push_back(last_exp_traj_time_pos[i].second);
                        guide_stamp.push_back(last_exp_traj_time_pos[i].first - last_exp_traj_time_pos.front().first);
                        guide_path_end_vel = last_exp_traj_vel[i];
                    }
                } else {
                    guide_path.push_back(pos_init_state.col(0));
                    guide_stamp.push_back(0.0);
                    last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                    guide_path_end_vel = solve_state_.v.norm();
                }
            }
        }

        // second, geometry part of the guide path
        ///=================The Second Part of Guide Path ================================================

        double guide_path_length = geometry_utils::computePathLength(guide_path);
        double temp_horizon = cfg_.local_window_m - guide_path_length;

        vector<int> path_passed_waypoint_id;
        vec_Vec3f inside_poly_goals;
        vector<int> sfc_waypoint_ids;

        if (guide_path.empty() ||
            ((guide_path.front() - pos_init_state.col(0)).norm() > 1e-2)) {
            guide_path.insert(guide_path.begin(), pos_init_state.col(0));
            guide_stamp.insert(guide_stamp.begin(), 0.0);
        }

        // if need a geometry path
        if (temp_horizon > cfg_.resolution * 2) {
            /// start point TT + exp_traj start_WT
//            double path_search_start_point_WT = guide_stamp.back() + guide_pos_traj.start_WT;
            // if the goal is close to the last point of the guide path, just add the goal to the guide path
            if ((guide_path.back() - gi_.goal_p).norm() <=
                    navigation_world_model::kNearGoalShortcutToleranceM &&
                navigation_world_model::isGoalSegmentTraversable(
                    *map_ptr_, guide_path.back().cast<double>(), gi_.goal_p.cast<double>())) {
                guide_stamp.push_back(guide_stamp.back() +
                                      (guide_path.back() - gi_.goal_p).norm() / cfg_.exp_traj_cfg.max_vel);
                guide_path.push_back(gi_.goal_p);
                // NO NEED
            } else {
                vec_Vec3f new_path;
                solve_stage_.store(2);
                const double local_search_horizon =
                    geometry_utils::localRouteSearchHorizon(
                        temp_horizon, cfg_.visibility_horizon_m);
                Vec3f local_search_goal = gi_.goal_p;
                RouteBackboneTarget route_backbone_target;
                if (route_snapshot_.has_value()) {
                    route_backbone_target = selectRouteBackboneTarget(
                        *route_snapshot_, guide_path.back().cast<double>(),
                        local_search_horizon);
                    if (route_backbone_target.valid) {
                        local_search_goal = route_backbone_target.point;
                    }
                }
                if (!std::isfinite(local_search_horizon) ||
                    !PathSearch(guide_path.back(), local_search_goal,
                                local_search_horizon,
                                new_path, solve_deadline,
                                route_backbone_target.valid)) {
                    planner_context_->warn(" -- [planner] PathSearch for new path failed");
                    return FAILED;
                }
                if (new_path.size() < 2) {
                    planner_context_->warn(" -- [planner] PathSearch for new path failed");
                    return FAILED;
                }

                // Keep remote mission progress as a controller-owned contract,
                // but give MINCO only a bounded certified prefix. A mission
                // waypoint can be farther away than the current visibility
                // horizon after a measured pass-through handoff. Solving that
                // entire route in one polynomial couples a remote endpoint to
                // finite local evidence and can repeatedly fail at the first
                // long outgoing leg. The next cycle receives the same mission
                // goal and advances this prefix from fresh measured state.
                const double route_distance =
                    (gi_.goal_p - guide_path.back()).norm();
                const double local_prefix_limit = local_search_horizon;
                if (std::isfinite(route_distance) &&
                    std::isfinite(local_prefix_limit) &&
                    local_prefix_limit > cfg_.resolution * 2.0 &&
                    route_distance > local_prefix_limit + cfg_.resolution) {
                    vec_Vec3f bounded_path;
                    bool prefix_truncated = false;
                    if (!geometry_utils::truncatePathAtDistance(
                            new_path, local_prefix_limit, bounded_path,
                            prefix_truncated) || bounded_path.size() < 2U ||
                        geometry_utils::computePathLength(bounded_path) >
                            local_prefix_limit + 1.0e-6) {
                        planner_context_->warn(
                            " -- [planner] unable to construct certified local route prefix: "
                            "route_distance={} prefix_limit={} source_length={} truncated={}",
                            route_distance, local_prefix_limit,
                            geometry_utils::computePathLength(new_path),
                            prefix_truncated);
                        return FAILED;
                    }
                    bool prefix_certified = true;
                    for (std::size_t index = 1U;
                         index < bounded_path.size(); ++index) {
                        if (!map_ptr_->isSegmentTraversable(
                                bounded_path[index - 1U].cast<double>(),
                                bounded_path[index].cast<double>(),
                                navigation_world_model::GridLayer::kInflated,
                                unknownPolicy())) {
                            prefix_certified = false;
                            break;
                        }
                    }
                    if (!prefix_certified) {
                        planner_context_->warn(
                            " -- [planner] bounded local route prefix failed inflated-map certification: "
                            "route_distance={} prefix_limit={}",
                            route_distance, local_prefix_limit);
                        return FAILED;
                    }
                    new_path = std::move(bounded_path);
                    gi_.goal_p = new_path.back();
                    planning_goal_p_ = gi_.goal_p;
                    goal_endpoint_adjusted_ = true;
                    planner_context_->info(
                        " -- [planner] bounded remote goal to certified route prefix: "
                        "route_distance={} prefix_limit={} prefix_length={} "
                        "mission_goal=({}, {}, {}) prefix_goal=({}, {}, {}) "
                        "route_backbone={} route_backbone_arc={}",
                        route_distance, local_prefix_limit,
                        geometry_utils::computePathLength(new_path),
                        requested_goal_p_.x(), requested_goal_p_.y(),
                        requested_goal_p_.z(), gi_.goal_p.x(), gi_.goal_p.y(),
                        gi_.goal_p.z(), route_backbone_target.valid,
                        route_backbone_target.valid
                            ? route_backbone_target.target_arc_m
                            : std::numeric_limits<double>::quiet_NaN());
                }

                const bool terminal_stop_required =
                    !pass_through_next_target_.has_value() &&
                    (new_path.back() - requested_goal_p_).norm() <=
                        navigation_world_model::kGoalConnectionToleranceM + 1.0e-6;
                geometry_utils::GuideTimeAllocation allocation;
                const bool allocation_valid = terminal_stop_required
                    ? geometry_utils::allocateGuideElapsedTimes(
                          cfg_.exp_traj_cfg.max_acc,
                          cfg_.exp_traj_cfg.max_vel,
                          guide_path_end_vel,
                          guide_path.back(), new_path, allocation)
                    : geometry_utils::allocateGuideContinuationElapsedTimes(
                          cfg_.exp_traj_cfg.max_acc,
                          cfg_.exp_traj_cfg.max_jerk,
                          cfg_.exp_traj_cfg.max_vel,
                          guide_path_end_vel,
                          guide_path.back(), new_path, allocation);
                if (!allocation_valid) {
                    planner_context_->warn(
                            " -- [planner] invalid A* guide time allocation: "
                            "mode={} path_points={} initial_velocity_mps={} "
                            "max_acceleration_mps2={} max_jerk_mps3={} "
                            "max_velocity_mps={}",
                            terminal_stop_required ? "terminal-stop" : "continuation",
                            new_path.size(), guide_path_end_vel,
                            cfg_.exp_traj_cfg.max_acc,
                            cfg_.exp_traj_cfg.max_jerk,
                            cfg_.exp_traj_cfg.max_vel);
                    return FAILED;
                }
                const double guide_time_origin_s = guide_stamp.back();
                for (std::size_t i = 0; i < allocation.points.size(); ++i) {
                    guide_path.emplace_back(allocation.points[i]);
                    guide_stamp.emplace_back(
                            guide_time_origin_s + allocation.elapsed_s[i]);
                }
                guide_path_end_vel = allocation.terminal_velocity_mps;
            }
        }

        // A pass-through waypoint is a measured mission boundary, not the end
        // of the executable route. Extend the guide through the next route
        // segment when the current solve has enough certified map horizon.
        // This gives MINCO geometric room to turn or continue and lets
        // MissionController advance the checkpoint while the same command is live.
        bool route_lookahead_active = false;
        bool route_lookahead_is_corner = false;
        double route_terminal_speed_cap_mps =
            cfg_.exp_traj_cfg.max_vel * cfg_.exp_traj_cfg.optimization_dynamic_reserve_ratio;
        std::optional<CorridorGenerator::RouteBoundaryGate> route_boundary_gate;
        const auto mission_incoming_tangent = [&]()
                -> std::optional<Eigen::Vector3d> {
            if (!route_snapshot_.has_value() ||
                route_snapshot_->active_waypoint_index == 0U) {
                return std::nullopt;
            }
            const auto segment = std::find_if(
                route_snapshot_->segments.begin(), route_snapshot_->segments.end(),
                [this](const navigation_mission::RouteSegment& candidate) {
                    return candidate.end_waypoint_index ==
                        route_snapshot_->active_waypoint_index;
                });
            if (segment == route_snapshot_->segments.end() ||
                !segment->tangent.allFinite() || segment->tangent.norm() <= 1.0e-6) {
                return std::nullopt;
            }
            return segment->tangent;
        };
        if (gi_.new_goal && pass_through_next_target_.has_value()) {
            planner_context_->info(
                " -- [planner] pass-through lookahead input active goal=({}, {}, {}) "
                "next=({}, {}, {}) guide_end=({}, {}, {}) guide_error={}",
                gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z(),
                pass_through_next_target_->x(), pass_through_next_target_->y(),
                pass_through_next_target_->z(), guide_path.back().x(),
                guide_path.back().y(), guide_path.back().z(),
                (guide_path.back() - gi_.goal_p).norm());
        }
        if (pass_through_next_target_.has_value() && guide_path.size() >= 2U &&
            guide_stamp.size() == guide_path.size() &&
            (guide_path.back() - gi_.goal_p).norm() <=
                navigation_world_model::kGoalConnectionToleranceM + 1.0e-6 &&
            passThroughGuideReachesMissionBoundary(
                guide_path.back().cast<double>(), requested_goal_p_.cast<double>(),
                navigation_world_model::kGoalConnectionToleranceM)) {
            const Eigen::Vector3d current_endpoint = guide_path.back().cast<double>();
            const Eigen::Vector3d next_target = *pass_through_next_target_;
            const double outgoing_distance = (next_target - current_endpoint).norm();
            const double guide_length = geometry_utils::computePathLength(guide_path);
            const double remaining_horizon = cfg_.local_window_m - guide_length;
            const double known_free_to_boundary_m = knownFreeGuideSupport(
                *map_ptr_, guide_path);
            const bool boundary_known_free =
                std::isfinite(known_free_to_boundary_m) &&
                std::isfinite(guide_length) &&
                known_free_to_boundary_m + cfg_.resolution >= guide_length;
            const bool outgoing_known_free = map_ptr_->isSegmentTraversable(
                current_endpoint, next_target,
                navigation_world_model::GridLayer::kInflated,
                navigation_world_model::UnknownPolicy::kRequireKnownFree);
            const bool pass_through_visibility_ready = boundary_known_free &&
                outgoing_known_free;
            if (!pass_through_visibility_ready) {
                planner_context_->info(
                    " -- [planner] defer pass-through lookahead: KNOWN_FREE "
                    "support_to_boundary={:.3f}/{:.3f} outgoing_known_free={}",
                    known_free_to_boundary_m, guide_length, outgoing_known_free);
            }
            Eigen::Vector3d incoming_tangent = Eigen::Vector3d::Zero();
            for (std::size_t index = guide_path.size(); index > 1U; --index) {
                const Eigen::Vector3d delta =
                    (guide_path[index - 1U] - guide_path[index - 2U]).cast<double>();
                if (delta.norm() > 1.0e-6) {
                    incoming_tangent = delta;
                    break;
                }
            }
            const bool genuine_corner = passThroughGenuineCorner(
                current_endpoint, next_target, incoming_tangent);
            // Obstacle detours can change the final A* edge enough to make a
            // true mission corner look almost collinear.  Use the immutable
            // route tangent as a second, geometry-owned witness; otherwise a
            // planner-local detour accidentally selects a long outgoing
            // MINCO lookahead and later fails its waypoint-boundary witness.
            const auto mission_tangent = mission_incoming_tangent();
            const bool mission_geometry_corner = mission_tangent.has_value() &&
                passThroughGenuineCorner(
                    current_endpoint, next_target, *mission_tangent);
            const bool effective_genuine_corner = genuine_corner ||
                mission_geometry_corner;
            const double required_lookahead_envelope =
                passThroughCruiseLookaheadDistance(
                    cfg_.exp_traj_cfg.max_vel,
                    cfg_.exp_traj_cfg.max_acc, cfg_.exp_traj_cfg.max_jerk,
                    cfg_.replan_forward_dt_s, cfg_.receding_distance_m);
            const double desired_lookahead = std::isfinite(
                    required_lookahead_envelope)
                ? std::min(outgoing_distance, required_lookahead_envelope)
                : 0.0;
            required_lookahead_m_ = desired_lookahead;
            // The A* query must be allowed to reach the next mission target;
            // the shorter desired_lookahead is applied only when selecting the
            // certified prefix from that returned route.
            std::optional<PassThroughRouteWindow> corner_window;
            double search_distance = remaining_horizon;
            Eigen::Vector3d outgoing_search_start = current_endpoint;
            if (effective_genuine_corner && guide_path.size() >= 2U) {
                auto candidate_window = passThroughRouteWindow(
                    current_endpoint, next_target, incoming_tangent,
                    goal_acceptance_radius_m_,
                    navigation_world_model::kGoalConnectionToleranceM);
                if (!candidate_window.has_value() && mission_tangent.has_value()) {
                    candidate_window = passThroughRouteWindow(
                        current_endpoint, next_target, *mission_tangent,
                        goal_acceptance_radius_m_,
                        navigation_world_model::kGoalConnectionToleranceM);
                }
                const Eigen::Vector3d predecessor =
                    guide_path[guide_path.size() - 2U].cast<double>();
                const auto point_is_traversable = [this](const Eigen::Vector3d& point) {
                    return point.allFinite() && map_ptr_->contains(point) &&
                        navigation_world_model::isCellTraversable(
                            map_ptr_->classify(
                                point, navigation_world_model::GridLayer::kInflated),
                            unknownPolicy());
                };
                const auto segment_is_traversable = [this](
                        const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
                    return map_ptr_->isSegmentTraversable(
                        start, end, navigation_world_model::GridLayer::kInflated,
                        unknownPolicy());
                };
                const bool window_certified = candidate_window.has_value() &&
                    point_is_traversable(candidate_window->entry) &&
                    point_is_traversable(candidate_window->outgoing_blend) &&
                    point_is_traversable(candidate_window->endpoint) &&
                    segment_is_traversable(predecessor, candidate_window->entry) &&
                    segment_is_traversable(candidate_window->entry,
                                           candidate_window->outgoing_blend) &&
                    segment_is_traversable(candidate_window->outgoing_blend,
                                           current_endpoint) &&
                    segment_is_traversable(current_endpoint,
                                           candidate_window->endpoint);
                if (window_certified) {
                    const double current_leg_length =
                        (current_endpoint - predecessor).norm();
                    const double window_length =
                        (candidate_window->entry - predecessor).norm() +
                        (candidate_window->outgoing_blend -
                         candidate_window->entry).norm() +
                        (current_endpoint - candidate_window->outgoing_blend).norm() +
                        (candidate_window->endpoint - current_endpoint).norm();
                    const double guide_length_with_window = guide_length -
                        current_leg_length + window_length;
                    search_distance = cfg_.local_window_m - guide_length_with_window;
                    outgoing_search_start = candidate_window->endpoint;
                    if (search_distance > 2.0 * cfg_.resolution) {
                        corner_window = candidate_window;
                    }
                }
            }
            // Every pass-through boundary needs enough certified outgoing
            // route to remain executable across the measured handoff. A
            // genuine corner is allowed to fillet inside its mission-owned
            // acceptance region; pinning a C3 trajectory to the exact corner
            // would require either zero velocity or an instantaneous tangent
            // change between the incoming and outgoing corridors.
            // A sharp turn can be geometrically reachable but still be an
            // invalid single MINCO problem when the outgoing route initially
            // folds back toward the vehicle. Keep that turn in the bounded
            // acceptance-ball fillet below; the next measured waypoint
            // handoff owns the outgoing leg. This avoids repeatedly
            // optimizing a U-shaped guide under one set of fixed boundary
            // derivatives while preserving every collision and dynamic gate.
            const bool corner_window_ready = !effective_genuine_corner ||
                corner_window.has_value();
            if (pass_through_visibility_ready && corner_window_ready &&
                passThroughOutgoingLookaheadEligible(
                desired_lookahead, outgoing_distance, search_distance,
                cfg_.resolution)) {
                vec_Vec3f next_path;
                solve_stage_.store(2);
                const Vec3f outgoing_start = outgoing_search_start;
                if (PathSearch(outgoing_start, next_target,
                               search_distance,
                               next_path, solve_deadline, true) && next_path.size() >= 2U) {
                    vec_Vec3f lookahead_points;
                    double accumulated_distance = 0.0;
                    Vec3f previous_point = outgoing_start;
                    for (const auto& point : next_path) {
                        const double segment_length =
                            (point - previous_point).norm();
                        if (!std::isfinite(segment_length) || segment_length <= 1.0e-6) {
                            previous_point = point;
                            continue;
                        }
                        if (accumulated_distance + segment_length <=
                            desired_lookahead + 1.0e-6) {
                            lookahead_points.emplace_back(point);
                            accumulated_distance += segment_length;
                            previous_point = point;
                            continue;
                        }
                        const double remaining_distance =
                            desired_lookahead - accumulated_distance;
                        if (remaining_distance > 1.0e-6) {
                            const double fraction = std::clamp(
                                remaining_distance / segment_length, 0.0, 1.0);
                            lookahead_points.emplace_back(
                                previous_point + static_cast<float>(fraction) *
                                    (point - previous_point));
                            accumulated_distance = desired_lookahead;
                        }
                        break;
                    }
                    if (!lookahead_points.empty()) {
                        geometry_utils::GuideTimeAllocation allocation;
                        if (geometry_utils::allocateGuideElapsedTimes(
                                cfg_.exp_traj_cfg.max_acc,
                                cfg_.exp_traj_cfg.max_vel,
                                corner_window.has_value()
                                    ? std::min(
                                          guide_path_end_vel,
                                          passThroughCornerSpeedCap(
                                              goal_acceptance_radius_m_,
                                              cfg_.exp_traj_cfg.max_acc,
                                              route_terminal_speed_cap_mps))
                                    : guide_path_end_vel,
                                outgoing_search_start,
                                lookahead_points, allocation)) {
                            bool certified = true;
                            Eigen::Vector3d segment_start = outgoing_search_start;
                            for (const auto& point : allocation.points) {
                                const Eigen::Vector3d segment_end = point.cast<double>();
                                if (!map_ptr_->isSegmentTraversable(
                                        segment_start, segment_end,
                                        navigation_world_model::GridLayer::kInflated,
                                        unknownPolicy())) {
                                    certified = false;
                                    break;
                                }
                                segment_start = segment_end;
                            }
                            if (certified) {
                                const bool complete = passThroughLookaheadComplete(
                                    desired_lookahead, allocation.path_length_m);
                                const double allocation_duration_s =
                                    allocation.elapsed_s.empty()
                                        ? std::numeric_limits<double>::quiet_NaN()
                                        : allocation.elapsed_s.back();
                                if (!complete && std::isfinite(allocation_duration_s) &&
                                    allocation_duration_s > 1.0e-6) {
                                    const double certified_speed_cap =
                                        terminalSpeedCapForPath(
                                            allocation.path_length_m,
                                            allocation_duration_s,
                                            std::max(0.0, guide_path_end_vel),
                                            route_terminal_speed_cap_mps);
                                    if (std::isfinite(certified_speed_cap)) {
                                        route_terminal_speed_cap_mps = std::min(
                                            route_terminal_speed_cap_mps,
                                            certified_speed_cap);
                                    }
                                }
                                if (corner_window.has_value()) {
                                    const double predecessor_stamp =
                                        guide_stamp.size() >= 2U
                                            ? guide_stamp[guide_stamp.size() - 2U]
                                            : guide_stamp.back();
                                    const auto segment_duration = [this](
                                            const Eigen::Vector3d& start,
                                            const Eigen::Vector3d& end) {
                                        return (end - start).norm() /
                                            cfg_.exp_traj_cfg.max_vel;
                                    };
                                    guide_path.back() = corner_window->entry;
                                    guide_stamp.back() = predecessor_stamp +
                                        segment_duration(
                                            guide_path[guide_path.size() - 2U].cast<double>(),
                                            corner_window->entry);
                                    guide_path.emplace_back(
                                        corner_window->outgoing_blend);
                                    guide_stamp.emplace_back(
                                        guide_stamp.back() + segment_duration(
                                            corner_window->entry,
                                            corner_window->outgoing_blend));
                                    // Keep the exact mission waypoint in the
                                    // guide so CorridorGenerator and MINCO
                                    // assign the route-boundary cell to an
                                    // interior junction, while the outgoing
                                    // endpoint remains inside the same
                                    // acceptance ball.
                                    guide_path.emplace_back(current_endpoint);
                                    guide_stamp.emplace_back(
                                        guide_stamp.back() + segment_duration(
                                            corner_window->outgoing_blend,
                                            current_endpoint));
                                    guide_path.emplace_back(corner_window->endpoint);
                                    guide_stamp.emplace_back(
                                        guide_stamp.back() + segment_duration(
                                            current_endpoint,
                                            corner_window->endpoint));
                                }
                                const double guide_time_origin_s = guide_stamp.back();
                                for (std::size_t index = 0;
                                     index < allocation.points.size(); ++index) {
                                    guide_path.emplace_back(allocation.points[index]);
                                    guide_stamp.emplace_back(
                                        guide_time_origin_s + allocation.elapsed_s[index]);
                                }
                                guide_path_end_vel = allocation.terminal_velocity_mps;
                                gi_.goal_p = guide_path.back();
                                planning_goal_p_ = gi_.goal_p;
                                goal_endpoint_adjusted_ = true;
                                route_lookahead_active = true;
                                route_lookahead_is_corner = effective_genuine_corner;
                                certified_lookahead_m_ = allocation.path_length_m;
                                lookahead_complete_ = complete;
                                // Every outgoing lookahead must still cross
                                // the controller-owned acceptance region. The
                                // optimizer may choose any dynamically smooth
                                // junction inside this ball; it is no longer
                                // pinned to the exact waypoint centre.
                                route_boundary_gate = CorridorGenerator::RouteBoundaryGate{
                                    current_endpoint, goal_acceptance_radius_m_};
                                planner_context_->info(
                                    " -- [planner] pass-through route lookahead distance={:.3f} "
                                    "required={:.3f} complete={} terminal_speed_cap={:.3f} "
                                    "remaining_horizon={:.3f} corner={}",
                                    allocation.path_length_m, desired_lookahead, complete,
                                    route_terminal_speed_cap_mps, remaining_horizon,
                                    effective_genuine_corner);
                            }
                        }
                    }
                }
            }
        }

        // If no long route lookahead is certified, retain a bounded
        // acceptance-ball fillet for a genuine corner. Ending the incoming
        // command at the exact waypoint with non-zero incoming velocity leaves
        // the next measured handoff an instantaneous direction change and can
        // exhaust its short planning window before the old command ends.
        bool route_window_endpoint = false;
        if (!route_lookahead_active &&
            pass_through_next_target_.has_value() &&
            guide_path.size() >= 2U &&
            guide_stamp.size() == guide_path.size() &&
            (guide_path.back() - gi_.goal_p).norm() <=
                navigation_world_model::kGoalConnectionToleranceM + 1.0e-6 &&
            (gi_.goal_p - requested_goal_p_).norm() <=
                navigation_world_model::kGoalConnectionToleranceM + 1.0e-6) {
            Eigen::Vector3d incoming_tangent = Eigen::Vector3d::Zero();
            for (std::size_t index = guide_path.size(); index > 1U; --index) {
                const Eigen::Vector3d delta =
                    (guide_path[index - 1U] - guide_path[index - 2U]).cast<double>();
                if (delta.norm() > 1.0e-6) {
                    incoming_tangent = delta;
                    break;
                }
            }
            auto route_window = passThroughRouteWindow(
                requested_goal_p_.cast<double>(), *pass_through_next_target_,
                incoming_tangent, goal_acceptance_radius_m_,
                navigation_world_model::kGoalConnectionToleranceM);
            // Prefer the measured A* approach when it proves the corner. If
            // the detour's final edge is shallow, retry with the immutable
            // mission incoming tangent so the same corner classification used
            // above also creates its bounded acceptance-ball fillet.
            if (!route_window.has_value()) {
                const auto mission_tangent = mission_incoming_tangent();
                if (mission_tangent.has_value()) {
                    route_window = passThroughRouteWindow(
                        requested_goal_p_.cast<double>(),
                        *pass_through_next_target_, *mission_tangent,
                        goal_acceptance_radius_m_,
                        navigation_world_model::kGoalConnectionToleranceM);
                }
            }
            const auto point_is_traversable = [this](const Eigen::Vector3d& point) {
                return point.allFinite() && map_ptr_->contains(point) &&
                    navigation_world_model::isCellTraversable(
                        map_ptr_->classify(
                            point, navigation_world_model::GridLayer::kInflated),
                        unknownPolicy());
            };
            const auto segment_is_traversable = [this](
                    const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
                return map_ptr_->isSegmentTraversable(
                    start, end, navigation_world_model::GridLayer::kInflated,
                    unknownPolicy());
            };
            const Eigen::Vector3d predecessor =
                guide_path[guide_path.size() - 2U].cast<double>();
            if (route_window.has_value() &&
                point_is_traversable(route_window->entry) &&
                point_is_traversable(route_window->outgoing_blend) &&
                point_is_traversable(route_window->endpoint) &&
                segment_is_traversable(predecessor, route_window->entry) &&
                segment_is_traversable(route_window->entry,
                                       route_window->outgoing_blend) &&
                segment_is_traversable(route_window->outgoing_blend,
                                       route_window->endpoint) &&
                guide_path.size() >= 2U) {
                const double first_segment_length =
                    (route_window->entry - predecessor).norm();
                const double blend_segment_length =
                    (route_window->outgoing_blend - route_window->entry).norm();
                const double final_segment_length =
                    (route_window->endpoint - route_window->outgoing_blend).norm();
                const double previous_stamp = guide_stamp.size() >= 2U
                    ? guide_stamp[guide_stamp.size() - 2U] : 0.0;
                const double first_segment_duration =
                    first_segment_length / cfg_.exp_traj_cfg.max_vel;
                const double blend_segment_duration =
                    blend_segment_length / cfg_.exp_traj_cfg.max_vel;
                const double final_segment_duration =
                    final_segment_length / cfg_.exp_traj_cfg.max_vel;
                if (std::isfinite(first_segment_length) && first_segment_length > 1.0e-6 &&
                    std::isfinite(blend_segment_length) && blend_segment_length > 1.0e-6 &&
                    std::isfinite(final_segment_length) && final_segment_length > 1.0e-6 &&
                    std::isfinite(first_segment_duration) && first_segment_duration > 0.0 &&
                    std::isfinite(blend_segment_duration) && blend_segment_duration > 0.0 &&
                    std::isfinite(final_segment_duration) && final_segment_duration > 0.0 &&
                    std::isfinite(previous_stamp)) {
                    guide_path.back() = route_window->entry;
                    guide_path.emplace_back(route_window->outgoing_blend);
                    guide_path.emplace_back(route_window->endpoint);
                    guide_stamp.back() = previous_stamp + first_segment_duration;
                    guide_stamp.emplace_back(
                        guide_stamp.back() + blend_segment_duration);
                    guide_stamp.emplace_back(
                        guide_stamp.back() + final_segment_duration);
                    const double route_window_offset =
                        (route_window->endpoint - requested_goal_p_.cast<double>()).norm();
                    gi_.goal_p = route_window->endpoint;
                    planning_goal_p_ = gi_.goal_p;
                    goal_endpoint_adjusted_ = true;
                    route_window_endpoint = true;
                    planner_context_->info(
                        " -- [planner] pass-through route window fillet offset={:.3f} "
                        "acceptance_radius={:.3f}",
                        route_window_offset, goal_acceptance_radius_m_);
                }
            }
        }

        // Keep the route geometry observable at the optimizer boundary.  A
        // large lateral excursion in an open mission is a map/A* or guide
        // construction decision; it must not be misdiagnosed as PX4 tracking
        // lag from the final MINCO control points.
        const Eigen::Vector3d diagnostic_route_goal = planning_goal_p_.allFinite()
            ? planning_goal_p_.cast<double>() : requested_goal_p_.cast<double>();
        if (guide_path.size() >= 2U && diagnostic_route_goal.allFinite()) {
            const Eigen::Vector3d route_start = guide_path.front().cast<double>();
            const Eigen::Vector3d route_delta =
                diagnostic_route_goal - route_start;
            const double route_length_squared = route_delta.squaredNorm();
            if (std::isfinite(route_length_squared) && route_length_squared > 1.0e-9) {
                double maximum_lateral_error = 0.0;
                Eigen::Vector3d maximum_lateral_point = route_start;
                for (std::size_t index = 1U; index + 1U < guide_path.size(); ++index) {
                    const auto& point = guide_path[index];
                    const Eigen::Vector3d point_d = point.cast<double>();
                    const double projection = std::clamp(
                        (point_d - route_start).dot(route_delta) /
                            route_length_squared,
                        0.0, 1.0);
                    const Eigen::Vector3d projected = route_start + projection * route_delta;
                    const double lateral_error = (point_d - projected).norm();
                    if (std::isfinite(lateral_error) && lateral_error > maximum_lateral_error) {
                        maximum_lateral_error = lateral_error;
                        maximum_lateral_point = point_d;
                    }
                }
                if (maximum_lateral_error > 0.5) {
                    planner_context_->warn(
                        " -- [planner] guide has large interior lateral excursion before MINCO: "
                        "error={} point=({}, {}, {}) start=({}, {}, {}) "
                        "requested_goal=({}, {}, {}) planning_goal=({}, {}, {}) "
                        "points={} pass_through_next={}",
                        maximum_lateral_error,
                        maximum_lateral_point.x(), maximum_lateral_point.y(),
                        maximum_lateral_point.z(), route_start.x(), route_start.y(),
                        route_start.z(), requested_goal_p_.x(), requested_goal_p_.y(),
                        requested_goal_p_.z(), diagnostic_route_goal.x(),
                        diagnostic_route_goal.y(), diagnostic_route_goal.z(),
                        guide_path.size(),
                        pass_through_next_target_.has_value());
                }
            }
        }

        // Resolve the terminal point before corridor construction. Snapping
        // only the MINCO tail after CIRI has certified the unsnapped guide can
        // place the fixed endpoint outside every generated polytope.
        const bool direct_goal_segment_safe =
            navigation_world_model::isGoalSegmentTraversable(
                *map_ptr_, guide_path.back().cast<double>(), gi_.goal_p.cast<double>());
        const GuideEndpoint resolved_endpoint = direct_goal_segment_safe
            ? resolveGuideEndpoint(
                  guide_path.back(), gi_.goal_p,
                  navigation_world_model::kGoalConnectionToleranceM)
            : GuideEndpoint{guide_path.back(), false};
        guide_path.back() = resolved_endpoint.position;
        const bool connected_goal = !route_lookahead_active &&
            (resolved_endpoint.goal_connected || route_window_endpoint);
        const bool mission_goal_connected = !route_lookahead_active &&
            mission_planning_goal.allFinite() &&
            (resolved_endpoint.position - mission_planning_goal).norm() <=
                navigation_world_model::kGoalConnectionToleranceM + 1.0e-6;
        out_exp_traj_info.setGoalConnectedFlag(connected_goal);

        latest_guide_start_ = guide_path.front();
        latest_guide_end_ = guide_path.back();
        latest_guide_min_ = guide_path.front();
        latest_guide_max_ = guide_path.front();
        for (const auto &point : guide_path) {
            latest_guide_min_ = latest_guide_min_.cwiseMin(point);
            latest_guide_max_ = latest_guide_max_.cwiseMax(point);
        }
        latest_guide_path_length_m_ = geometry_utils::computePathLength(guide_path);
        latest_guide_duration_s_ = guide_stamp.empty()
                ? std::numeric_limits<double>::quiet_NaN()
                : guide_stamp.back() - guide_stamp.front();

        sfc.clear();
        {
            TimeConsuming t_viz("tviz", false);
            planner_context_->vizFrontendPath(guide_path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }
        shifted_sfc_start_pt_ = Vec3f(9999,9999,9999);
        solve_stage_.store(3);
        bool bool_ret_code = cg_ptr_->SearchPolytopeOnPath(
            guide_path, sfc, shifted_sfc_start_pt_, cfg_.use_fov_cut,
            &solve_deadline, route_boundary_gate);

        if (!bool_ret_code) {
            planner_context_->warn(" -- [planner] SearchPolytopeOnPath for new path failed");
            return FAILED;
        }
        const auto guide_vertical_envelope = deriveGuideVerticalEnvelope(
            guide_path, map_ptr_->geometry().inflated_resolution_m);
        if (!applyGuideVerticalEnvelope(sfc, guide_vertical_envelope)) {
            planner_context_->warn(
                " -- [planner] failed to bind SFC to scale-aware guide vertical envelope");
            return FAILED;
        }
        {
            TimeConsuming t_viz("tviz", false);
            planner_context_->vizExpSfc(sfc);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        time_consuming_[EPX_TRAJ_FRONTEND] = t_exp_frontend.stop();

        const Eigen::Vector3d guide_direction =
            (guide_path.back() - guide_path.front()).cast<double>();
        const auto directional_support =
            navigation_world_model::directionalSupportToLocalBoundary(
                guide_path.front().cast<double>(), guide_direction,
                map_ptr_->geometry());
        const double route_support_m = geometry_utils::computePathLength(guide_path);
        const double known_free_support_m = knownFreeGuideSupport(*map_ptr_, guide_path);
        const auto governed_speed = evidenceAwareSpeedLimit(
            cfg_.exp_traj_cfg.max_vel,
            cfg_.back_traj_cfg.max_acc, cfg_.back_traj_cfg.max_jerk,
            {navigation_planning::PlanningTimingContract::kLocalWindowM,
             directional_support.value_or(0.0), route_support_m,
             known_free_support_m});
        if (!governed_speed.sufficient || governed_speed.speed_mps <= 0.0) {
            planner_context_->warn(
                " -- [planner] MAIN rejected: insufficient braking evidence "
                "local=20.000 directional={:.3f} route={:.3f} known_free={:.3f}",
                directional_support.value_or(0.0), route_support_m,
                known_free_support_m);
            return FAILED;
        }

        pos_fina_state.setZero();
        pos_fina_state.col(0) = guide_path.back();
        if (connected_goal && pass_through_next_target_.has_value()) {
            // Keep the outgoing tangent just inside the optimizer's declared
            // interior search target. The mission/product V/A/J limits below
            // remain unchanged; this only avoids asking a short MINCO guide
            // to reach the exact speed boundary while rotating toward the
            // next leg, which otherwise amplifies hard-gate churn at a
            // pass-through waypoint.
            double terminal_velocity_cap =
                    std::min(cfg_.exp_traj_cfg.max_vel,
                             governed_speed.speed_mps) *
                    cfg_.exp_traj_cfg.optimization_dynamic_reserve_ratio;
            if (route_window_endpoint) {
                const double fillet_radius_m =
                    (pos_fina_state.col(0).cast<double>() -
                     requested_goal_p_.cast<double>()).norm();
                const double corner_speed_cap = passThroughCornerSpeedCap(
                    fillet_radius_m, cfg_.exp_traj_cfg.max_acc,
                    terminal_velocity_cap);
                if (corner_speed_cap > 0.0) {
                    terminal_velocity_cap = std::min(
                        terminal_velocity_cap, corner_speed_cap);
                    planner_context_->info(
                        " -- [planner] acceptance-fillet terminal speed cap={:.3f} "
                        "radius={:.3f}",
                        terminal_velocity_cap, fillet_radius_m);
                }
            }
            const auto terminal_velocity = passThroughTerminalVelocity(
                    pos_fina_state.col(0).cast<double>(),
                    *pass_through_next_target_,
                    pos_init_state.col(1).cast<double>(),
                    guide_stamp.empty() ? std::numeric_limits<double>::quiet_NaN()
                                         : guide_stamp.back(),
                    terminal_velocity_cap,
                    cfg_.exp_traj_cfg.max_acc,
                    cfg_.exp_traj_cfg.max_jerk);
            Eigen::Vector3d guide_tangent = Eigen::Vector3d::Zero();
            for (std::size_t index = guide_path.size(); index > 1U; --index) {
                const Eigen::Vector3d delta =
                    (guide_path[index - 1U] - guide_path[index - 2U]).cast<double>();
                if (delta.norm() > 1.0e-6) {
                    guide_tangent = delta.normalized();
                    break;
                }
            }
            const Eigen::Vector3d outgoing_delta =
                    *pass_through_next_target_ - pos_fina_state.col(0).cast<double>();
            const bool genuine_corner = guide_tangent.norm() > 0.5 &&
                    outgoing_delta.allFinite() && outgoing_delta.norm() > 1.0e-6 &&
                    guide_tangent.dot(outgoing_delta.normalized()) <= 0.7;
            if (genuine_corner && guide_path.size() >= 2U && !route_window_endpoint) {
                // A corner is a hard route boundary, not a request to rotate
                // the velocity vector at the endpoint of the incoming MINCO
                // solve. Keep the incoming tangent at the waypoint and let
                // the next measured handoff solve the outgoing leg.
                const double guide_duration_s = guide_stamp.empty()
                    ? std::numeric_limits<double>::quiet_NaN()
                    : guide_stamp.back();
                const double corner_speed_cap = std::max(
                    1.0e-3,
                    std::min(
                        terminal_velocity_cap,
                        terminalSpeedCapForPath(
                            geometry_utils::computePathLength(guide_path),
                            guide_duration_s,
                            guide_tangent.dot(pos_init_state.col(1).cast<double>()),
                            terminal_velocity_cap)));
                const auto incoming_corner_velocity = frontierContinuationVelocity(
                    pos_fina_state.col(0).cast<double>(),
                    guide_path[guide_path.size() - 2U].cast<double>(),
                    pos_init_state.col(1).cast<double>(),
                    corner_speed_cap,
                    terminal_velocity_cap,
                    cfg_.exp_traj_cfg.max_acc,
                    cfg_.exp_traj_cfg.max_jerk,
                    guide_duration_s);
                if (incoming_corner_velocity.has_value()) {
                    pos_fina_state.col(1) = *incoming_corner_velocity;
                    // The jerk-limited blend above preserves continuity when
                    // the guide is long enough. On a short final leg it can
                    // still exceed the displacement-derived terminal-speed
                    // cap because the measured initial velocity is fixed.
                    // Prefer the attainable incoming tangent cap in that
                    // case; the MINCO and immutable-world certificates remain
                    // authoritative for the complete command.
                    if (pos_fina_state.col(1).norm() > corner_speed_cap + 1.0e-6) {
                        pos_fina_state.col(1) = guide_tangent * corner_speed_cap;
                    }
                }
            } else if (terminal_velocity.has_value()) {
                const double incoming_speed_along_path =
                    guide_tangent.dot(pos_init_state.col(1).cast<double>());
                const double path_terminal_speed_cap = terminalSpeedCapForPath(
                    geometry_utils::computePathLength(guide_path),
                    guide_stamp.empty() ? std::numeric_limits<double>::quiet_NaN()
                                         : guide_stamp.back(),
                    incoming_speed_along_path,
                    terminal_velocity_cap);
                if (guide_tangent.norm() > 0.5 &&
                    path_terminal_speed_cap + 1.0e-6 < terminal_velocity->norm()) {
                    // The current waypoint is too close for the requested
                    // outgoing corner velocity. Keep the incoming leg and
                    // let the next goal transition own the turn; forcing the
                    // corner here creates an impossible P/V boundary and
                    // repeated optimizer failure.
                    pos_fina_state.col(1) = guide_tangent * std::min(
                        path_terminal_speed_cap, terminal_velocity_cap);
                } else {
                    pos_fina_state.col(1) = *terminal_velocity;
                }
            }
        }
        if (!connected_goal && guide_path.size() >= 2U) {
            // A frontier endpoint must continue along the current guide leg.
            // Applying the next waypoint tangent here would turn toward a
            // corner before the active waypoint is reached; leaving the
            // endpoint at rest would create the stop/restart jitter that the
            // receding planner is intended to avoid.
            const double terminal_velocity_cap =
                    cfg_.exp_traj_cfg.max_vel *
                    cfg_.exp_traj_cfg.optimization_dynamic_reserve_ratio;
            double preferred_terminal_speed = std::min(
                std::max(guide_path_end_vel, solve_state_.v.norm()),
                route_terminal_speed_cap_mps);
            if (route_lookahead_active && route_lookahead_is_corner &&
                pass_through_next_target_.has_value()) {
                // The route-boundary corridor is intentionally local to the
                // acceptance region. Do not request full cruise speed at its
                // far endpoint: the next measured handoff will own recovery
                // on the outgoing leg, while this solve must leave room for
                // a certified heading change through the boundary.
                const double corner_speed_cap = passThroughCornerSpeedCap(
                    goal_acceptance_radius_m_, cfg_.exp_traj_cfg.max_acc,
                    terminal_velocity_cap);
                if (corner_speed_cap > 0.0) {
                    preferred_terminal_speed = std::min(
                        preferred_terminal_speed, corner_speed_cap);
                    planner_context_->info(
                        " -- [planner] pass-through corner terminal speed cap={:.3f} "
                        "preferred={:.3f}",
                        corner_speed_cap, guide_path_end_vel);
                }
            }
            const auto terminal_velocity = frontierContinuationVelocity(
                    guide_path.back().cast<double>(),
                    guide_path[guide_path.size() - 2U].cast<double>(),
                    pos_init_state.col(1).cast<double>(),
                    preferred_terminal_speed,
                    terminal_velocity_cap,
                    cfg_.exp_traj_cfg.max_acc,
                    cfg_.exp_traj_cfg.max_jerk,
                    guide_stamp.empty() ? std::numeric_limits<double>::quiet_NaN()
                                         : guide_stamp.back());
            if (terminal_velocity.has_value()) {
                pos_fina_state.col(1) = *terminal_velocity;
            }
        }
        if (connected_goal) {
            if (pass_through_coincident_terminal_stop_) {
                planner_context_->info(
                    " -- [planner] pass-through checkpoint is coincident with "
                    "following STOP; suppressing outgoing terminal velocity");
            }
            // Preserve an outgoing tangent only when the same solve has a
            // certified outgoing lookahead.  If the local horizon cannot
            // certify that leg, this command must terminate at the current
            // waypoint acceptance ball; exposing a moving endpoint would
            // force BACKUP to start before the checkpoint or require
            // KNOWN_FREE evidence beyond the available horizon.
            if (!pass_through_next_target_.has_value() ||
                !route_lookahead_active) {
                pos_fina_state.col(1).setZero();
                if (pass_through_next_target_.has_value()) {
                    planner_context_->info(
                        " -- [planner] pass-through outgoing lookahead is not "
                        "certified; terminating at acceptance-ball endpoint");
                }
            }
        }
        // The mission may declare a distant final STOP while this solve only
        // reaches a bounded local prefix. Keep that prefix moving; activate
        // terminal semantics only once the current endpoint is connected to
        // the actual STOP waypoint.
        candidate_terminal_stop_active_ =
            terminal_stop_required_ && mission_goal_connected;
        if (terminal_stop_required_ && connected_goal &&
            !mission_goal_connected) {
            planner_context_->info(
                " -- [planner] remote STOP remains moving on bounded local prefix: "
                "local_endpoint=({}, {}, {}) mission_endpoint=({}, {}, {})",
                resolved_endpoint.position.x(), resolved_endpoint.position.y(),
                resolved_endpoint.position.z(), mission_planning_goal.x(),
                mission_planning_goal.y(), mission_planning_goal.z());
        }
        if (candidate_terminal_stop_active_) {
            // STOP is an explicit mission terminal contract.  It must not
            // inherit an outgoing tangent or a non-zero frontier velocity,
            // even when the route snapshot also carries a next waypoint for
            // diagnostics.  Otherwise a short residual STOP motion is handed
            // to backup generation as a moving terminal and can collapse its
            // visibility seed to a zero-length CIRI line.
            if (pos_fina_state.col(1).norm() > 1.0e-9) {
                planner_context_->info(
                    " -- [planner] terminal STOP forces zero terminal velocity "
                    "after route/goal endpoint selection speed={}",
                    pos_fina_state.col(1).norm());
            }
            pos_fina_state.col(1).setZero();
            pos_fina_state.col(2).setZero();
            pos_fina_state.col(3).setZero();
        }

        // optimize and update exp traj
        bool temp_ret;
        Trajectory out_traj;
        TimeConsuming t_exp_opt("t_exp_opt", false);
        auto original_sfc = sfc;
        solve_stage_.store(4);
        const auto committed_before_refinement = cmd_traj_info_.snapshot();
        const double remaining_main_before_refinement_s =
            committed_before_refinement.empty
                ? std::numeric_limits<double>::infinity()
                : committed_before_refinement.position.start_WT +
                      committed_before_refinement.backup_start_tt -
                      planner_context_->getSimTime();
        const double committed_elapsed_before_refinement_s =
            committed_before_refinement.empty
                ? std::numeric_limits<double>::quiet_NaN()
                : planner_context_->getSimTime() -
                      committed_before_refinement.position.start_WT;
        const bool committed_main_active =
            !committed_before_refinement.empty &&
            std::isfinite(committed_elapsed_before_refinement_s) &&
            committed_elapsed_before_refinement_s >= 0.0 &&
            committed_elapsed_before_refinement_s <
                committed_before_refinement.backup_start_tt;
        const bool urgent_baseline =
            committed_main_active &&
            std::isfinite(remaining_main_before_refinement_s) &&
            remaining_main_before_refinement_s > 0.0 &&
            remaining_main_before_refinement_s <=
                navigation_planning::PlanningTimingContract::
                    kUrgentBaselineThresholdS;
        const auto refinement_deadline_ns = urgent_baseline
            ? std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now().time_since_epoch()).count()
            : solve_deadline.refinementDeadlineNanoseconds(
                  cfg_.finalization_reserve_s);
        exp_traj_opt_->setSolveBudget(
                &solve_cancelled_, refinement_deadline_ns);
        const auto nominal_result = exp_traj_opt_->solve(
                pos_init_state, pos_fina_state, guide_path, guide_stamp,
                sfc, out_traj,
                urgent_baseline || solve_deadline.conservativeRemaining(
                    planner_context_->getSimTime()) <= cfg_.finalization_reserve_s);
        temp_ret = nominal_result.candidateAvailable();
        time_consuming_[EXP_TRAJ_OPT] = t_exp_opt.stop();
        {
            VecDf init_ts;
            vec_Vec3f init_ps;
            exp_traj_opt_->getInitValue(init_ts, init_ps);
            latest_replan.setExpCondition(init_ts, init_ps, pos_init_state, pos_fina_state, sfc);
        }
        if (!temp_ret) {
            planner_context_->warn(" -- [planner] OptimizationExpTrajInPolytopes for new path failed");
            return FAILED;
        }
        double replan_total_t = (planner_context_->getSimTime() - replan_process_start_WT);
        if (!last_exp_traj_info.empty() && replan_total_t > cfg_.replan_forward_dt_s) {
            planner_context_->warn(" -- [planner] Replan over time({})!!!! Return FAILED", replan_total_t);
            return FAILED;
        }

        {
            TimeConsuming t_viz("tviz", false);
            planner_context_->vizExpTraj(out_traj);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        // Both rest-to-rest and hot-replan trajectories are seeded from the
        // state captured at the beginning of this solve.  Keep that same
        // command clock; moving a rest-plan's start to the post-optimization
        // time would pair a current timestamp with a historical PVA sample
        // when optimizer latency lets the vehicle continue moving.  The
        // execution boundary samples this candidate at the current time and
        // rejects it if the propagated state has diverged beyond the existing
        // anchor envelope.
        const double new_traj_WT = replan_process_start_WT;

        Trajectory temp_exp_traj;
        if (!last_exp_traj_info.empty() &&
            !guide_pos_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                       temp_exp_traj)) {
            planner_context_->error(" -- [planner] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
            return FAILED;
        }
        out_exp_traj_info.setSFC(sfc);
        temp_exp_traj = temp_exp_traj + out_traj;
        temp_exp_traj.start_WT = new_traj_WT; //last_exp_traj_info.replan_start_WT ;
        const auto generated_boundary_state = temp_exp_traj.getState(0.0);
        StatePVAJ expected_boundary_state = pos_init_state;
        bool expected_boundary_valid = expected_boundary_state.allFinite();
        if (!last_exp_traj_info.empty()) {
            expected_boundary_valid = guide_pos_traj.getState(
                replan_process_start_TT, expected_boundary_state);
        }
        if (expected_boundary_valid && generated_boundary_state.allFinite()) {
            const double boundary_error = (
                generated_boundary_state.col(0) - expected_boundary_state.col(0)).norm();
            if (!std::isfinite(boundary_error) || boundary_error > 0.02) {
                planner_context_->warn(
                    " -- [planner] generated EXP boundary disagrees with its seed: "
                    "mode={} solve_start={} output_start={} boundary_error={} "
                    "solve_p=({},{},{}) expected_p=({},{},{}) generated_p=({},{},{})",
                    last_exp_traj_info.empty() ? "measured" : "hot",
                    replan_process_start_WT, new_traj_WT, boundary_error,
                    solve_state_.p.x(), solve_state_.p.y(), solve_state_.p.z(),
                    expected_boundary_state.col(0).x(), expected_boundary_state.col(0).y(),
                    expected_boundary_state.col(0).z(), generated_boundary_state.col(0).x(),
                    generated_boundary_state.col(0).y(), generated_boundary_state.col(0).z());
            }
        }
        double required_main_prefix_duration_TT = 0.0;

        if (!last_exp_traj_info.empty()) {
            StatePVAJ yaw_replan_state;
            if (!guide_yaw_traj.getState(replan_state_TT, yaw_replan_state)) {
                planner_context_->warn(" -- [planner] Invalid traj or eval t");
                return FAILED;
            }
            init_yaw = yaw_replan_state.row(0);
        }


        Trajectory new_traj, old_traj;

        if (!yaw_traj_opt_->optimizeToTarget(
                init_yaw, route_yaw_reference_.target_yaw_rad,
                out_traj, new_traj)) {
            const auto& yaw = yaw_traj_opt_->lastDiagnostics();
            planner_context_->error(
                " -- [planner] in [generateExpTraj]: YawTrajOpt failed reason={} "
                "duration={} init=[{},{},{}] target={} delta={} "
                "full_max=[{},{}] hold_max=[{},{}] stop=[{},{},{}] limits=[{},{}]",
                static_cast<int>(yaw.failure), yaw.duration_s,
                yaw.initial_state(0), yaw.initial_state(1), yaw.initial_state(2),
                yaw.target_yaw_rad, yaw.requested_delta_rad,
                yaw.full_turn_max_rate_rad_s,
                yaw.full_turn_max_acceleration_rad_s2,
                yaw.hold_max_rate_rad_s,
                yaw.hold_max_acceleration_rad_s2,
                yaw.stopping_displacement_rad,
                yaw.stopping_max_rate_rad_s,
                yaw.stopping_max_acceleration_rad_s2,
                cfg_.yaw_rate_max_rad_s, cfg_.yaw_acceleration_max_rad_s2);
            return FAILED;
        }
        if (!last_exp_traj_info.empty()) {
            if (!guide_yaw_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                           old_traj)) {
                planner_context_->error(" -- [planner] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
                return FAILED;
            }
        }

        auto temp_yaw_traj = old_traj + new_traj;

        // The route-boundary gate is enforced by the optimized MAIN
        // corridor. Carry the first independently evaluated entry time into
        // ExpTraj so a visibility/braking BACKUP suffix cannot become active
        // before the vehicle has entered the mission acceptance ball. Without
        // this certificate the composed command can expose a stop suffix on
        // the incoming leg, even though the nominal MAIN itself crosses the
        // waypoint.
        if (route_lookahead_active && route_boundary_gate.has_value()) {
            const auto route_boundary_time = firstRouteBoundaryEntryTime(
                temp_exp_traj,
                route_boundary_gate->point,
                route_boundary_gate->radius_m);
            if (!route_boundary_time.has_value()) {
                double nearest_junction_distance_m =
                    std::numeric_limits<double>::infinity();
                int nearest_junction_index = -1;
                for (int junction_index = 0;
                     junction_index <= temp_exp_traj.getPieceNum();
                     ++junction_index) {
                    const Eigen::Vector3d position =
                        temp_exp_traj.getJuncPos(junction_index);
                    const double distance =
                        (position - route_boundary_gate->point).norm();
                    if (std::isfinite(distance) &&
                        distance < nearest_junction_distance_m) {
                        nearest_junction_distance_m = distance;
                        nearest_junction_index = junction_index;
                    }
                }
                planner_context_->error(
                    " -- [planner] pass-through MAIN missing route-boundary "
                    "entry witness point=({}, {}, {}) radius={} pieces={} "
                    "nearest_junction_distance={} index={}",
                    route_boundary_gate->point.x(), route_boundary_gate->point.y(),
                    route_boundary_gate->point.z(), route_boundary_gate->radius_m,
                    temp_exp_traj.getPieceNum(), nearest_junction_distance_m,
                    nearest_junction_index);
                return FAILED;
            }
            required_main_prefix_duration_TT = *route_boundary_time;
            planner_context_->info(
                " -- [planner] certified pass-through MAIN prefix before BACKUP: "
                "duration={} boundary=({}, {}, {}) radius={}",
                required_main_prefix_duration_TT,
                route_boundary_gate->point.x(), route_boundary_gate->point.y(),
                route_boundary_gate->point.z(), route_boundary_gate->radius_m);
        }

        traj_opt::TrajectoryDynamicReport exp_dynamic_report;
        if (!traj_opt::trajectorySatisfiesFlatnessEnvelope(
                temp_exp_traj, cfg_.exp_traj_cfg, &exp_dynamic_report,
                0.01, &temp_yaw_traj)) {
            planner_context_->error(
                    " -- [planner] combined EXP position/yaw body-rate or thrust gate failed: "
                    "body_rate={} thrust=[{},{}]",
                    exp_dynamic_report.maximum_body_rate_rad_s,
                    exp_dynamic_report.minimum_thrust_n,
                    exp_dynamic_report.maximum_thrust_n);
            return FAILED;
        }
        // check if part of the exp on last backup
        double on_backup_end_TT{-1}, on_backup_start_TT{-1};
        if (!last_exp_traj_info.empty()) {
            const auto inherited_backup = inheritedBackupInterval(
                    cmd_traj_info_.getBackupTrajStartTT(),
                    replan_window,
                    temp_exp_traj.getTotalDuration());
            if (!inherited_backup.valid) {
                planner_context_->error(
                        " -- [generateExpTraj] invalid inherited BACKUP interval");
                return FAILED;
            }
            if (inherited_backup.present) {
                on_backup_start_TT = inherited_backup.begin_tt_s;
                on_backup_end_TT = inherited_backup.end_tt_s;
            }
        }
        out_exp_traj_info.setTrajectory(new_traj_WT, temp_exp_traj, temp_yaw_traj, on_backup_start_TT,
                                        on_backup_end_TT);
        if (!out_exp_traj_info.setRequiredMainPrefixDuration(
                required_main_prefix_duration_TT)) {
            planner_context_->error(
                    " -- [planner] invalid required main prefix duration={}",
                    required_main_prefix_duration_TT);
            return FAILED;
        }

        latest_replan.setExpYawTraj(temp_yaw_traj);
        latest_replan.setExpTraj(temp_exp_traj);

        return SUCCESS;
    }

    RET_CODE Planner::generateBackupTrajectory(
            ExpTraj &ref_exp_traj,
            BackupTraj &back_traj_info,
            const AbsoluteDeadline &solve_deadline) {
        // The executable candidate starts at the first sample of the newly
        // generated EXP trajectory.  PlanFromRest may have moved that sample
        // by one inflated voxel when the measured pose lies in an occupied
        // raster cell.  Building the backup visibility ray from solve_state_
        // in that case creates an artificial blocked first segment and makes
        // CIRI fail on a degenerate seed.  Use the actual command boundary;
        // authorizeAndStage() still validates the complete main+backup bundle
        // against the latest immutable world before publication.
        TimeConsuming t_back_frontend("t_back_frontend", false);
        backup_certificate_diagnostics_ = {};
        backup_certificate_diagnostics_.attempted = true;
        double total_dur = ref_exp_traj.getTotalDuration();
        double start_t = planner_context_->getSimTime() - ref_exp_traj.getStartWallTime();


        if (start_t > total_dur - 0.01) {
            if (cfg_.print_log) {
                planner_context_->info(" -- [planner] in [generateBackupTrajectory]: start_t > total_dur, return NO_NEED");
            }
            return NO_NEED;
        }

        const double command_start_t = std::clamp(start_t, 0.0, total_dur);
        const Vec3f command_start = ref_exp_traj.getPos(command_start_t);
        if (!command_start.allFinite() || !map_ptr_->contains(command_start) ||
            !map_ptr_->isSegmentTraversable(
                command_start, command_start,
                navigation_world_model::GridLayer::kInflated,
                navigation_world_model::UnknownPolicy::kRequireKnownFree)) {
            backup_certificate_diagnostics_.last_reject_stage = static_cast<int>(
                navigation_planning::BackupCertificateRejectStage::kCommandBoundary);
            planner_context_->warn(
                    " -- [planner] backup command boundary is not KNOWN_FREE; "
                    "rejecting backup origin command=({}, {}, {}) measured=({}, {}, {})",
                    command_start.x(), command_start.y(), command_start.z(),
                    solve_state_.p.x(), solve_state_.p.y(), solve_state_.p.z());
            return FAILED;
        }
        back_traj_info.setRobotPos(command_start);
        if (cfg_.print_log &&
            (command_start - solve_state_.p).norm() > cfg_.resolution * 0.5) {
            planner_context_->info(
                    " -- [planner] backup visibility origin follows executable "
                    "command boundary command=({}, {}, {}) measured=({}, {}, {}) "
                    "offset_m={}",
                    command_start.x(), command_start.y(), command_start.z(),
                    solve_state_.p.x(), solve_state_.p.y(), solve_state_.p.z(),
                    (command_start - solve_state_.p).norm());
        }

        Vec3f temp_point;
        double out_t;
        bool all_traj_visible{true};
        vector<TimePosPair> eval_ps;

        // Collect geometrically distinct samples first. A nearly straight
        // trajectory shares one sensor ray, so checking every longer prefix
        // from the robot is quadratic in trajectory length at fine map
        // resolution. In that common open-space case one farthest-endpoint
        // ray is equivalent for occupied-grid visibility.
        vector<TimePosPair> candidate_ps;
        Vec3f last_pos = ref_exp_traj.getPos(start_t);
        for (out_t = start_t; out_t < total_dur; out_t += cfg_.sample_traj_dt_s) {
            temp_point = ref_exp_traj.getPos(out_t);
            if ((last_pos - temp_point).norm() < cfg_.resolution * 0.8) {
                continue;
            }
            last_pos = temp_point;
            candidate_ps.emplace_back(out_t, temp_point);
        }
        // The loop above intentionally samples only times strictly before the
        // terminal time. The terminal point is still part of the backup
        // visibility certificate; otherwise the all-visible fast path could
        // return FINISH without checking the actual end of the executable
        // trajectory.
        temp_point = ref_exp_traj.getPos(total_dur);
        if (!temp_point.allFinite()) {
            planner_context_->warn(
                    " -- [planner] backup visibility terminal point is non-finite");
            return FAILED;
        }
        candidate_ps.emplace_back(total_dur, temp_point);

        bool shared_visibility_ray{false};
        const Vec3f visibility_origin = back_traj_info.getRobotPos();
        if (!candidate_ps.empty()) {
            const Vec3f farthest_delta = candidate_ps.back().second - visibility_origin;
            const double farthest_distance = farthest_delta.norm();
            if (farthest_distance > cfg_.resolution) {
                const Vec3f ray_direction = farthest_delta / farthest_distance;
                double last_projection{-1.0};
                shared_visibility_ray = true;
                // isLineFree checks the configured robot-radius neighbor tube
                // around the ray. Any stitched prefix contained by that same
                // tube is covered by the endpoint check as well.
                // A single inflated ray may represent another sample only when
                // their centerlines are the same ray. Merely being within one
                // robot radius is unsafe: the two radius-r tubes then overlap
                // but neither contains the other.
                const double lateral_tolerance =
                        std::max(1.0e-9, cfg_.resolution * 1.0e-6);
                for (const auto &sample : candidate_ps) {
                    const Vec3f delta = sample.second - visibility_origin;
                    const double projection = delta.dot(ray_direction);
                    const double lateral_error = (delta - projection * ray_direction).norm();
                    if (projection + cfg_.resolution < last_projection || projection < 0.0 ||
                        projection > farthest_distance + cfg_.resolution ||
                        lateral_error > lateral_tolerance) {
                        shared_visibility_ray = false;
                        break;
                    }
                    last_projection = projection;
                }
            }
        }

        const double visibility_limit =
                cfg_.sensing_horizon_m > 0 ? std::min(cfg_.sensing_horizon_m, cfg_.visibility_horizon_m)
                                           : cfg_.visibility_horizon_m;
        const auto inflated_line_visible = [&](const Vec3f &endpoint) {
            if (visibility_limit > 0.0 &&
                (endpoint - visibility_origin).norm() > visibility_limit) {
                return false;
            }
            // A backup is the fail-safe suffix. Its visibility certificate must
            // be known-free, independent of the exploratory main policy.
            return map_ptr_->isSegmentTraversable(
                    visibility_origin, endpoint,
                    navigation_world_model::GridLayer::kInflated,
                    navigation_world_model::UnknownPolicy::kRequireKnownFree);
        };
        if (shared_visibility_ray &&
            inflated_line_visible(candidate_ps.back().second)) {
            eval_ps = candidate_ps;
            out_t = total_dur;
        } else {
            eval_ps.clear();
            for (const auto &sample : candidate_ps) {
                out_t = sample.first;
                eval_ps.push_back(sample);
                if (!inflated_line_visible(sample.second)) {
                    all_traj_visible = false;
                    break;
                }
            }
            if (all_traj_visible) out_t = total_dur;
        }
        if (all_traj_visible &&
            trajectoryTerminalIsRestWithinRoundoff(ref_exp_traj.posTraj()) &&
            !candidate_terminal_stop_active_) {
            // A main-only command is safe to complete only when every
            // remaining sample is known-free and its terminal PVAJ is a true
            // rest state. A terminal STOP still needs an independently
            // certified braking suffix until measured waypoint acceptance;
            // otherwise tracking drift can expose an uncertified main-only
            // endpoint and force a reactive emergency brake later.
            back_traj_info.setEmpty();
            time_consuming_[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
            return FINISH;
        }
        const bool terminal_rest_capture_ready = all_traj_visible &&
            candidate_terminal_stop_active_ &&
            trajectoryTerminalIsRestWithinRoundoff(ref_exp_traj.posTraj()) &&
            std::isfinite(cfg_.robot_r) && cfg_.robot_r > 0.0 &&
            (candidate_ps.back().second - command_start).norm() <= cfg_.robot_r;
        if (terminal_rest_capture_ready) {
            // Once a measured-stop replan starts inside one vehicle radius of
            // the known-free terminal endpoint, there is no positive-length
            // corridor from which to construct another BACKUP suffix.  The
            // complete main-only candidate is already a rest command; runtime
            // still requires the immutable MAIN endpoint and fresh measured
            // position to satisfy the waypoint acceptance gate.
            planner_context_->info(
                    " -- [planner] terminal rest capture is already within "
                    "certified radius; keeping main-only terminal command "
                    "distance={} radius={}",
                    (candidate_ps.back().second - command_start).norm(), cfg_.robot_r);
            back_traj_info.setEmpty();
            time_consuming_[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
            return FINISH;
        }
        if (all_traj_visible) {
            planner_context_->info(
                    " -- [planner] visible MAIN requires certified backup "
                    "suffix instead of main-only command terminal_stop={} "
                    "terminal_rest={}",
                    candidate_terminal_stop_active_ ? 1 : 0,
                    trajectoryTerminalIsRestWithinRoundoff(ref_exp_traj.posTraj()) ? 1 : 0);
            all_traj_visible = false;
        }
        Vec3f invisible_p = eval_ps.back().second;
        while (out_t > start_t) {
            out_t -= cfg_.sample_traj_dt_s;
            Vec3f out_p = ref_exp_traj.getPos(out_t);
            if ((out_p - invisible_p).norm() > cfg_.robot_r) {
                break;
            }
        }

        double seed_point_t = std::max(start_t, out_t);

        // Seed the backup corridor at the latest point that remains within
        // the visible command prefix and outside the robot-radius retreat
        // band. Completion is decided above from the full visibility and
        // terminal-rest checks; no historical backup timestamp may override
        // this current-world result.
        Vec3f seed_point = ref_exp_traj.getPos(seed_point_t);

        // Do not re-snap this point through the Evidence grid.  The command
        // boundary has already been selected by PlanFromRest/corridor
        // generation and the Evidence-grid centre can move it back toward an
        // obstacle or collapse the first CIRI line to zero length.  The
        // command-boundary KNOWN_FREE check above plus the full candidate
        // validator are the authority here.
        const Vec3f backup_origin = back_traj_info.getRobotPos();
        // The seed has already been truncated at the first sample that is not
        // within the configured KNOWN_FREE visibility horizon. Do not apply
        // corridor_segment_max_length_m here: that parameter subdivides sparse
        // guide edges, but using it as a second backup-distance cap limits an
        // open-space vehicle to a 3 m recovery prefix and forces a brake every
        // few metres. GeneratePolytopeFromLine receives only the certified
        // visibility segment; the full candidate validator remains mandatory.
        if (!std::isfinite(seed_point_t) || !seed_point.allFinite() ||
            (seed_point - backup_origin).norm() <= cfg_.resolution) {
            planner_context_->warn(
                    " -- [planner] backup visibility seed is too short "
                    "after KNOWN_FREE visibility selection length={}",
                    (seed_point - backup_origin).norm());
            return FAILED;
        }
        Line line{backup_origin, seed_point};
        Polytope temp_poly;
        if (!cg_ptr_->GeneratePolytopeFromLine(line, temp_poly, &solve_deadline)) {
            planner_context_->warn(" -- [planner] GeneratePolytopeFromLine failed, force return");
            return FAILED;
        }
        Eigen::Vector3d inner;
        Eigen::Matrix3Xd vPoly;
        if (!geometry_utils::findInterior(temp_poly.GetPlanes(), inner)) {
            planner_context_->warn(" -- [planner] Cannot generate feasible backup sfc, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        if (cfg_.use_fov_cut) {
            if (!fov_checker_->cutPolyByFov(solve_state_.p, solve_state_.q, seed_point,
                                            temp_poly)) {
                planner_context_->warn(" -- [planner] cutPolyByFov failed, force return");
                return FAILED;
            }
        }
        // cut by sensing horizon
        if (cfg_.sensing_horizon_m > 0 &&
            !fov_checker_->cutPolyBySensingHorizon(solve_state_.p, seed_point, cfg_.sensing_horizon_m,
                                                   temp_poly)) {
            planner_context_->warn(" -- [planner] cutPolyBySensingHorizon failed, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        // Keep the visibility corridor separate from the optimizer corridor.
        // The former certifies the EXP prefix up to the first invisible sample;
        // the latter must certify the actual braking hull selected below.
        const Polytope visibility_poly = temp_poly;

//        Vec3f out_p = temp_point;
//        double t_R = 0.0;
        double eval_t = eval_ps.back().first + cfg_.sample_traj_dt_s;
        last_pos = eval_ps.back().second;
        while (visibility_poly.PointIsInside(eval_ps.back().second) && eval_t < total_dur) {
            Vec3f cur_pos = ref_exp_traj.getPos(eval_t);

            if ((cur_pos - last_pos).norm() < cfg_.resolution * 0.8) {
                eval_t += cfg_.sample_traj_dt_s;
                continue;
            }
            eval_ps.emplace_back(eval_t, cur_pos);
            last_pos = cur_pos;
            eval_t += cfg_.sample_traj_dt_s;
        }
        if (eval_ps.size() <= 1U) {
            planner_context_->warn(
                    " -- [planner] backup visibility produced no certified seed before first invisible sample");
            return FAILED;
        }
        eval_ps.pop_back();
        seed_point = eval_ps.back().second;
        seed_point_t = eval_ps.back().first;

        //        bool use_new{true};
        //        if (use_new) {
        double t0 = planner_context_->getSimTime() -
                    ref_exp_traj.getStartWallTime() + 0.01;
        double te = seed_point_t;
        const double required_main_prefix_duration_TT =
            ref_exp_traj.getRequiredMainPrefixDuration();
        if (!std::isfinite(required_main_prefix_duration_TT) ||
            required_main_prefix_duration_TT < 0.0) {
            planner_context_->error(
                    " -- [planner] invalid required main prefix duration={}",
                    required_main_prefix_duration_TT);
            return OPT_FAILED;
        }
        // A measured-state rebase handoff is part of nominal EXP, not a
        // disposable prefix.  Start BACKUP only after the complete connector
        // has elapsed; the full candidate/world certificate still authorizes
        // the connector and the suffix independently.
        const double backup_switch_lower_bound = std::max(
                t0, required_main_prefix_duration_TT);
        if (!std::isfinite(backup_switch_lower_bound) ||
            !(backup_switch_lower_bound < te)) {
            planner_context_->warn(
                " -- [planner] no backup switch window after required main "
                "prefix prefix={} lower_bound={} visibility_end={}",
                required_main_prefix_duration_TT, backup_switch_lower_bound, te);
            return OPT_FAILED;
        }
        //            cout << "t0: " << t0 << endl;
        //            cout << "te: " << te << endl;
        //            cout << "exp_traj_dur: " << ref_exp_traj.optimized_exp_traj.getTotalDuration() << endl;
        double vel_e_n = ref_exp_traj.getVel(te).norm();
        double heu_ts = std::max((t0 + te) / 2, te - vel_e_n / cfg_.back_traj_cfg.max_acc);
        // Preserve planner backend's visibility/braking-derived switch point.  The
        // main+backup bundle is already committed atomically; forcing the
        // switch into a fixed number of replanning cycles makes a vehicle at
        // low speed brake before it can enter an obstacle detour.
        heu_ts = std::clamp(heu_ts, backup_switch_lower_bound, te);
        StatePVAJ switch_state;
        BackupBrakingSeed braking_seed;
        bool braking_seed_inside_sfc = false;

        // The geometric SFC is generated under the mission's exploratory
        // UNKNOWN policy, but a BACKUP suffix is certified under
        // KNOWN_FREE.  A minimum-snap seed can fit inside that SFC while its
        // swept tube still bends into an UNKNOWN cell.  Check the actual seed
        // at each candidate switch time before accepting the switch; moving
        // the switch earlier gives the vehicle more braking distance and
        // preserves a known-free recovery suffix without weakening the
        // backup contract.
        const auto record_known_free_validation = [this](
                const SweptValidationResult& validation,
                const navigation_planning::BackupCertificateRejectStage reject_stage) {
            auto& diagnostics = backup_certificate_diagnostics_;
            ++diagnostics.known_free_check_count;
            if (validation.valid) {
                ++diagnostics.known_free_pass_count;
                return true;
            }
            diagnostics.last_reject_stage = static_cast<int>(reject_stage);
            diagnostics.last_known_free_failure_code = static_cast<int>(validation.failure);
            diagnostics.last_known_free_cell_state =
                static_cast<int>(validation.blocked_cell_state);
            diagnostics.last_known_free_blocked_role =
                static_cast<int>(validation.blocked_role);
            diagnostics.last_known_free_first_blocked_time_s = validation.first_blocked_tt;
            diagnostics.last_known_free_blocked_position = validation.blocked_position;
            return false;
        };
        const auto minimum_snap_backup_validation =
                [this, &ref_exp_traj](const double candidate_ts,
                                      const double duration,
                                      const geometry_utils::Piece &backup_piece) {
            if (solve_cancelled_.load() || !std::isfinite(candidate_ts) ||
                !std::isfinite(duration) || duration <= 0.0) {
                return SweptValidationResult{};
            }
            const double backup_start_wall_time =
                    ref_exp_traj.getStartWallTime() + candidate_ts;
            if (!std::isfinite(backup_start_wall_time) ||
                backup_start_wall_time <= 0.0) {
                return SweptValidationResult{};
            }
            CandidateCommandBundle backup_candidate;
            if (!backup_piece.getCoeffMat().allFinite() ||
                std::abs(backup_piece.getDuration() - duration) > 1.0e-9) {
                return SweptValidationResult{};
            }
            backup_candidate.position.emplace_back(backup_piece);
            backup_candidate.position.start_WT = backup_start_wall_time;
            backup_candidate.start_wall_time = backup_start_wall_time;
            backup_candidate.roles.push_back(
                    {0.0, duration, CandidateTrajectoryRole::BACKUP});
            const auto validation = validateExecutableCandidate(
                    *map_ptr_, backup_candidate, planner_context_->getSimTime(),
                    navigation_world_model::UnknownPolicy::kRequireKnownFree);
            return validation;
        };
        const double initial_switch_guess = heu_ts;
        const double backup_altitude_target_m = planning_goal_p_.z();
        geometry_utils::Piece selected_braking_piece;
        bool selected_braking_piece_ready = false;
        const auto build_candidate_backup_sfc =
            [this, &solve_deadline](const StatePVAJ &state,
                                    const BackupBrakingSeed &seed,
                                    Polytope &candidate_poly) {
            if (!seed.feasible || !state.allFinite() || !seed.endpoint.allFinite()) {
                return false;
            }
            Line braking_line{state.col(0), seed.endpoint};
            if (!braking_line.first.allFinite() || !braking_line.second.allFinite() ||
                (braking_line.second - braking_line.first).norm() <= cfg_.resolution) {
                return false;
            }
            if (!cg_ptr_->GeneratePolytopeFromLine(
                    braking_line, candidate_poly, &solve_deadline)) {
                return false;
            }
            Eigen::Vector3d inner;
            if (!geometry_utils::findInterior(candidate_poly.GetPlanes(), inner)) {
                return false;
            }
            if (cfg_.use_fov_cut &&
                !fov_checker_->cutPolyByFov(
                    solve_state_.p, solve_state_.q, seed.endpoint, candidate_poly)) {
                return false;
            }
            if (cfg_.sensing_horizon_m > 0 &&
                !fov_checker_->cutPolyBySensingHorizon(
                    solve_state_.p, seed.endpoint, cfg_.sensing_horizon_m,
                    candidate_poly)) {
                return false;
            }
            return true;
        };
        // The latest visibility-derived switch is desirable for progress, but
        // its braking endpoint may lie outside the generated safety corridor.
        // Search backward on the EXP trajectory until the complete Bezier
        // hull of a dynamically feasible stop is contained by that corridor.
        // This retains the latest certifiable switch instead of either
        // disabling backup or imposing an unrelated fixed replan horizon.
        for (double candidate_ts = heu_ts;;) {
            auto& certificate_diagnostics = backup_certificate_diagnostics_;
            ++certificate_diagnostics.switch_candidate_count;
            Polytope candidate_sfc = visibility_poly;
            bool candidate_aligned_sfc = false;
            switch_state = ref_exp_traj.posTraj().getState(candidate_ts);
            const double backup_altitude_target =
                cfg_.preserve_backup_altitude &&
                        std::isfinite(backup_altitude_target_m)
                    ? backup_altitude_target_m
                    : std::numeric_limits<double>::quiet_NaN();
            braking_seed = makeBackupBrakingSeedWithTerminalAltitude(
                candidate_ts, switch_state,
                    cfg_.back_traj_cfg.max_vel, cfg_.back_traj_cfg.max_acc,
                    cfg_.back_traj_cfg.max_jerk, cfg_.sample_traj_dt_s,
                    0.0, backup_altitude_target);
            geometry_utils::Piece candidate_braking_piece;
            if (braking_seed.feasible &&
                std::isfinite(braking_seed.duration_s) &&
                braking_seed.duration_s > 0.0) {
                candidate_braking_piece = braking_seed.terminal_altitude_preserved
                    ? minimumSnapStopPieceWithTerminalAltitude(
                          switch_state, braking_seed.duration_s,
                          backup_altitude_target_m)
                    : minimumSnapStopPiece(switch_state, braking_seed.duration_s);
            }
            certificate_diagnostics.last_seed_switch_time_s = candidate_ts;
            certificate_diagnostics.last_seed_duration_s = braking_seed.duration_s;
            certificate_diagnostics.last_seed_initial_velocity_mps =
                braking_seed.initial_velocity_mps;
            certificate_diagnostics.last_seed_max_velocity_mps =
                braking_seed.maximum_velocity_mps;
            certificate_diagnostics.last_seed_max_acceleration_mps2 =
                braking_seed.maximum_acceleration_mps2;
            certificate_diagnostics.last_seed_max_jerk_mps3 =
                braking_seed.maximum_jerk_mps3;
            certificate_diagnostics.last_seed_endpoint = braking_seed.endpoint.cast<double>();
            braking_seed_inside_sfc = braking_seed.feasible &&
                    braking_seed.duration_s > cfg_.sample_traj_dt_s;
            if (!braking_seed_inside_sfc) {
                certificate_diagnostics.last_reject_stage = static_cast<int>(
                    navigation_planning::BackupCertificateRejectStage::kSeed);
            } else {
                ++certificate_diagnostics.feasible_seed_count;
            }
            if (braking_seed_inside_sfc) {
                const auto braking_control_points = minimumSnapStopBezierControlPoints(
                    candidate_braking_piece);
                for (int i = 0;
                     braking_seed_inside_sfc && i < braking_control_points.cols(); ++i) {
                    braking_seed_inside_sfc =
                            candidate_sfc.PointIsInside(braking_control_points.col(i));
                }
                if (braking_seed_inside_sfc) {
                    ++certificate_diagnostics.visibility_hull_pass_count;
                } else {
                    certificate_diagnostics.last_reject_stage = static_cast<int>(
                        navigation_planning::BackupCertificateRejectStage::kVisibilityHull);
                }
            }
            // The visibility SFC is built from the command origin to the EXP
            // visibility seed.  It is not the authority for a braking curve
            // whose endpoint and curvature are selected later.  If that
            // legacy corridor rejects the complete hull, rebuild one from the
            // actual switch state to the certified braking endpoint.  The
            // candidate still has to pass the strict KNOWN_FREE swept check
            // and the final full-bundle authorization below.
            if (!braking_seed_inside_sfc) {
                if (build_candidate_backup_sfc(
                        switch_state, braking_seed, candidate_sfc)) {
                    ++certificate_diagnostics.aligned_sfc_built_count;
                    const auto braking_control_points =
                        minimumSnapStopBezierControlPoints(
                            candidate_braking_piece);
                    braking_seed_inside_sfc = true;
                    for (int i = 0;
                         braking_seed_inside_sfc && i < braking_control_points.cols(); ++i) {
                        braking_seed_inside_sfc =
                            candidate_sfc.PointIsInside(braking_control_points.col(i));
                    }
                    if (braking_seed_inside_sfc) {
                        ++certificate_diagnostics.aligned_hull_pass_count;
                        candidate_aligned_sfc = true;
                    } else {
                        certificate_diagnostics.last_reject_stage = static_cast<int>(
                            navigation_planning::BackupCertificateRejectStage::kAlignedHull);
                    }
                } else {
                    certificate_diagnostics.last_reject_stage = static_cast<int>(
                        navigation_planning::BackupCertificateRejectStage::kAlignedSfc);
                }
            }
            if (braking_seed_inside_sfc) {
                const auto validation = minimum_snap_backup_validation(
                    candidate_ts, braking_seed.duration_s, candidate_braking_piece);
                braking_seed_inside_sfc = record_known_free_validation(
                    validation,
                    navigation_planning::BackupCertificateRejectStage::kKnownFree);
            }
            if (braking_seed_inside_sfc) {
                temp_poly = candidate_sfc;
                certificate_diagnostics.selected = true;
                if (candidate_aligned_sfc) {
                    planner_context_->info(
                        " -- [planner] selected braking-hull-aligned backup SFC "
                        "switch_t={} duration={} endpoint=({}, {}, {})",
                        candidate_ts, braking_seed.duration_s,
                        braking_seed.endpoint.x(), braking_seed.endpoint.y(),
                        braking_seed.endpoint.z());
                }
                heu_ts = candidate_ts;
                selected_braking_piece = candidate_braking_piece;
                selected_braking_piece_ready = true;
                break;
            }
            if (candidate_ts <= backup_switch_lower_bound + 1.0e-9) break;
            candidate_ts = std::max(
                    backup_switch_lower_bound,
                    candidate_ts - cfg_.sample_traj_dt_s);
        }
        const auto& certificate_diagnostics = backup_certificate_diagnostics_;
        if (!braking_seed_inside_sfc) {
            planner_context_->warn(
                " -- [planner] no dynamically feasible KNOWN_FREE minimum-snap "
                "backup hull inside SFC candidates={} feasible_seeds={} "
                "visibility_hull_pass={} aligned_sfc_built={} aligned_hull_pass={} "
                "known_free_checks={} known_free_pass={} last_reject_stage={} "
                "last_known_free_failure={} last_known_free_cell={} "
                "last_known_free_blocked_role={} last_known_free_tt={} "
                "last_known_free_position=({}, {}, {}) "
                "last_seed_switch={} last_seed_duration={} last_seed_v={} "
                "last_seed_a={} last_seed_j={} last_seed_endpoint=({}, {}, {})",
                certificate_diagnostics.switch_candidate_count,
                certificate_diagnostics.feasible_seed_count,
                certificate_diagnostics.visibility_hull_pass_count,
                certificate_diagnostics.aligned_sfc_built_count,
                certificate_diagnostics.aligned_hull_pass_count,
                certificate_diagnostics.known_free_check_count,
                certificate_diagnostics.known_free_pass_count,
                navigation_planning::backupCertificateRejectStageName(
                    static_cast<navigation_planning::BackupCertificateRejectStage>(
                        certificate_diagnostics.last_reject_stage)),
                certificate_diagnostics.last_known_free_failure_code,
                certificate_diagnostics.last_known_free_cell_state,
                certificate_diagnostics.last_known_free_blocked_role,
                certificate_diagnostics.last_known_free_first_blocked_time_s,
                certificate_diagnostics.last_known_free_blocked_position.x(),
                certificate_diagnostics.last_known_free_blocked_position.y(),
                certificate_diagnostics.last_known_free_blocked_position.z(),
                certificate_diagnostics.last_seed_switch_time_s,
                certificate_diagnostics.last_seed_duration_s,
                certificate_diagnostics.last_seed_max_velocity_mps,
                certificate_diagnostics.last_seed_max_acceleration_mps2,
                certificate_diagnostics.last_seed_max_jerk_mps3,
                certificate_diagnostics.last_seed_endpoint.x(),
                certificate_diagnostics.last_seed_endpoint.y(),
                certificate_diagnostics.last_seed_endpoint.z());
            return OPT_FAILED;
        }
        if (!selected_braking_piece_ready) {
            planner_context_->error(
                " -- [planner] backup certificate selected without a braking polynomial");
            return OPT_FAILED;
        }
        // The selected candidate SFC is the geometric contract for the backup
        // optimizer.  Keep it attached to BackupTraj before refinement and
        // before exporting the warm-start condition; otherwise the optimizer
        // receives its default empty polytope and rejects every valid seed as
        // "Invalid corridor planes".
        back_traj_info.setSFC(temp_poly);
        if (cfg_.print_log && initial_switch_guess - heu_ts > cfg_.sample_traj_dt_s * 0.5) {
            planner_context_->info(
                    " -- [planner] moved backup switch backward from {} to {} for certified hull",
                    initial_switch_guess, heu_ts);
        }
        double heu_dur = braking_seed.duration_s;
        // Keep the optimized switch close to the derived braking state. A
        // small interval avoids mapping an exact interval endpoint to
        // infinity while preventing the split-time reward from drifting back
        // toward the visibility boundary.
        const double backup_switch_upper_bound = std::min(
                te, heu_ts + std::max(0.01, cfg_.replan_forward_dt_s * 0.25));
        time_consuming_[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
        TimeConsuming t_back_opt("t_back_opt", false);
        double opt_ts = heu_ts;
        Trajectory temp_pos_traj;
        VecDf seed_times(cfg_.back_traj_cfg.piece_num);
        seed_times.setConstant(heu_dur / cfg_.back_traj_cfg.piece_num);
        vec_Vec3f seed_points;
        seed_points.reserve(cfg_.back_traj_cfg.piece_num);
        const auto &braking_piece = selected_braking_piece;
        for (int i = 1; i <= cfg_.back_traj_cfg.piece_num; ++i) {
            seed_points.emplace_back(braking_piece.getPos(
                    heu_dur * static_cast<double>(i) /
                    cfg_.back_traj_cfg.piece_num));
        }
        bool temp_ret = false;
        if (cfg_.backup_refinement_enabled) {
            back_traj_opt_->setSolveBudget(
                    &solve_cancelled_, solve_deadline.steadyDeadlineNanoseconds());
            temp_ret = back_traj_opt_->optimize(ref_exp_traj.posTraj(),
                                                backup_switch_lower_bound,
                                                backup_switch_upper_bound,
                                                heu_ts,
                                                back_traj_info.getSFC(),
                                                seed_times,
                                                seed_points,
                                                temp_pos_traj,
                                                opt_ts);
        }
        time_consuming_[BACK_TRAJ_OPT] = t_back_opt.stop();

        if (cfg_.backup_refinement_enabled) {
            double init_ts;
            VecDf init_times;
            vec_Vec3f init_ps;
            back_traj_opt_->getInitValue(init_ts, init_times, init_ps);
            latest_replan.setBackupCondition(init_ts, init_times, init_ps,
                                             backup_switch_lower_bound,
                                             backup_switch_upper_bound,
                                             back_traj_info.getSFC());
        }

        if (!temp_ret) {
            // The seed has already passed exact PVAJ boundary checks,
            // analytic velocity/acceleration/jerk extrema, and full Bezier
            // hull containment. L-BFGS is an optional refinement, not an
            // authority to discard that certified safety trajectory.
            ++backup_refinement_fallback_count_;
            planner_context_->warn(
                    " -- [planner] backup refinement failed; using certified minimum-snap seed "
                    "backup_refinement_success={} backup_refinement_fallback={}",
                    backup_refinement_success_count_, backup_refinement_fallback_count_);
            temp_pos_traj.clear();
            temp_pos_traj.emplace_back(braking_piece);
            opt_ts = heu_ts;
        } else {
            // Backup is an executable safety suffix, not a second route
            // planner. A numerically successful refinement may remain inside
            // the SFC and pass all pointwise gates while bending away from
            // the already certified braking seed. Replacing the seed on every
            // hot replan then changes the stopping direction and can create a
            // low-speed lateral/altitude oscillation. Keep refinement only
            // when its normalized spatial trace stays close to that seed.
            const double refined_duration = temp_pos_traj.getTotalDuration();
            const double seed_duration = braking_piece.getDuration();
            const double shape_tolerance_m = std::max(0.5, 3.0 * cfg_.resolution);
            double maximum_seed_deviation_m = 0.0;
            bool shape_bounded = std::isfinite(refined_duration) &&
                    refined_duration > 0.0 && std::isfinite(seed_duration) &&
                    seed_duration > 0.0 && std::isfinite(shape_tolerance_m) &&
                    shape_tolerance_m > 0.0;
            constexpr int kShapeSamples = 32;
            for (int sample = 0; shape_bounded && sample <= kShapeSamples; ++sample) {
                const double alpha = static_cast<double>(sample) /
                        static_cast<double>(kShapeSamples);
                const Vec3f refined_point = temp_pos_traj.getPos(alpha * refined_duration);
                const Vec3f seed_point = braking_piece.getPos(alpha * seed_duration);
                const double deviation = (refined_point - seed_point).norm();
                if (!refined_point.allFinite() || !seed_point.allFinite() ||
                    !std::isfinite(deviation)) {
                    shape_bounded = false;
                    break;
                }
                maximum_seed_deviation_m = std::max(maximum_seed_deviation_m, deviation);
                if (maximum_seed_deviation_m > shape_tolerance_m) {
                    shape_bounded = false;
                    break;
                }
            }
            if (!shape_bounded) {
                ++backup_refinement_fallback_count_;
                planner_context_->warn(
                        " -- [planner] backup refinement rejected: seed-trace deviation={} "
                        "limit={}; using certified minimum-snap seed",
                        maximum_seed_deviation_m, shape_tolerance_m);
                temp_pos_traj.clear();
                temp_pos_traj.emplace_back(braking_piece);
                opt_ts = heu_ts;
            } else {
                ++backup_refinement_success_count_;
                planner_context_->info(
                        " -- [planner] backup refinement accepted: "
                        "seed_trace_deviation={} limit={} "
                        "backup_refinement_success={} backup_refinement_fallback={}",
                        maximum_seed_deviation_m, shape_tolerance_m,
                        backup_refinement_success_count_, backup_refinement_fallback_count_);
            }
        }
        const auto backupCandidateValidation = [this](
                const Trajectory& backup_position, const double backup_start_wall_time) {
            SweptValidationResult invalid_result;
            if (!map_ptr_ || backup_position.empty()) {
                invalid_result.failure = SweptValidationResult::Failure::kNonFiniteTrajectory;
                return invalid_result;
            }
            if (!std::isfinite(backup_start_wall_time) || backup_start_wall_time <= 0.0) {
                invalid_result.failure = SweptValidationResult::Failure::kInvalidTimeWindow;
                return invalid_result;
            }
            CandidateCommandBundle backup_candidate;
            backup_candidate.position = backup_position;
            backup_candidate.start_wall_time = backup_start_wall_time;
            const double duration = backup_position.getTotalDuration();
            if (!std::isfinite(duration) || duration <= 0.0) {
                invalid_result.failure = SweptValidationResult::Failure::kInvalidTimeWindow;
                return invalid_result;
            }
            backup_candidate.roles.push_back(
                    {0.0, duration, CandidateTrajectoryRole::BACKUP});
            return validateExecutableCandidate(
                    *map_ptr_, backup_candidate, planner_context_->getSimTime(),
                    navigation_world_model::UnknownPolicy::kRequireKnownFree);
        };
        // The backup optimizer is constrained by the geometric SFC, while
        // the execution certificate is stricter: every swept cell in the
        // backup tube must be KNOWN_FREE. A successful numerical refinement
        // can still bend through an UNKNOWN cell inside an allow-unknown
        // mission corridor. Reject that refinement locally and retain the
        // already checked minimum-snap braking seed; never let
        // authorizeAndStage discover this only after the full mission solve.
        auto backup_validation = backupCandidateValidation(
            temp_pos_traj, ref_exp_traj.getStartWallTime() + opt_ts);
        if (!record_known_free_validation(
                    backup_validation,
                    navigation_planning::BackupCertificateRejectStage::kRefinementKnownFree)) {
            if (temp_ret) {
                ++backup_refinement_fallback_count_;
                planner_context_->warn(
                        " -- [planner] backup refinement crossed UNKNOWN; "
                        "using certified minimum-snap seed "
                        "backup_refinement_success={} backup_refinement_fallback={}",
                        backup_refinement_success_count_, backup_refinement_fallback_count_);
                temp_pos_traj.clear();
                temp_pos_traj.emplace_back(braking_piece);
                opt_ts = heu_ts;
            }
            backup_validation = backupCandidateValidation(
                temp_pos_traj, ref_exp_traj.getStartWallTime() + opt_ts);
            if (!record_known_free_validation(
                        backup_validation,
                        navigation_planning::BackupCertificateRejectStage::kRefinementKnownFree)) {
                planner_context_->warn(
                        " -- [planner] minimum-snap backup seed is not KNOWN_FREE; "
                        "rejecting backup candidate failure={} cell={} role={} "
                        "blocked_tt={} blocked_position=({}, {}, {})",
                        sweptValidationFailureName(backup_validation.failure),
                        static_cast<int>(backup_validation.blocked_cell_state),
                        static_cast<int>(backup_validation.blocked_role),
                        backup_validation.first_blocked_tt,
                        backup_validation.blocked_position.x(),
                        backup_validation.blocked_position.y(),
                        backup_validation.blocked_position.z());
                return OPT_FAILED;
            }
        }
        Vec4f yaw_init_vec = ref_exp_traj.getYawState(opt_ts).row(0);
        Trajectory temp_yaw_traj;
        if (!yaw_traj_opt_->optimizeToTarget(
                yaw_init_vec, yaw_init_vec(0), temp_pos_traj,
                temp_yaw_traj)) {
            const auto& yaw = yaw_traj_opt_->lastDiagnostics();
            // Backup yaw normally holds the current route yaw. When the yaw
            // optimizer cannot fit that hold because it must first remove a
            // non-zero yaw rate, construct the same minimum-snap stop used by
            // the emergency boundary and extend its duration until the
            // existing yaw limits accept it. The backup remains a complete
            // atomic candidate and the combined flatness/world checks below
            // stay authoritative.
            StatePVAJ yaw_state = StatePVAJ::Zero();
            yaw_state(0, 0) = yaw_init_vec(0);
            yaw_state(0, 1) = yaw_init_vec(1);
            yaw_state(0, 2) = yaw_init_vec(2);
            yaw_state(0, 3) = yaw_init_vec(3);
            double yaw_stop_duration_s = temp_pos_traj.getTotalDuration();
            bool yaw_stop_certificate_valid = false;
            for (int attempt = 0; attempt < 24; ++attempt) {
                const auto yaw_piece = minimumSnapStopPiece(
                    yaw_state, yaw_stop_duration_s);
                const double yaw_max_rate = yaw_piece.getMaxVelRate();
                const double yaw_max_acceleration = yaw_piece.getMaxAccRate();
                if (yaw_piece.getCoeffMat().allFinite() &&
                    std::isfinite(yaw_max_rate) &&
                    std::isfinite(yaw_max_acceleration) &&
                    yaw_max_rate <= cfg_.yaw_rate_max_rad_s + 1.0e-6 &&
                    yaw_max_acceleration <=
                        cfg_.yaw_acceleration_max_rad_s2 + 1.0e-6) {
                    temp_yaw_traj.clear();
                    temp_yaw_traj.emplace_back(yaw_piece);
                    yaw_stop_certificate_valid = true;
                    break;
                }
                yaw_stop_duration_s *= 1.15;
                if (!std::isfinite(yaw_stop_duration_s) ||
                    yaw_stop_duration_s <= 0.0) {
                    break;
                }
            }
            if (!yaw_stop_certificate_valid) {
                planner_context_->error(
                    " -- [planner] in [generateBackupTrajectory] YawTrajOpt failed "
                    "and certified yaw stop fallback failed reason={} duration={} "
                    "init=[{},{},{}] target={} delta={} full_max=[{},{}] "
                    "hold_max=[{},{}] stop=[{},{},{}] limits=[{},{}]",
                    static_cast<int>(yaw.failure), yaw.duration_s,
                    yaw.initial_state(0), yaw.initial_state(1), yaw.initial_state(2),
                    yaw.target_yaw_rad, yaw.requested_delta_rad,
                    yaw.full_turn_max_rate_rad_s,
                    yaw.full_turn_max_acceleration_rad_s2,
                    yaw.hold_max_rate_rad_s,
                    yaw.hold_max_acceleration_rad_s2,
                    yaw.stopping_displacement_rad,
                    yaw.stopping_max_rate_rad_s,
                    yaw.stopping_max_acceleration_rad_s2,
                    cfg_.yaw_rate_max_rad_s, cfg_.yaw_acceleration_max_rad_s2);
                return OPT_FAILED;
            }
            planner_context_->warn(
                " -- [planner] YawTrajOpt failed; using certified minimum-snap yaw stop "
                "duration={} reason={}",
                yaw_stop_duration_s, static_cast<int>(yaw.failure));
        }
        traj_opt::TrajectoryDynamicReport backup_dynamic_report;
        if (!traj_opt::trajectorySatisfiesFlatnessEnvelope(
                temp_pos_traj, cfg_.back_traj_cfg, &backup_dynamic_report,
                0.01, &temp_yaw_traj)) {
            planner_context_->error(
                    " -- [planner] combined backup position/yaw body-rate or thrust gate failed: "
                    "body_rate={} thrust=[{},{}]",
                    backup_dynamic_report.maximum_body_rate_rad_s,
                    backup_dynamic_report.minimum_thrust_n,
                    backup_dynamic_report.maximum_thrust_n);
            return OPT_FAILED;
        }


        if (!std::isfinite(opt_ts) ||
            opt_ts + 1.0e-9 < backup_switch_lower_bound ||
            opt_ts > te + 1.0e-9) {
            planner_context_->error(
                    " -- [planner] opt_ts {} outside certified backup switch window "
                    "[{}, {}]",
                    opt_ts, backup_switch_lower_bound, te);
            return OPT_FAILED;
        }
        double new_ts_WT = ref_exp_traj.getStartWallTime() + opt_ts;
        // The switch estimate is allowed to move earlier between hot replans.
        // An earlier switch is the more conservative backup choice; rejecting
        // it here can deadlock recovery after a map revision has invalidated
        // the old execution bundle while the backend still owns its history.
        // The new complete main+backup candidate is still checked by
        // authorizeAndStage() against the newest immutable world before it can
        // replace the committed command.


        {
            TimeConsuming t_viz("tviz", false);
            planner_context_->vizBackupTraj(temp_pos_traj);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        back_traj_info.setTrajectory(new_ts_WT, opt_ts, temp_pos_traj, temp_yaw_traj);
        latest_replan.setBackupTraj(temp_pos_traj);
        latest_replan.setBackupYawTraj(temp_yaw_traj);
        return SUCCESS;
    }

    int Planner::getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt) {
        if (goals.size() == 1) {
            return 0;
        }
        Vec3f a = start_pt, b;
        int min_id = 0;
        double min_dis = 1e10;
        for (long unsigned int i = 0; i < goals.size() - 1; i++) {
            b = goals[i];
            double dis = geometry_utils::pointLineSegmentDistance(start_pt, a, b);
            if (dis < min_dis) {
                min_dis = dis;
                min_id = i;
            }
            a = b;
        }
        return min_id;
    }

    bool
    Planner::PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                             const double &searching_horizon,
                             vec_Vec3f &path,
                             const AbsoluteDeadline &solve_deadline,
                             const bool allow_partial_route) {
        using namespace path_search;
        if (searching_horizon <= 0.0) {
            planner_context_->error(" -- [planner] Goal waypoints empty or searching horizon negative, force return.");
            return false;
        }
        if (!map_ptr_) {
            planner_context_->error(" -- [planner] route support check has no world model");
            return false;
        }

        // ROG-Map storage is an axis-aligned ENU AABB.  Do not let A* project
        // a long route onto the nearest map face and report that shortened
        // path as if the requested route had enough support. The requirement
        // is bounded by the requested route distance and the safety horizon.
        // Ordinary goal searches fail closed when the current AABB cannot
        // support the requested route. A pass-through look-ahead may opt into
        // an explicitly partial frontier: A* can return the certified map
        // boundary, and the caller must record incompleteness and lower the
        // terminal speed from that prefix. The two-cell margin matches the
        // inward endpoint margin in A* below.
        const Eigen::Vector3d route_delta =
            goal.cast<double>() - start_pt.cast<double>();
        const double route_distance = route_delta.norm();
        if (std::isfinite(route_distance) && route_distance > 1.0e-6) {
            const auto support = navigation_world_model::directionalSupportToLocalBoundary(
                start_pt.cast<double>(), route_delta, map_ptr_->geometry());
            const double required_support =
                std::min({route_distance, searching_horizon, cfg_.visibility_horizon_m}) +
                2.0 * cfg_.resolution;
            const double minimum_partial_support = 2.0 * cfg_.resolution;
            const bool support_is_partial = support.has_value() &&
                std::isfinite(*support) &&
                *support > minimum_partial_support + 1.0e-9;
            if (!support.has_value() || !std::isfinite(required_support) ||
                (*support + 1.0e-9 < required_support &&
                 (!allow_partial_route || !support_is_partial))) {
                planner_context_->error(
                    " -- [planner] route direction lacks AABB support: "
                    "available={:.3f} required={:.3f} partial_allowed={} "
                    "route_distance={:.3f} "
                    "start=({:.3f},{:.3f},{:.3f}) goal=({:.3f},{:.3f},{:.3f})",
                    support.value_or(0.0), required_support, allow_partial_route,
                    route_distance,
                    start_pt.x(), start_pt.y(), start_pt.z(), goal.x(), goal.y(), goal.z());
                return false;
            }
            if (*support + 1.0e-9 < required_support) {
                planner_context_->warn(
                    " -- [planner] using partial route frontier: "
                    "available={:.3f} required={:.3f} route_distance={:.3f}",
                    *support, required_support, route_distance);
            }
        }

        // 1) check and shift pts
        // 		For start point, must be collision free
        const auto start_type = map_ptr_->classify(
                start_pt, navigation_world_model::GridLayer::kEvidence);

        /// If the start_pt is obstacle in prob map, just shift it to the nearest free point.
        if (!navigation_world_model::isCellTraversable(start_type, unknownPolicy())) {
            planner_context_->warn(
                    " -- [planner] The start point in obstacle, this should not happen since the start point should be shift before pathsearch.");
            return false;
        }

        // Escape, preferred-altitude, unrestricted, and probability-map A*
        // are alternatives within one search stage.  Start one absolute
        // budget before the first alternative so fallback attempts cannot
        // multiply the callback latency when simulation time is stalled.
        const double stage_budget = std::min(
                cfg_.astar_total_time_limit_s,
                solve_deadline.conservativeRemaining(planner_context_->getSimTime()));
        if (stage_budget <= 0.0) {
            planner_context_->warn(" -- [Astar] solve deadline exhausted before path search");
            return false;
        }
        const AbsoluteDeadline search_deadline(
                planner_context_->getSimTime(), stage_budget);
        const int unknown_space_flag =
            unknownPolicy() == navigation_world_model::UnknownPolicy::kRequireKnownFree
                ? UNKNOWN_AS_OCCUPIED
                : UNKNOWN_AS_FREE;
        const auto remaining_search_budget = [&]() {
            return std::min(search_deadline.remaining(planner_context_->getSimTime()),
                            solve_deadline.conservativeRemaining(planner_context_->getSimTime()));
        };
        vec_E<Vec3f> start_point_escape_path;

        int flag_es = ON_PROB_MAP | unknown_space_flag;
        vec_Vec3f out_path;
        RET_CODE ret_es = astar_ptr_->escapePathSearch(
                start_pt, flag_es, out_path, true, remaining_search_budget());
        if (ret_es != NO_NEED && ret_es != REACH_HORIZON &&
            ret_es != REACH_GOAL && ret_es != INIT_ERROR) {
            planner_context_->warn(
                    " -- [Astar] Preferred-altitude escape failed with [{}]; "
                    "retry unrestricted 3-D escape.", RET_CODE_STR[ret_es].c_str());
            if (remaining_search_budget() > 0.0) {
                ret_es = astar_ptr_->escapePathSearch(
                        start_pt, flag_es, out_path, false, remaining_search_budget());
            }
        }
        if (ret_es != NO_NEED) {
            if (ret_es != REACH_HORIZON && ret_es != REACH_GOAL) {
                planner_context_->error(
                        " -- [planner] Escape path search failed with [{}], force return.",
                        RET_CODE_STR[ret_es].c_str());
                return false;
            } else {
                start_point_escape_path = out_path;
            }
        }

        Vec3f shifted_start_pt = start_pt;

        if (!start_point_escape_path.empty()) {
            shifted_start_pt = start_point_escape_path.back();
        }

        if (remaining_search_budget() <= 0.0) {
            planner_context_->warn(" -- [Astar] solve deadline exhausted before point-to-point search");
            return false;
        }

        Vec3f temp_goal_point, temp_start_point;
        temp_start_point = shifted_start_pt;
        double temp_plannning_horizon = searching_horizon;
        //            int start_id = getNearestFurtherGoalPoint(goal_waypoints, start_pt);

        int flag = ON_INF_MAP | unknown_space_flag | DONT_USE_INF_NEIGHBOR;

        RET_CODE ret_code = astar_ptr_->pointToPointPathSearch(
                temp_start_point, goal, flag, temp_plannning_horizon,
                path, std::min(cfg_.astar_search_time_limit_s,
                               remaining_search_budget()), true);

        if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL &&
            ret_code != INIT_ERROR &&
            remaining_search_budget() > 0.0) {
            planner_context_->warn(
                    " -- [Astar] Preferred start-goal altitude search failed with [{}]; "
                    "retry unrestricted 3-D search.", RET_CODE_STR[ret_code].c_str());
            ret_code = astar_ptr_->pointToPointPathSearch(
                    temp_start_point, goal, flag, temp_plannning_horizon,
                    path, remaining_search_budget(), false);
        }

        if(ret_code == INIT_ERROR){
            gi_.goal_valid = false;
            return false;
        }
        //add may23, if failed on inf map, use prob map try again

        if (ret_code == NO_PATH &&
            remaining_search_budget() > 0.0) {
            flag = ON_PROB_MAP | unknown_space_flag |
                   USE_INF_NEIGHBOR;
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map, try again on prob map.\n");
            ret_code = astar_ptr_->pointToPointPathSearch(
                    temp_start_point, goal, flag, temp_plannning_horizon,
                    path, remaining_search_budget(), true);
            if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL &&
                ret_code != INIT_ERROR &&
                remaining_search_budget() > 0.0) {
                ret_code = astar_ptr_->pointToPointPathSearch(
                        temp_start_point, goal, flag, temp_plannning_horizon,
                        path, remaining_search_budget(), false);
            }
            if (ret_code == SUCCESS || ret_code == REACH_HORIZON || ret_code == REACH_GOAL) {
                fmt::print(fg(fmt::color::lime_green) | fmt::emphasis::bold,
                           " -- [Astar] Path search on prob map success.\n");
            } else {
                fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                           " -- [Astar] Path search failed on prob map still failed.\n");
            }
        }
        if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL) {
            planner_context_->error(
                    " -- [planner] Path search failed with [{}], force return.\n", RET_CODE_STR[ret_code].c_str());
            return false;
        }

        // Prefer a mission-altitude route when the same A* lateral detour is
        // collision-free at the requested route altitude. The old projection
        // interpolated from the measured start altitude to the goal altitude;
        // after a tracking/recovery event that made every later nominal route
        // inherit the previous altitude error. The certified projection below
        // inserts an explicit climb first, then keeps the route horizontal at
        // the mission altitude. Preserve the original 3-D route when either
        // the climb or the horizontal projection is not traversable: a real
        // vertical avoidance route must remain available.
        if (path.size() >= 2) {
            const Vec3f route_delta = goal - shifted_start_pt;
            const double horizontal_distance = route_delta.head<2>().norm();
            if (horizontal_distance > cfg_.resolution) {
                vec_Vec3f constant_altitude_path;
                const auto segment_is_traversable = [this](
                        const Vec3f& begin, const Vec3f& end) {
                    return map_ptr_->isSegmentTraversable(
                            begin, end,
                            navigation_world_model::GridLayer::kInflated,
                            unknownPolicy());
                };
                if (buildConstantAltitudeRoute(
                        path, goal.z(), cfg_.resolution * 0.25,
                        segment_is_traversable, constant_altitude_path)) {
                    path = std::move(constant_altitude_path);
                    planner_context_->info(
                        " -- [Astar] selected certified constant mission-altitude route "
                        "altitude={} start_altitude={} points={}",
                        goal.z(), shifted_start_pt.z(), path.size());
                } else {
                    // Retain the prior start-to-goal profile as a conservative
                    // fallback for maps where the explicit climb is blocked.
                    const Eigen::Vector2d horizontal_direction =
                            route_delta.head<2>() / horizontal_distance;
                    vec_Vec3f altitude_preserving_path = path;
                    for (auto &point : altitude_preserving_path) {
                        const double progress = std::clamp(
                                (point.head<2>() - shifted_start_pt.head<2>())
                                        .dot(horizontal_direction) / horizontal_distance,
                                0.0, 1.0);
                        point.z() = shifted_start_pt.z() + progress * route_delta.z();
                    }
                    bool altitude_projection_free = true;
                    for (std::size_t index = 1;
                         index < altitude_preserving_path.size(); ++index) {
                        if (!map_ptr_->isSegmentTraversable(
                                altitude_preserving_path[index - 1],
                                altitude_preserving_path[index],
                                navigation_world_model::GridLayer::kInflated,
                                unknownPolicy())) {
                            altitude_projection_free = false;
                            break;
                        }
                    }
                    if (altitude_projection_free) {
                        path = std::move(altitude_preserving_path);
                        planner_context_->info(
                            " -- [Astar] retained start-to-goal altitude profile; "
                            "constant mission-altitude route was not certified "
                            "target_altitude={} start_altitude={}",
                            goal.z(), shifted_start_pt.z());
                    } else {
                        planner_context_->info(
                            " -- [Astar] retained certified 3-D altitude route; "
                            "neither constant nor interpolated projection was free "
                            "target_altitude={} start_altitude={}",
                            goal.z(), shifted_start_pt.z());
                    }
                }
            }
        }
        if (!start_point_escape_path.empty()) {
            path.insert(path.begin(), start_point_escape_path.begin(),
                        start_point_escape_path.end());
        }

        // Keep the route inside the exact metric box used by corridor
        // generation. ROG's circular-buffer insideLocalMap predicate is index
        // based and can accept a point beyond this box near a sliding edge,
        // which leaves MINCO's tail outside every generated polytope.
        const auto world_geometry = map_ptr_->geometry();
        const Vec3f corridor_center = world_geometry.local_center_m;
        const Vec3f corridor_half_size = 0.5 * world_geometry.local_size_m;
        const Vec3f corridor_margin = Vec3f::Constant(
                2.0 * world_geometry.inflated_resolution_m);
        const Vec3f corridor_min = corridor_center - corridor_half_size + corridor_margin;
        const Vec3f corridor_max = corridor_center + corridor_half_size - corridor_margin;
        const auto inside_corridor_map = [&](const Vec3f &point) {
            return (point.array() >= corridor_min.array()).all() &&
                   (point.array() <= corridor_max.array()).all();
        };
        bool trimmed_to_corridor_map = false;
        while (!path.empty() && !inside_corridor_map(path.back())) {
            path.pop_back();
            trimmed_to_corridor_map = true;
        }

        if (path.empty()) {
            planner_context_->warn(
                    " -- [planner] Path search failed with empty segments, force return.");
            return false;
        }

        // A* returns a grid route whose one-cell turns are safe but needlessly
        // inject high-frequency curvature into the MINCO guide.  Collapse only
        // shortcuts that pass the same continuous inflated-map oracle used by
        // the final certificate.  The bounded look-ahead keeps this
        // optimization linear in route length and avoids a quadratic raycast
        // tail on long local paths.
        const auto continuous_edge_is_safe = [this](
                const Vec3f& start, const Vec3f& end) {
            return map_ptr_->isSegmentTraversable(
                start, end, navigation_world_model::GridLayer::kInflated,
                unknownPolicy());
        };
        const std::size_t original_path_size = path.size();
        constexpr std::size_t kShortcutLookAheadPoints = 24U;
        const std::size_t protected_prefix_size = std::min(
            start_point_escape_path.size(), path.size());
        vec_Vec3f simplified_path;
        simplified_path.reserve(path.size());
        simplified_path.insert(
            simplified_path.end(), path.begin(),
            path.begin() + static_cast<std::ptrdiff_t>(protected_prefix_size));
        std::size_t anchor = protected_prefix_size == 0U
            ? 0U
            : protected_prefix_size - 1U;
        if (simplified_path.empty()) {
            simplified_path.push_back(path.front());
        }
        while (anchor + 1U < path.size()) {
            const std::size_t last_candidate = std::min(
                path.size() - 1U, anchor + kShortcutLookAheadPoints);
            std::size_t selected = anchor + 1U;
            for (std::size_t candidate = last_candidate;
                 candidate > anchor + 1U; --candidate) {
                const double shortcut_length =
                    (path[candidate] - path[anchor]).norm();
                if (shortcut_length <= cfg_.corridor_segment_max_length_m &&
                    continuous_edge_is_safe(path[anchor], path[candidate])) {
                    selected = candidate;
                    break;
                }
            }
            simplified_path.push_back(path[selected]);
            anchor = selected;
        }
        path = std::move(simplified_path);
        if (cfg_.print_log && path.size() < original_path_size) {
            planner_context_->info(
                " -- [Astar] continuous shortcut reduced guide points {} -> {}",
                original_path_size, path.size());
        }

        // A REACH_GOAL result may already contain the goal, but the explicit
        // endpoint is still needed when the search terminates at a nearby
        // graph representative.  Add it before the single final validation
        // pass; previously this edge was appended after validation.
        if (ret_code == REACH_GOAL && !trimmed_to_corridor_map &&
            inside_corridor_map(goal) &&
            (path.back() - goal).norm() > 1.0e-6) {
            path.push_back(goal);
        }
        path.insert(path.begin(), start_pt);

        // A point-to-point fallback may already return the measured start as
        // its first point.  Prepending the authoritative measured start above
        // would then manufacture a zero-length edge (start -> start).  That
        // edge is not a physical route segment, but the continuous oracle
        // quite correctly rejects it as a non-traversable segment.  Remove
        // only adjacent finite duplicates; every non-zero edge remains in the
        // path and is still checked by the invariant below.
        constexpr double kDuplicatePathPointToleranceM = 1.0e-6;
        const std::size_t path_size_before_deduplication = path.size();
        vec_Vec3f deduplicated_path;
        deduplicated_path.reserve(path.size());
        for (const Vec3f &point : path) {
            if (!deduplicated_path.empty() && point.allFinite() &&
                deduplicated_path.back().allFinite() &&
                (point - deduplicated_path.back()).norm() <=
                    kDuplicatePathPointToleranceM) {
                continue;
            }
            deduplicated_path.push_back(point);
        }
        path = std::move(deduplicated_path);
        if (cfg_.print_log && path.size() < path_size_before_deduplication) {
            planner_context_->info(
                " -- [Astar] removed {} adjacent duplicate path point(s)",
                path_size_before_deduplication - path.size());
        }

        // A* and corridor generation must consume the same continuous
        // traversability contract. A grid-connected edge that fails this
        // check is an invariant failure, not a safe executable prefix: trim
        // would hide a world-revision, quantization, or oracle mismatch and
        // can create repeated replanning churn.
        for (std::size_t index = 1; index < path.size(); ++index) {
            if (continuous_edge_is_safe(path[index - 1], path[index])) continue;
            planner_context_->warn(
                    " -- [planner] A* path contains a blocked continuous edge "
                    "index={} from {} to {}; invariant failure",
                    index, path[index - 1].transpose(), path[index].transpose());
            return false;
        }
        return true;
    }


    bool Planner::setState(const navigation_planning::KinematicState &state) {
        if (!state.finite()) {
            return false;
        }
        const double acceleration_limit = std::min(
            cfg_.exp_traj_cfg.max_acc, cfg_.back_traj_cfg.max_acc);
        const double jerk_limit = std::min(
            cfg_.exp_traj_cfg.max_jerk, cfg_.back_traj_cfg.max_jerk);
        const auto bounded_acceleration = boundEstimatedDerivative(
            state.acceleration_world, state.acceleration_estimated, acceleration_limit);
        const auto bounded_jerk = boundEstimatedDerivative(
            state.jerk_world, state.jerk_estimated, jerk_limit);
        const bool acceleration_bounded =
            !bounded_acceleration.isApprox(state.acceleration_world, 0.0);
        const bool jerk_bounded = !bounded_jerk.isApprox(state.jerk_world, 0.0);
        if ((acceleration_bounded || jerk_bounded) && !estimated_boundary_warning_emitted_) {
            planner_context_->warn(
                " -- [planner] bounded propagated derivative estimates: "
                "acceleration={}/{} jerk={}/{}; raw estimates remain in runtime diagnostics",
                state.acceleration_world.norm(), acceleration_limit,
                state.jerk_world.norm(), jerk_limit);
            estimated_boundary_warning_emitted_ = true;
        }
        navigation_math::RobotState internal;
        internal.p = state.position_world;
        internal.v = state.velocity_world;
        internal.a = bounded_acceleration;
        internal.j = bounded_jerk;
        const double quaternion_scale =
            state.orientation_world_body.coeffs().cwiseAbs().maxCoeff();
        if (!std::isfinite(quaternion_scale) || quaternion_scale <= 1.0e-9) {
            return false;
        }
        internal.q = navigation_math::Quatf(
            static_cast<navigation_math::decimal_t>(state.orientation_world_body.w() /
                                                    quaternion_scale),
            static_cast<navigation_math::decimal_t>(state.orientation_world_body.x() /
                                                    quaternion_scale),
            static_cast<navigation_math::decimal_t>(state.orientation_world_body.y() /
                                                    quaternion_scale),
            static_cast<navigation_math::decimal_t>(state.orientation_world_body.z() /
                                                    quaternion_scale));
        internal.q.normalize();
        if (!internal.q.coeffs().allFinite()) return false;
        internal.yaw = state.yaw_rad;
        internal.rcv_time = static_cast<double>(state.source_stamp_ns) * 1.0e-9;
        internal.rcv = true;
        std::lock_guard<std::mutex> guard(drone_state_mutex_);
        robot_state_ = internal;
        robot_acceleration_estimated_ = state.acceleration_estimated;
        robot_jerk_estimated_ = state.jerk_estimated;
        return true;
    }
}
