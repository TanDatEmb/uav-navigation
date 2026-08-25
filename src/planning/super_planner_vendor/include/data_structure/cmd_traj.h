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


#ifndef CMD_TRAJ_H
#define CMD_TRAJ_H

#include <data_structure/exp_traj.h>
#include <data_structure/backup_traj.h>
#include <data_structure/base/trajectory.h>
#include <navigation_world_model/world_model_view.hpp>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <mutex>
#include <optional>
#include <vector>


namespace super_planner {
    using geometry_utils::Trajectory;

    enum class CandidateTrajectoryRole : std::uint8_t { MAIN = 0, BACKUP = 1 };
    enum class BackupDisposition : std::uint8_t {
        SUCCESS = 0, FINISH = 1, NO_NEED = 2, EMERGENCY = 3
    };

    struct CandidateRoleInterval {
        double begin_tt{0.0};
        double end_tt{0.0};
        CandidateTrajectoryRole role{CandidateTrajectoryRole::MAIN};
    };

    struct CandidateCommandBundle {
        Trajectory position{};
        Trajectory yaw{};
        double start_wall_time{0.0};
        std::vector<CandidateRoleInterval> roles{};
        bool backup_suffix_available{false};
        double backup_start_tt{0.0};
        bool connected_goal{false};
        BackupDisposition backup_disposition{BackupDisposition::SUCCESS};
    };

    struct CommandCertificate {
        navigation_world_model::WorldSnapshotIdentity pinned_world{};
        navigation_world_model::WorldSnapshotIdentity validated_world{};
        double validation_begin_tt{0.0};
    };

    struct CommitDiagnostics {
        std::uint64_t generation{0};
        std::uint64_t previous_generation{0};
        double candidate_start_wall_time{0.0};
        StatePVAJ candidate_start_pvaj{StatePVAJ::Zero()};
        double candidate_start_yaw{0.0};
        double candidate_start_yaw_rate{0.0};
        bool previous_valid{false};
        double previous_sample_tt{0.0};
        StatePVAJ previous_pvaj{StatePVAJ::Zero()};
        double previous_yaw{0.0};
        double previous_yaw_rate{0.0};
        Vec3f position_residual{Vec3f::Zero()};
        Vec3f velocity_residual{Vec3f::Zero()};
        Vec3f acceleration_residual{Vec3f::Zero()};
        Vec3f jerk_residual{Vec3f::Zero()};
        double yaw_residual{0.0};
        double yaw_rate_residual{0.0};
    };

    struct CommittedTrajectorySnapshot {
        Trajectory position{};
        Trajectory yaw{};
        std::uint64_t generation{0};
        CommandCertificate certificate{};
        CommitDiagnostics diagnostics{};
        bool empty{true};
        bool backup_available{false};
        double backup_start_tt{0.0};
    };

    struct CommittedTrajectoryMetadata {
        std::uint64_t generation{0};
        CommitDiagnostics diagnostics{};
    };

#define LOCK_G std::lock_guard<std::mutex> lock(mtx_);

    class CmdTraj{
        /* The update and query lock */
        mutable std::mutex mtx_;

        /* The optimized positional trajectory */
        Trajectory pos_traj_{};

        /* The optimized yaw trajectory */
        Trajectory yaw_traj_{};

        double start_WT_{0.0};
        double backup_traj_start_TT_{0.0};
        /* some part of exp traj may belong to last backup, record this */
        double on_backup_start_TT_{-1}, on_backup_end_TT_{-1};
        bool first_part_exp_has_backup_traj_{false};


        /* some flags */
        bool flag_empty_{true};
        bool flag_backup_traj_avilibale_{false};
        std::uint64_t generation_{0};
        std::vector<CandidateRoleInterval> role_intervals_{};
        CommandCertificate certificate_{};
        CommitDiagnostics commit_diagnostics_{};

        static bool trajectoryFinite(const Trajectory &trajectory) {
            if (trajectory.empty() || !std::isfinite(trajectory.start_WT) ||
                !std::isfinite(trajectory.getTotalDuration()) ||
                trajectory.getTotalDuration() < 0.0) {
                return false;
            }
            constexpr std::size_t kValidationIntervals = 100U;
            for (std::size_t index = 0; index <= kValidationIntervals; ++index) {
                const double t = trajectory.getTotalDuration() *
                    static_cast<double>(index) / static_cast<double>(kValidationIntervals);
                if (!trajectory.getState(t).allFinite()) return false;
            }
            return true;
        }

        void checkFirstPartBackupTraj(const ExpTraj & exp) {
            double tmp_s, tmp_e;
            if(exp.getFirstPartBackupTraj(tmp_s, tmp_e)) {
                on_backup_end_TT_ = tmp_e;
                on_backup_start_TT_ = tmp_s;
                first_part_exp_has_backup_traj_ = true;
            }else {
                first_part_exp_has_backup_traj_ =false;
            }
        }

    public:
        explicit  CmdTraj() = default;

        void setEmpty() {
            flag_empty_ = true;
        }

        bool empty() const {
            return flag_empty_;
        }

        void lock() {
            mtx_.lock();
        }

        void unlock() {
            mtx_.unlock();
        }


        static std::optional<CandidateCommandBundle> buildCandidate(
            const ExpTraj& exp_traj, const BackupTraj* backup_traj,
            BackupDisposition disposition) {
            Trajectory tmp_pos_traj, tmp_yaw_traj;
            CandidateCommandBundle candidate;
            candidate.connected_goal = exp_traj.connectedToGoal();
            candidate.backup_disposition = disposition;
            if (backup_traj != nullptr) {
                const double backup_start = backup_traj->getStartTT();
                if (!std::isfinite(backup_start) || backup_start < 0.0 ||
                    !exp_traj.getPartialTrajectoryByTrajectoryTime(
                        0, backup_start, tmp_pos_traj, tmp_yaw_traj)) {
                    return std::nullopt;
                }
                candidate.position = tmp_pos_traj + backup_traj->posTraj();
                candidate.yaw = tmp_yaw_traj + backup_traj->yawTraj();
                candidate.backup_suffix_available = true;
                candidate.backup_start_tt = backup_start;
            } else {
                candidate.position = exp_traj.posTraj();
                candidate.yaw = exp_traj.yawTraj();
                candidate.backup_suffix_available = false;
                candidate.backup_start_tt = candidate.position.getTotalDuration();
            }
            if (!trajectoryFinite(candidate.position) || !trajectoryFinite(candidate.yaw) ||
                std::abs(candidate.position.start_WT - candidate.yaw.start_WT) > 1.0e-6) {
                return std::nullopt;
            }
            candidate.start_wall_time = candidate.position.start_WT;

            double inherited_start = -1.0;
            double inherited_end = -1.0;
            const bool inherited_backup =
                exp_traj.getFirstPartBackupTraj(inherited_start, inherited_end);

            const double duration = candidate.position.getTotalDuration();
            std::vector<CandidateRoleInterval> backup_intervals;
            if (inherited_backup) {
                const double upper = backup_traj
                    ? std::min(duration, candidate.backup_start_tt) : duration;
                const double clipped_begin = std::max(0.0, inherited_start);
                const double clipped_end = std::min(upper, inherited_end);
                if (clipped_end >= clipped_begin) {
                    backup_intervals.push_back(
                        {clipped_begin, clipped_end, CandidateTrajectoryRole::BACKUP});
                }
            }
            if (candidate.backup_suffix_available) {
                backup_intervals.push_back({candidate.backup_start_tt, duration,
                                            CandidateTrajectoryRole::BACKUP});
            }
            std::sort(backup_intervals.begin(), backup_intervals.end(),
                      [](const auto& lhs, const auto& rhs) {
                          return lhs.begin_tt < rhs.begin_tt;
                      });
            std::vector<CandidateRoleInterval> merged_backup;
            for (const auto& interval : backup_intervals) {
                if (!merged_backup.empty() &&
                    interval.begin_tt <= merged_backup.back().end_tt) {
                    merged_backup.back().end_tt = std::max(
                        merged_backup.back().end_tt, interval.end_tt);
                } else {
                    merged_backup.push_back(interval);
                }
            }
            double cursor = 0.0;
            for (const auto& interval : merged_backup) {
                if (interval.begin_tt > cursor) {
                    candidate.roles.push_back(
                        {cursor, interval.begin_tt, CandidateTrajectoryRole::MAIN});
                }
                candidate.roles.push_back(interval);
                cursor = interval.end_tt;
            }
            if (cursor < duration || candidate.roles.empty()) {
                candidate.roles.push_back(
                    {cursor, duration, CandidateTrajectoryRole::MAIN});
            }
            return candidate;
        }

        static std::optional<CandidateCommandBundle> buildEmergencyCandidate(
                const Trajectory& position, const Trajectory& yaw) {
            if (!trajectoryFinite(position) || !trajectoryFinite(yaw) ||
                std::abs(position.start_WT - yaw.start_WT) > 1.0e-6 ||
                yaw.getTotalDuration() + 1.0e-6 < position.getTotalDuration()) {
                return std::nullopt;
            }
            CandidateCommandBundle candidate;
            candidate.position = position;
            candidate.yaw = yaw;
            candidate.start_wall_time = position.start_WT;
            candidate.backup_suffix_available = true;
            candidate.backup_start_tt = 0.0;
            candidate.backup_disposition = BackupDisposition::EMERGENCY;
            candidate.roles.push_back(
                {0.0, position.getTotalDuration(), CandidateTrajectoryRole::BACKUP});
            return candidate;
        }

        bool commitCandidate(CandidateCommandBundle&& candidate,
                             const CommandCertificate& certificate) {
            LOCK_G
            // A candidate that starts before the current bundle would make
            // command trajectory time jump forward at the generation switch.
            // Reject transactionally; never rebase historical polynomials or
            // mutate the current bundle/history to conceal the regression.
            if (!flag_empty_ && candidate.start_wall_time < start_WT_) {
                return false;
            }
            CommitDiagnostics diagnostics;
            diagnostics.generation = generation_ + 1U;
            diagnostics.previous_generation = generation_;
            diagnostics.candidate_start_wall_time = candidate.start_wall_time;
            diagnostics.candidate_start_pvaj = candidate.position.getState(0.0);
            const auto candidate_yaw_state = candidate.yaw.getState(0.0);
            diagnostics.candidate_start_yaw = candidate_yaw_state(0, 0);
            diagnostics.candidate_start_yaw_rate = candidate_yaw_state(0, 1);
            if (!flag_empty_ && !pos_traj_.empty() && !yaw_traj_.empty()) {
                const double previous_duration = pos_traj_.getTotalDuration();
                diagnostics.previous_sample_tt = std::clamp(
                    candidate.start_wall_time - start_WT_, 0.0, previous_duration);
                diagnostics.previous_pvaj = pos_traj_.getState(
                    diagnostics.previous_sample_tt);
                const auto previous_yaw_state = yaw_traj_.getState(
                    diagnostics.previous_sample_tt);
                diagnostics.previous_yaw = previous_yaw_state(0, 0);
                diagnostics.previous_yaw_rate = previous_yaw_state(0, 1);
                diagnostics.previous_valid = diagnostics.previous_pvaj.allFinite() &&
                    previous_yaw_state.allFinite();
                if (diagnostics.previous_valid) {
                    diagnostics.position_residual =
                        diagnostics.candidate_start_pvaj.col(0) -
                        diagnostics.previous_pvaj.col(0);
                    diagnostics.velocity_residual =
                        diagnostics.candidate_start_pvaj.col(1) -
                        diagnostics.previous_pvaj.col(1);
                    diagnostics.acceleration_residual =
                        diagnostics.candidate_start_pvaj.col(2) -
                        diagnostics.previous_pvaj.col(2);
                    diagnostics.jerk_residual =
                        diagnostics.candidate_start_pvaj.col(3) -
                        diagnostics.previous_pvaj.col(3);
                    diagnostics.yaw_residual =
                        std::remainder(
                            diagnostics.candidate_start_yaw - diagnostics.previous_yaw,
                            2.0 * std::acos(-1.0));
                    diagnostics.yaw_rate_residual =
                        diagnostics.candidate_start_yaw_rate - diagnostics.previous_yaw_rate;
                }
            }
            pos_traj_ = std::move(candidate.position);
            yaw_traj_ = std::move(candidate.yaw);
            start_WT_ = pos_traj_.start_WT;
            flag_empty_ = false;
            flag_backup_traj_avilibale_ = candidate.backup_suffix_available;
            backup_traj_start_TT_ = candidate.backup_start_tt;
            role_intervals_ = std::move(candidate.roles);
            certificate_ = certificate;
            first_part_exp_has_backup_traj_ = false;
            on_backup_start_TT_ = on_backup_end_TT_ = -1.0;
            for (const auto& interval : role_intervals_) {
                if (interval.role == CandidateTrajectoryRole::BACKUP &&
                    (!flag_backup_traj_avilibale_ ||
                     interval.end_tt < backup_traj_start_TT_)) {
                    first_part_exp_has_backup_traj_ = true;
                    on_backup_start_TT_ = interval.begin_tt;
                    on_backup_end_TT_ = interval.end_tt;
                    break;
                }
            }
            ++generation_;
            commit_diagnostics_ = diagnostics;
            return true;
        }

        CommittedTrajectorySnapshot snapshot() const {
            LOCK_G
            CommittedTrajectorySnapshot snapshot;
            snapshot.position = pos_traj_;
            snapshot.yaw = yaw_traj_;
            snapshot.generation = generation_;
            snapshot.certificate = certificate_;
            snapshot.diagnostics = commit_diagnostics_;
            snapshot.empty = flag_empty_;
            snapshot.backup_available = flag_backup_traj_avilibale_;
            snapshot.backup_start_tt = backup_traj_start_TT_;
            return snapshot;
        }

        std::uint64_t generationSnapshot() const {
            LOCK_G
            return generation_;
        }

        CommittedTrajectoryMetadata metadataSnapshot() const {
            LOCK_G
            return {generation_, commit_diagnostics_};
        }

        bool setTrajectory(const ExpTraj&exp_traj,
            const BackupTraj & backup_traj) {
            auto candidate = buildCandidate(
                exp_traj, &backup_traj, BackupDisposition::SUCCESS);
            return candidate && commitCandidate(std::move(*candidate), {});
        }

        bool setTrajectory(const ExpTraj&exp_traj) {
            auto candidate = buildCandidate(
                exp_traj, nullptr, BackupDisposition::NO_NEED);
            return candidate && commitCandidate(std::move(*candidate), {});
        }

        // Commit a safety-only trajectory generated from the current measured
        // vehicle state.  Build and validate both polynomials before entering
        // this method; the lock makes the position/yaw pair and its BACKUP
        // ownership visible to the command thread as one atomic bundle.
        bool setEmergencyBackup(const Trajectory &pos_traj,
                                const Trajectory &yaw_traj) {
            auto candidate = buildEmergencyCandidate(pos_traj, yaw_traj);
            return candidate && commitCandidate(std::move(*candidate), {});
        }

        bool isTTOnBackupTraj(const double & checkTT) const {
            for (const auto& interval : role_intervals_) {
                if (interval.role == CandidateTrajectoryRole::BACKUP &&
                    checkTT >= interval.begin_tt && checkTT <= interval.end_tt) {
                    return true;
                }
            }
            return false;
        }

        bool backupTrajAvilibale() const {
            return flag_backup_traj_avilibale_;
        }


        double getTotalDuration() const {
            return pos_traj_.getTotalDuration();
        }

        double getBackupTrajStartTT() const {
            return backup_traj_start_TT_;
        }

        std::uint64_t generation() const { return generation_; }
        const CommandCertificate& certificate() const { return certificate_; }

        Vec3f getPos(const double & t)const {
            return pos_traj_.getPos(t);
        }

        Vec3f getVel(const double & t)const {
            return pos_traj_.getVel(t);
        }

        Vec3f getYaw(const double & t)const {
            return yaw_traj_.getPos(t);
        }

        Vec3f getYawRate(const double & t)const {
            return yaw_traj_.getVel(t);
        }

        StatePVAJ getYawState(const double &t)const {
            return yaw_traj_.getState(t);
        }

        const Trajectory & posTraj() const {
            return pos_traj_;
        }

        const Trajectory & yawTraj() const {
            return yaw_traj_;
        }


        const double & getStartWallTime() const {
            return pos_traj_.start_WT;
        }

        bool getPartialTrajectoryByTrajectoryTime(const double & start_t,
            const double & end_t,
            Trajectory & partial_pos_traj,
            Trajectory & partial_yaw_traj) {

            if(!pos_traj_.getPartialTrajectoryByTime(start_t,end_t,partial_pos_traj)) {
                return false;
            }

            if(!yaw_traj_.getPartialTrajectoryByTime(start_t, end_t,partial_yaw_traj)) {
                return false;
            }

            return true;
        }

    };
}

#endif //EXP_TRAJ_H
