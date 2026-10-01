/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */


#ifndef EXP_TRAJ_H
#define EXP_TRAJ_H
#include <data_structure/base/trajectory.h>

#include <cmath>


namespace navigation_planning_backend {
    using geometry_utils::Trajectory;
    using geometry_utils::PolytopeVec;

    class ExpTraj{
        /* The optimized positional trajectory */
        Trajectory pos_traj_{};

        /* The optimized yaw trajectory */
        Trajectory yaw_traj_{};

        double start_WT_{0.0};

        /* The safe flight corridor */
        PolytopeVec sfc_{};

        /* some flags */
        bool flag_connected_goal_{false};
        bool flag_preserve_incoming_route_tangent_{false};
        bool flag_empty_{true};

        /* some part of exp traj may belong to last backup, record this */
        double on_backup_start_TT{-1}, on_backup_end_TT{-1};

        // A measured-state rebase may prepend a certified C3 handoff.  The
        // backup switch must not cut that prefix away when the candidate is
        // assembled; it remains nominal EXP until this duration has elapsed.
        double required_main_prefix_duration_TT_{0.0};

    public:
        explicit  ExpTraj() = default;

        void setEmpty() {
            flag_empty_ = true;
            flag_connected_goal_ = false;
            flag_preserve_incoming_route_tangent_ = false;
            required_main_prefix_duration_TT_ = 0.0;
        }

        bool empty() const {
            return flag_empty_;
        }

        bool connectedToGoal()const {
            return flag_connected_goal_;
        }

        bool preservesIncomingRouteTangent() const {
            return flag_preserve_incoming_route_tangent_;
        }

        size_t getSFCSize() const {
            return sfc_.size();
        }

        bool getFirstPartBackupTraj(double & on_backup_traj_start_TT,
            double & on_backup_traj_end_TT) const {
            if(on_backup_start_TT < 0 || on_backup_end_TT < 0 || on_backup_end_TT < on_backup_start_TT) {
                return false;
            }
            on_backup_traj_start_TT = on_backup_start_TT;
            on_backup_traj_end_TT = on_backup_end_TT;
            return true;
        }

        void setTrajectory(const double & start_WT,
            const Trajectory & pos_traj_in,
            const Trajectory & yaw_traj_in,
            const double & _on_backup_start_TT = -1,
            const double & _on_backup_end_TT = -1) {
            pos_traj_ = pos_traj_in;
            yaw_traj_ = yaw_traj_in;
            start_WT_ = start_WT;
            pos_traj_.start_WT = start_WT;
            yaw_traj_.start_WT = start_WT;
            flag_empty_ = false;
            on_backup_start_TT = _on_backup_start_TT;
            on_backup_end_TT = _on_backup_end_TT;
            required_main_prefix_duration_TT_ = 0.0;
        }

        bool setRequiredMainPrefixDuration(const double duration_TT) {
            if (!std::isfinite(duration_TT) || duration_TT < 0.0) {
                return false;
            }
            required_main_prefix_duration_TT_ = duration_TT;
            return true;
        }

        double getRequiredMainPrefixDuration() const {
            return required_main_prefix_duration_TT_;
        }

        double getTotalDuration() const {
            return pos_traj_.getTotalDuration();
        }

        Vec3f getPos(const double & t)const {
            return pos_traj_.getPos(t);
        }

        Vec3f getVel(const double & t)const {
            return pos_traj_.getVel(t);
        }

        StatePVAJ getYawState(const double &t)const {
            return yaw_traj_.getState(t);
        }


        void setSFC(const PolytopeVec & sfc) {
            sfc_ = sfc;
        }

        void setGoalConnectedFlag(const bool & _in) {
            flag_connected_goal_ = _in;
        }

        void setPreserveIncomingRouteTangentFlag(const bool preserve) {
            flag_preserve_incoming_route_tangent_ = preserve;
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
            Trajectory & partial_yaw_traj)  const {

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
