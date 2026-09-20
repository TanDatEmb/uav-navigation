/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */


#ifndef BACKUP_TRAJ_H
#define BACKUP_TRAJ_H

#include <data_structure/base/trajectory.h>


namespace navigation_planning_backend {
    using geometry_utils::Trajectory;
    using geometry_utils::PolytopeVec;

    class BackupTraj{
        /* The optimized positional trajectory */
        Trajectory pos_traj_{};

        /* The optimized yaw trajectory */
        Trajectory yaw_traj_{};

        double start_WT_{0.0};

        /* The start time on exp traj*/
        double start_TT_{0.0};

        /* The safe flight corridor */
        Polytope sfc_{};

        /* some flags */
        bool flag_empty_{true};

        /* Robot position during replan */
        Vec3f robot_p_;




    public:
        explicit  BackupTraj() = default;

        ~BackupTraj() = default;

        void setEmpty() {
            flag_empty_ = true;
            start_TT_ = -1;
        }

        bool empty() const {
            return flag_empty_;
        }

        void setRobotPos(const Vec3f & _p) {
            robot_p_ = _p;
        }

        Vec3f getRobotPos() const {
            return robot_p_;
        }

        double getStartTT() const {
            return start_TT_;
        }

        void setTrajectory(const double & start_WT,
            const double & start_TT,
            const Trajectory & pos_traj_in,
            const Trajectory & yaw_traj_in) {

            pos_traj_ = pos_traj_in;
            yaw_traj_ = yaw_traj_in;
            start_WT_ = start_WT;
            pos_traj_.start_WT = start_WT;
            yaw_traj_.start_WT = start_WT;
            start_TT_ = start_TT;
            flag_empty_ = false;
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


        void setSFC(const Polytope & sfc) {
            sfc_ = sfc;
            sfc_.SetKnownFree(true);
        }

        const Polytope & getSFC(){
            return sfc_;
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

#endif //BACKUP_TRAJ_H
