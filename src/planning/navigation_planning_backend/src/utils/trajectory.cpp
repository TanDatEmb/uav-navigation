/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#include <data_structure/base/trajectory.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace geometry_utils;
using namespace navigation_math;
using namespace color_text;
// Trasjectory==================================================

namespace {

Eigen::Vector3d invalidVector() {
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
}

Vec3f invalidVec3f() {
    return Vec3f::Constant(std::numeric_limits<double>::quiet_NaN());
}

Mat3Df invalidState() {
    Mat3Df state(3, 4);
    state.setConstant(std::numeric_limits<double>::quiet_NaN());
    return state;
}

bool validPiece(const Piece& piece) {
    const auto& coefficients = piece.getCoeffMat();
    return std::isfinite(piece.getDuration()) && piece.getDuration() > 0.0 &&
           coefficients.rows() == 3 && coefficients.cols() > 0 &&
           coefficients.allFinite();
}

bool locateSliceStart(const Trajectory& trajectory, const double time_s,
                      int& piece_index, double& local_time_s) {
    if (!std::isfinite(time_s) || time_s < 0.0 || trajectory.empty()) return false;
    double piece_begin = 0.0;
    for (int index = 0; index < trajectory.getPieceNum(); ++index) {
        const double piece_duration = trajectory[index].getDuration();
        if (!std::isfinite(piece_duration) || piece_duration <= 0.0) return false;
        const double piece_end = piece_begin + piece_duration;
        if (!std::isfinite(piece_end)) return false;
        // A slice starts in the next piece at an exact internal boundary. This
        // prevents a zero-duration prefix from being emitted.
        if (time_s < piece_end || index == trajectory.getPieceNum() - 1) {
            piece_index = index;
            local_time_s = std::clamp(time_s - piece_begin, 0.0, piece_duration);
            return true;
        }
        piece_begin = piece_end;
    }
    return false;
}

bool locateSliceEnd(const Trajectory& trajectory, const double time_s,
                    int& piece_index, double& local_time_s) {
    if (!std::isfinite(time_s) || time_s < 0.0 || trajectory.empty()) return false;
    double piece_begin = 0.0;
    for (int index = 0; index < trajectory.getPieceNum(); ++index) {
        const double piece_duration = trajectory[index].getDuration();
        if (!std::isfinite(piece_duration) || piece_duration <= 0.0) return false;
        const double piece_end = piece_begin + piece_duration;
        if (!std::isfinite(piece_end)) return false;
        // A slice ending at an exact boundary belongs to the piece just
        // completed, so its local endpoint is that piece's duration.
        if (time_s <= piece_end || index == trajectory.getPieceNum() - 1) {
            piece_index = index;
            local_time_s = std::clamp(time_s - piece_begin, 0.0, piece_duration);
            return true;
        }
        piece_begin = piece_end;
    }
    return false;
}

}  // namespace

Trajectory::Trajectory(const std::vector<double> &durs,
                       const std::vector<Eigen::MatrixXd> &cMats) {
    if (durs.size() != cMats.size()) {
        throw std::invalid_argument(
            "trajectory durations and coefficient matrices must have equal sizes");
    }
    const int N = static_cast<int>(durs.size());
    pieces.reserve(N);
    for (int i = 0; i < N; i++) {
        pieces.emplace_back(durs[i], cMats[i]);
    }
}

vec_Vec3f Trajectory::getWaypoints() const {
    vec_Vec3f pts;
    if (pieces.empty() ||
        !std::all_of(pieces.begin(), pieces.end(), validPiece)) {
        return pts;
    }
    for (size_t i = 0; i < pieces.size(); i++) {
        pts.push_back(pieces[i].getPos(0));
    }
    pts.push_back(getPos(getTotalDuration()));
    return pts;
}

bool Trajectory::empty() const  {
    return pieces.empty();
}

int Trajectory::getPieceNum() const {
    return pieces.size();
}

Eigen::VectorXd Trajectory::getDurations() const {
    int N = getPieceNum();
    Eigen::VectorXd durations(N);
    for (int i = 0; i < N; i++) {
        durations(i) = pieces[i].getDuration();
    }
    return durations;
}

double Trajectory::getTotalDuration() const {
    int N = getPieceNum();
    double totalDuration = 0.0;
    for (int i = 0; i < N; i++) {
        if (!validPiece(pieces[i])) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        totalDuration += pieces[i].getDuration();
        if (!std::isfinite(totalDuration)) {
            return std::numeric_limits<double>::quiet_NaN();
        }
    }
    return totalDuration;
}

void Trajectory::clear() {
    pieces.clear();
    return;
}

void Trajectory::reserve(const int &n) {
    pieces.reserve(n);
    return;
}

void Trajectory::emplace_back(const Piece &piece) {
    pieces.emplace_back(piece);
    return;
}

void Trajectory::emplace_back(const double &dur,
                              const Eigen::MatrixXd &cMat) {
    pieces.emplace_back(dur, cMat);
    return;
}

void Trajectory::append(const Trajectory &traj) {
    pieces.insert(pieces.end(), traj.begin(), traj.end());
    return;
}

int Trajectory::locatePieceIdx(double &t) const {
    const int N = getPieceNum();
    if (N == 0 || !std::isfinite(t) || t < 0.0) {
        return -1;
    }

    for (int idx = 0; idx < N; ++idx) {
        const Piece& piece = pieces[idx];
        if (!validPiece(piece)) {
            return -1;
        }
        const double duration = piece.getDuration();
        if (t <= duration) {
            return idx;
        }
        t -= duration;
    }

    // Preserve the established endpoint semantics while preventing
    // extrapolation beyond the final polynomial piece.
    t = pieces.back().getDuration();
    return N - 1;
}

double Trajectory::getWaypointTT(const int &waypoint_id) const {
    if (waypoint_id < 0 || waypoint_id >= getPieceNum()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double t = 0.0;
    for (int i = 0; i <= waypoint_id; i++) {
        if (!validPiece(pieces[i])) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        t += pieces[i].getDuration();
    }
    return t;
}

Eigen::Vector3d Trajectory::getPos(double t) const {

    int pieceIdx = locatePieceIdx(t);
    if (pieceIdx < 0) return invalidVector();
    return pieces[pieceIdx].getPos(t);
}

Eigen::Vector3d Trajectory::getVel(double t) const {
    int pieceIdx = locatePieceIdx(t);
    if (pieceIdx < 0) return invalidVector();
    return pieces[pieceIdx].getVel(t);
}

Eigen::Vector3d Trajectory::getAcc(double t) const {
    int pieceIdx = locatePieceIdx(t);
    if (pieceIdx < 0) return invalidVector();
    return pieces[pieceIdx].getAcc(t);
}

Eigen::Vector3d Trajectory::getJer(double t) const {
    int pieceIdx = locatePieceIdx(t);
    if (pieceIdx < 0) return invalidVector();
    return pieces[pieceIdx].getJer(t);
}


Vec3f Trajectory::getSnap(double t) const {
    int pieceIdx = locatePieceIdx(t);
    if (pieceIdx < 0) return invalidVec3f();
    return pieces[pieceIdx].getSnap(t);
}

Mat3Df Trajectory::getState(double t) const {
    int pieceIdx = locatePieceIdx(t);
    if (pieceIdx < 0) return invalidState();
    return pieces[pieceIdx].getState(t);
}

bool Trajectory::getState(double t, StatePVAJ& out_state) const {
    const double dur = getTotalDuration();
    if (!std::isfinite(t) || t < 0.0 || !std::isfinite(dur) || dur <= 0.0) {
        return false;
    }
    t = std::min(t, dur);
    out_state.col(0) = getPos(t);
    out_state.col(1) = getVel(t);
    out_state.col(2) = getAcc(t);
    out_state.col(3) = getJer(t);
    return out_state.allFinite();
}


Eigen::Vector3d Trajectory::getJuncPos(int juncIdx) const {
    if (juncIdx < 0 || juncIdx > getPieceNum()) return invalidVector();
    if (juncIdx != getPieceNum() && validPiece(pieces[juncIdx])) {
        return pieces[juncIdx].getCoeffMat().col(pieces[juncIdx].getDegree());
    } else if (juncIdx == getPieceNum() && juncIdx > 0 && validPiece(pieces[juncIdx - 1])) {
        return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
    }
    return invalidVector();
}

Eigen::Vector3d Trajectory::getJuncVel(int juncIdx) const {
    if (juncIdx < 0 || juncIdx > getPieceNum()) return invalidVector();
    if (juncIdx != getPieceNum() && validPiece(pieces[juncIdx]) &&
        pieces[juncIdx].getDegree() >= 1) {
        return pieces[juncIdx].getCoeffMat().col(pieces[juncIdx].getDegree() - 1);
    } else if (juncIdx == getPieceNum() && juncIdx > 0 && validPiece(pieces[juncIdx - 1])) {
        return pieces[juncIdx - 1].getVel(pieces[juncIdx - 1].getDuration());
    }
    return invalidVector();
}

Eigen::Vector3d Trajectory::getJuncAcc(int juncIdx) const {
    if (juncIdx < 0 || juncIdx > getPieceNum()) return invalidVector();
    if (juncIdx != getPieceNum() && validPiece(pieces[juncIdx]) &&
        pieces[juncIdx].getDegree() >= 2) {
        return pieces[juncIdx].getCoeffMat().col(pieces[juncIdx].getDegree() - 2) * 2.0;
    } else if (juncIdx == getPieceNum() && juncIdx > 0 && validPiece(pieces[juncIdx - 1])) {
        return pieces[juncIdx - 1].getAcc(pieces[juncIdx - 1].getDuration());
    }
    return invalidVector();
}

bool Trajectory::getPartialTrajectoryByID(const int &start_id, const int &end_id,
                                          Trajectory &out_traj) const {
    out_traj.clear();
    const int piece_count = getPieceNum();
    int end_id_ = end_id;
    if (end_id_ == -1) {
        end_id_ = piece_count;
    }

    if (start_id < 0 || end_id_ > piece_count || start_id >= end_id_) {
        return false;
    }

    double start_time = 0.0;
    for (int i = 0; i < end_id_; ++i) {
        if (!validPiece(pieces[i])) return false;
        if (i < start_id) start_time += pieces[i].getDuration();
    }
    for (int i = start_id; i < end_id_; ++i) {
        out_traj.emplace_back(pieces[i]);
    }
    out_traj.start_WT = start_WT + start_time;
    return true;
}

bool Trajectory::getPartialTrajectoryByTime(const double &start_TT, const double &end_TT,
                                            Trajectory &out_traj) const {
    const double total_dur = getTotalDuration();
    out_traj.clear();
    if (!std::isfinite(total_dur) || total_dur <= 0.0 ||
        !std::isfinite(start_TT) || start_TT < 0.0 || start_TT >= total_dur ||
        !std::isfinite(end_TT) || end_TT <= 0.0 || end_TT > total_dur) {
        return false;
    }
    if (end_TT <= start_TT) {
        return false;
    }

    if (start_TT == 0) {
        // Only the final piece duration changes; its local polynomial remains
        // expressed from the original piece start.
        double end_local_t = 0.0;
        int pieceEndIdx = -1;
        if (!locateSliceEnd(*this, end_TT, pieceEndIdx, end_local_t)) {
            return false;
        }
        for (int i = 0; i < pieceEndIdx; ++i) out_traj.emplace_back(pieces[i]);
        Piece new_pie = pieces[pieceEndIdx];
        new_pie.setDuration(end_local_t);
        out_traj.emplace_back(new_pie);
        out_traj.start_WT = start_WT;
        return true;
    }

    // Get the start piece id
    double t0 = 0.0;
    double local_end_t = 0.0;
    int pieceIdx = -1;
    int pieceEndIdx = -1;
    if (!locateSliceStart(*this, start_TT, pieceIdx, t0) ||
        !locateSliceEnd(*this, end_TT, pieceEndIdx, local_end_t) ||
        pieceIdx > pieceEndIdx ||
        pieces[pieceIdx].getDegree() != pieces[pieceEndIdx].getDegree()) {
        return false;
    }
    if (pieces[pieceIdx].getDegree() == 5) {
        double t02 = t0 * t0;
        double t03 = t02 * t0;
        double t04 = t03 * t0;
        double t05 = t04 * t0;
        // Re-express the selected polynomial around the requested slice start.
        Eigen::MatrixXd coef_mat = pieces[pieceIdx].getCoeffMat();
        Eigen::Matrix<double, 6, 6> cvt_M;
        cvt_M << 1, 0, 0, 0, 0, 0,
                5 * t0, 1, 0, 0, 0, 0,
                10 * t02, 4 * t0, 1, 0, 0, 0,
                10 * t03, 6 * t02, 3 * t0, 1, 0, 0,
                5 * t04, 4 * t03, 3 * t02, 2 * t0, 1, 0,
                t05, 1 * t04, 1 * t03, t02, t0, 1;

        coef_mat = coef_mat * cvt_M.transpose();
        double p1_t = std::min(pieces[pieceIdx].getDuration() - t0, end_TT - start_TT);
        Piece new_pie(p1_t, coef_mat);
        out_traj.pieces.push_back(new_pie);
        if (pieceIdx == pieceEndIdx) {
            out_traj.start_WT = start_WT + start_TT;
            return true;
        }
        // Append the untouched interior pieces and a truncated final piece.
        for (int i = pieceIdx + 1; i < pieceEndIdx; i++) {
            out_traj.pieces.push_back(pieces[i]);
        }
        Eigen::MatrixXd end_coef = pieces[pieceEndIdx].getCoeffMat();
        Piece new_pie_end(local_end_t, end_coef);
        out_traj.pieces.push_back(new_pie_end);
        out_traj.start_WT = start_WT + start_TT;
        return true;
    } else if (pieces[pieceIdx].getDegree() == 7) {
        double t02 = t0 * t0;
        double t03 = t02 * t0;
        double t04 = t03 * t0;
        double t05 = t04 * t0;
        double t06 = t03 * t03;
        double t07 = t03 * t04;

        // Re-express the selected polynomial around the requested slice start.
        Eigen::MatrixXd coef_mat = pieces[pieceIdx].getCoeffMat();
        Eigen::Matrix<double, 8, 8> cvt_M;
        cvt_M << 1, 0, 0, 0, 0, 0, 0, 0,
                7 * t0, 1, 0, 0, 0, 0, 0, 0,
                21 * t02, 6 * t0, 1, 0, 0, 0, 0, 0,
                35 * t03, 15 * t02, 5 * t0, 1, 0, 0, 0, 0,
                35 * t04, 20 * t03, 10 * t02, 4 * t0, 1, 0, 0, 0,
                21 * t05, 15 * t04, 10 * t03, 6 * t02, 3 * t0, 1, 0, 0,
                7 * t06, 6 * t05, 5 * t04, 4 * t03, 3 * t02, 2 * t0, 1, 0,
                t07, t06, 1 * t05, t04, t03, t02, t0, 1;

        coef_mat = coef_mat * cvt_M.transpose();
        double p1_t = std::min(pieces[pieceIdx].getDuration() - t0, end_TT - start_TT);
        Piece new_pie(p1_t, coef_mat);
        out_traj.pieces.push_back(new_pie);
        if (pieceIdx == pieceEndIdx) {
            out_traj.start_WT = start_WT + start_TT;
            return true;
        }
        // Append the untouched interior pieces and a truncated final piece.
        for (int i = pieceIdx + 1; i < pieceEndIdx; i++) {
            out_traj.pieces.push_back(pieces[i]);
        }
        Eigen::MatrixXd end_coef = pieces[pieceEndIdx].getCoeffMat();
        Piece new_pie_end(local_end_t, end_coef);
        out_traj.pieces.push_back(new_pie_end);
        out_traj.start_WT = start_WT + start_TT;
        return true;

    } else {
        return false;
    }
}

double Trajectory::getMaxVelRate() const {
    int N = getPieceNum();
    double maxVelRate = -INFINITY;
    double tempNorm;
    for (int i = 0; i < N; i++) {
        tempNorm = pieces[i].getMaxVelRate();
        maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
    }
    return maxVelRate;
}

double Trajectory::getMaxAccRate() const {
    int N = getPieceNum();
    double maxAccRate = -INFINITY;
    double tempNorm;
    for (int i = 0; i < N; i++) {
        tempNorm = pieces[i].getMaxAccRate();
        maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
    }
    return maxAccRate;
}

double Trajectory::getMaxJerRate() const {
    double maxJerRate = -INFINITY;
    for (const auto& piece : pieces) {
        maxJerRate = std::max(maxJerRate, piece.getMaxJerRate());
    }
    return maxJerRate;
}

bool Trajectory::checkMaxVelRate(const double &maxVelRate) const {
    int N = getPieceNum();
    bool feasible = true;
    for (int i = 0; i < N && feasible; i++) {
        feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);
    }
    return feasible;
}

bool Trajectory::checkMaxAccRate(const double &maxAccRate) const {
    int N = getPieceNum();
    bool feasible = true;
    for (int i = 0; i < N && feasible; i++) {
        feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
    }
    return feasible;
}

void Trajectory::printProfile() const {
    std::cout << GREEN << "[Trajectory::printProfile] The trajectory has " << getPieceNum() << " pieces" << RESET
              << std::endl;
    std::cout << GREEN << "\tMaximum Velocity Rate: " << getMaxVelRate() << RESET << std::endl;
    std::cout << GREEN << "\tMaximum Acceleration Rate: " << getMaxAccRate() << RESET << std::endl;
    std::cout << GREEN << "\tTotal Duration: " << getTotalDuration() << RESET << std::endl;
}
