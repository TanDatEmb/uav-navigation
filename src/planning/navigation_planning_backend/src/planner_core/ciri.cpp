/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#include <planner_core/ciri.h>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace color_text;
using namespace optimization_utils;
using namespace navigation_math;


namespace navigation_planning_backend {

    RET_CODE CIRI::convexDecomposition(const Eigen::MatrixX4d& boundary_planes, const Eigen::Matrix3Xd& pc, const Eigen::Vector3d& a,
                                       const Eigen::Vector3d& b,
                                       const AbsoluteDeadline* deadline) {
        const auto deadlineExpired = [deadline]() noexcept {
            return deadline != nullptr && deadline->steadyExpired();
        };
        if (deadlineExpired()) return TIME_OUT;
        if (boundary_planes.rows() <= 0 || pc.cols() <= 0 || !boundary_planes.allFinite() || !pc.allFinite() ||
            !a.allFinite() || !b.allFinite()) {
            return INIT_ERROR;
        }
        const Eigen::Vector3d seed_delta = b - a;
        const double seed_scale = std::max({1.0, a.squaredNorm(), b.squaredNorm()});
        const double seed_degeneracy_limit =
            64.0 * std::numeric_limits<double>::epsilon() * seed_scale;
        if (!std::isfinite(seed_degeneracy_limit) ||
            !std::isfinite(seed_delta.squaredNorm()) ||
            seed_delta.squaredNorm() <= seed_degeneracy_limit) {
            return INIT_ERROR;
        }
        const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
        const Eigen::Vector4d bh(b(0), b(1), b(2), 1.0);

        /// force return if the seed is not inside the boundary
        if ((boundary_planes * ah).maxCoeff() > epsilon_ ||
            (boundary_planes * bh).maxCoeff() > epsilon_) {
//            cout << YELLOW << " -- [WARN] ah, bh not in BD, forced return." << endl;
//            cout << "boundary_planes * ah: " << (boundary_planes * ah).transpose().maxCoeff() << endl;
//            cout << "boundary_planes * bh: " << (boundary_planes * bh).transpose().maxCoeff() << endl;
            return INIT_ERROR;
        }


        /// Maximum M boundary constraints and N point constraints
        const int M = boundary_planes.rows();
        const int N = pc.cols();

        Ellipsoid E(Mat3f::Identity(), (a + b) / 2);
        if ((a - b).norm() > 0.1) {
            /// use line seed
            findEllipsoid(pc, a, b, E);
        }

        vector<Eigen::Vector4d> planes;
        MatD4f hPoly;

//        bool infeasible_problem{false};
        Vec3f infeasible_pt_w;

        for (int loop = 0; loop < iter_num_; ++loop) {
            if (deadlineExpired()) return TIME_OUT;
            // Initialize the boundary in ellipsoid frame
            const Eigen::Vector3d fwd_a = E.toEllipsoidFrame(a);
            const Eigen::Vector3d fwd_b = E.toEllipsoidFrame(b);
            const Eigen::MatrixX4d bd_e = E.toEllipsoidFrame(boundary_planes);
            const Eigen::VectorXd boundary_norms = bd_e.leftCols<3>().rowwise().norm();
            if (!boundary_norms.allFinite() ||
                (boundary_norms.array() <= std::numeric_limits<double>::epsilon()).any()) {
                return INIT_ERROR;
            }
            const Eigen::VectorXd distDs = bd_e.rightCols<1>().cwiseAbs().cwiseQuotient(
                    boundary_norms);
            const Eigen::Matrix3Xd pc_e = E.toEllipsoidFrame(pc);
            Eigen::VectorXd distRs = pc_e.colwise().norm();
            if (!distDs.allFinite() || !distRs.allFinite()) return INIT_ERROR;

            Eigen::Matrix<uint8_t, -1, 1> bdFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(M, 1);
            Eigen::Matrix<uint8_t, -1, 1> pcFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);

            bool completed = false;
            int bdMinId, pcMinId;
            double minSqrD = distDs.minCoeff(&bdMinId);
            double minSqrR = distRs.minCoeff(&pcMinId);

            Eigen::Vector4d temp_tangent, temp_plane_w;
            const Mat3f C_inv = E.C().inverse();

            planes.clear();
            planes.reserve(30);
            Vec3f tmp_nn_pt;
            Vec4f plan_before_ab;

            for (int i = 0; !completed && i < (M + N); ++i) {
                if (deadlineExpired()) return TIME_OUT;
                if (minSqrD < minSqrR) {
                    /// Case [Bd closer than ob]  enable the boundary constrain.
                    Vec4f p_e = bd_e.row(bdMinId);
                    temp_plane_w = E.toWorldFrame(p_e);
                    bdFlags(bdMinId) = 0;
                }
                else {
                    /// Case [Ob closer than Bd] enable the obstacle point constraint.
                    ///     Compute the tangent plane of sphere
                    ///
                    const auto & pt_w = pc.col(pcMinId);
                    const auto dis = distancePointToSegment(pt_w,a,b);
                    constexpr double kSeedClearanceToleranceM = 0.01;
                    const double minimum_clearance = std::max(
                        0.0, robot_r_ - kSeedClearanceToleranceM);
                    if(dis <= minimum_clearance) {
//                        infeasible_problem = true;
                        infeasible_pt_w = pt_w;
                        cout<<YELLOW<<" -- [CIRI] WARNING! The problem is not feasible, the min dis to obstacle is only: "<<dis<<RESET<<endl;
                        return FAILED;
                    }

                    if (robot_r_ < epsilon_) {
                        const Vec3f& pt_e = pc_e.col(pcMinId);
                        temp_tangent(3) = -distRs(pcMinId);
                        temp_tangent.head(3) = pt_e.transpose() / distRs(pcMinId);

                        if (temp_tangent.head(3).dot(fwd_a) + temp_tangent(3) > epsilon_) {
                            const Eigen::Vector3d delta = pc_e.col(pcMinId) - fwd_a;
                            const double delta_squared_norm = delta.squaredNorm();
                            if (!std::isfinite(delta_squared_norm) ||
                                delta_squared_norm <= std::numeric_limits<double>::epsilon()) {
                                return FAILED;
                            }
                            temp_tangent.head(3) = fwd_a -
                                (delta.dot(fwd_a) / delta_squared_norm) * delta;
                            distRs(pcMinId) = temp_tangent.head(3).norm();
                            if (!std::isfinite(distRs(pcMinId)) ||
                                distRs(pcMinId) <= std::numeric_limits<double>::epsilon()) {
                                return FAILED;
                            }
                            temp_tangent(3) = -distRs(pcMinId);
                            temp_tangent.head(3) /= distRs(pcMinId);
                        }
                        if (temp_tangent.head(3).dot(fwd_b) + temp_tangent(3) > epsilon_) {
                            const Eigen::Vector3d delta = pc_e.col(pcMinId) - fwd_b;
                            const double delta_squared_norm = delta.squaredNorm();
                            if (!std::isfinite(delta_squared_norm) ||
                                delta_squared_norm <= std::numeric_limits<double>::epsilon()) {
                                return FAILED;
                            }
                            temp_tangent.head(3) = fwd_b -
                                (delta.dot(fwd_b) / delta_squared_norm) * delta;
                            distRs(pcMinId) = temp_tangent.head(3).norm();
                            if (!std::isfinite(distRs(pcMinId)) ||
                                distRs(pcMinId) <= std::numeric_limits<double>::epsilon()) {
                                return FAILED;
                            }
                            temp_tangent(3) = -distRs(pcMinId);
                            temp_tangent.head(3) /= distRs(pcMinId);
                        }
                        temp_plane_w = E.toWorldFrame(temp_tangent);
                    }
                    else {
                        /// Case [Ob closer than Bd] enable the obstacle point constraint.
                        const Vec3f &pt_e = pc_e.col(pcMinId);
                        const Vec3f &pt_w = pc.col(pcMinId);
                        Ellipsoid E_pe(C_inv * sphere_template_.C(), pt_e);
                        Vec3f close_pt_e;
                        E_pe.pointDistanceToEllipsoid(Vec3f(0, 0, 0), close_pt_e);
                        Vec3f c_pt_w = E.toWorldFrame(close_pt_e);
                        temp_plane_w.head(3) = (pt_w - c_pt_w).normalized();
                        temp_plane_w(3) = -temp_plane_w.head(3).dot(c_pt_w);

                        /// Cut line with sphere A and B,
                        if (temp_plane_w.head(3).dot(a) + temp_plane_w(3) > -epsilon_) {
                            // Case the plan make seed out, the plane should be modified in world frame
                            if (!findTangentPlaneOfSphere(pt_w, robot_r_, a, E.d(), temp_plane_w)) {
                                return FAILED;
                            }
                        } else if (temp_plane_w.head(3).dot(b) + temp_plane_w(3) > -epsilon_) {
                            // Case the plan make seed out, the plane should be modified in world frame
                            if (!findTangentPlaneOfSphere(pt_w, robot_r_, b, E.d(), temp_plane_w)) {
                                return FAILED;
                            }
                        }
                    }
                    if (!temp_plane_w.allFinite()) return FAILED;
                    pcFlags(pcMinId) = 0;
                    tmp_nn_pt = pc.col(pcMinId);
                }
                // update pcMinId and bdMinId
                completed = true;
                minSqrD = INFINITY;
                for (int j = 0; j < M; ++j) {
                    if (bdFlags(j)) {
                        completed = false;
                        if (minSqrD > distDs(j)) {
                            bdMinId = j;
                            minSqrD = distDs(j);
                        }
                    }
                }
                minSqrR = INFINITY;
                for (int j = 0; j < N; ++j) {
                    if (pcFlags(j)) {
                        if ((temp_plane_w.head(3).dot(pc.col(j)) + temp_plane_w(3)) > robot_r_ - epsilon_) {
                            pcFlags(j) = 0;
                        }
                        else {
                            completed = false;
                            if (minSqrR > distRs(j)) {
                                pcMinId = j;
                                minSqrR = distRs(j);
                            }
                        }
                    }
                }
                planes.push_back(temp_plane_w);
            }

            hPoly.resize(planes.size(), 4);
            for (size_t i = 0; i < planes.size(); ++i) {
                hPoly.row(i) = planes[i];
            }

            if (loop == iter_num_ - 1) {
                break;
            }

            if(!hPoly.allFinite()) {
                cout << YELLOW << " -- [CIRI] ERROR! maxVolInsEllipsoid failed." << RESET << endl;
//                optimized_polytope_.Reset();
//                optimized_polytope_.SetPlanes(hPoly);
//                optimized_polytope_.SetSeedLine(std::make_pair(a, b));
//                optimized_polytope_.SetEllipsoid(E);
//                planner_context_->vizCiriSeedLine(a, b,robot_r_);
//                cout<<" -- [CIRI] infeasible_pt_w: "<<infeasible_pt_w.transpose()<<endl;
//                planner_context_->vizCiriInfeasiblePoint(infeasible_pt_w);
//                planner_context_->vizCiriEllipsoid(E);
////                optimized_polytope_.Visualize(debug_pub_, "optimized_polytope");
//                std::cout << " -- [CIRI] hPoly: " << hPoly << std::endl;
                return FAILED;
            }

            if (!MVIE::maxVolInsEllipsoid(hPoly, E)) {
                return FAILED;
            }
        }

        if (!hPoly.allFinite()) {
            cout << YELLOW << " -- [CIRI] ERROR! There is nan in generated planes." << RESET << endl;
            cout << a.transpose() << endl;
            cout << b.transpose() << endl;
            planner_context_->vizCiriSeedLine(a, b,robot_r_);
            planner_context_->vizCiriEllipsoid(E);
            return FAILED;
        }
        Vec3f inner;
        if (!geometry_utils::findInterior(hPoly, inner)) {
            cout<<RED<<" -- [CIRI] The polytope is empty."<<RESET<<endl;
            planner_context_->vizCiriSeedLine(a, b,robot_r_);
            planner_context_->vizCiriEllipsoid(E);
            return FAILED;
        }
        optimized_polytope_.Reset();
        optimized_polytope_.SetPlanes(hPoly);
        optimized_polytope_.SetSeedLine(std::make_pair(a, b));
        optimized_polytope_.SetEllipsoid(E);

        return SUCCESS;
    }

    void CIRI::getPolytope(Polytope& optimized_poly) {
        optimized_poly = optimized_polytope_;
    }

    void CIRI::setupParams(double robot_r, int iter_num) {
        if (!std::isfinite(robot_r) || robot_r <= 0.0 || iter_num <= 0) {
            throw std::invalid_argument(
                "CIRI requires a finite positive radius and iteration count");
        }
        robot_r_ = robot_r;
        iter_num_ = iter_num;
        sphere_template_ = Ellipsoid(Mat3f::Identity(), robot_r_ * Vec3f(1, 1, 1), Vec3f(0, 0, 0));
        //        split_seed_max_ = split_seed_max;
        //        split_thresh_ = split_thresh;
    }

    bool CIRI::findTangentPlaneOfSphere(const Eigen::Vector3d& center, const double& r,
                                        const Eigen::Vector3d& pass_point,
                                        const Eigen::Vector3d& seed_p,
                                        Eigen::Vector4d& outer_plane) {

        const Eigen::Vector3d point_from_center = pass_point - center;
        const double point_distance = point_from_center.norm();
        const double scale = std::max({1.0, point_distance, std::abs(r),
                                       (seed_p - center).norm()});
        const double tolerance = 128.0 * std::numeric_limits<double>::epsilon() * scale;
        if (!std::isfinite(r) || r <= 0.0 || !point_from_center.allFinite() ||
            !std::isfinite(point_distance) || point_distance <= r + tolerance) {
            return false;
        }

        // The tangent construction needs a plane through the pass point and
        // the seed direction.  Collinear inputs have no unique orientation;
        // choose the least-aligned coordinate axis instead of normalizing a
        // zero cross product.
        Eigen::Vector3d normal = point_from_center.cross(seed_p - center);
        if (!normal.allFinite() || normal.squaredNorm() <= tolerance * tolerance) {
            Eigen::Index least_aligned_axis = 0;
            point_from_center.cwiseAbs().minCoeff(&least_aligned_axis);
            Eigen::Vector3d reference_axis = Eigen::Vector3d::Zero();
            reference_axis(least_aligned_axis) = 1.0;
            normal = point_from_center.cross(reference_axis);
        }
        if (!normal.allFinite() || normal.squaredNorm() <= tolerance * tolerance) {
            return false;
        }
        normal.normalize();

        Eigen::Vector3d P = point_from_center;
        Eigen::Matrix3d R = Eigen::Quaterniond::FromTwoVectors(
                normal, Eigen::Vector3d::UnitZ()).matrix();
        P = R * P;
        Eigen::Vector3d C = R * (seed_p - center);
        Eigen::Vector3d Q;
        double r2 = r * r;
        double p1p2n = P.head(2).squaredNorm();
        const double tangent_radicand = p1p2n - r2;
        const double squared_tolerance = tolerance * tolerance;
        if (!std::isfinite(tangent_radicand) ||
            tangent_radicand <= squared_tolerance) {
            return false;
        }
        double d = sqrt(tangent_radicand);
        double rp1p2n = r / p1p2n;
        double q11 = rp1p2n * (P(0) * r - P(1) * d);
        double q21 = rp1p2n * (P(1) * r + P(0) * d);

        double q12 = rp1p2n * (P(0) * r + P(1) * d);
        double q22 = rp1p2n * (P(1) * r - P(0) * d);
        if (q11 * C(0) + q21 * C(1) < 0) {
            Q(0) = q12;
            Q(1) = q22;
        }
        else {
            Q(0) = q11;
            Q(1) = q21;
        }
        Q(2) = 0;
        // point(Q) + normal (AQ)
        outer_plane.head(3) = R.transpose() * Q;
        Q = outer_plane.head(3) + center;
        outer_plane(3) = -Q.dot(outer_plane.head(3));
        if (outer_plane.head(3).dot(seed_p) + outer_plane(3) > epsilon_) {
            outer_plane = -outer_plane;
        }
        return outer_plane.allFinite();
    }

    void CIRI::findEllipsoid(const Eigen::Matrix3Xd& pc,
                             const Eigen::Vector3d& a,
                             const Eigen::Vector3d& b,
                             Ellipsoid& out_ell) {
        double f = (a - b).norm() / 2;
        Mat3f C = f * Mat3f::Identity();
        Vec3f r = Vec3f::Constant(f);
        Vec3f center = (a + b) / 2;
        C(0, 0) += robot_r_;
        r(0) += robot_r_;
        if (r(0) > 0) {
            double ratio = r(1) / r(0);
            r *= ratio;
            C *= ratio;
        }

        Mat3f Ri = Eigen::Quaterniond::FromTwoVectors(Vec3f::UnitX(), (b - a)).toRotationMatrix();
        Ellipsoid E(Ri, r, center);
        Mat3f Rf = Ri;
        Mat3Df obs;
        int min_dis_id;
        Vec3f pw;
        if (E.pointsInside(pc, obs, min_dis_id)) {
            pw = obs.col(min_dis_id);
        }
        else {
            out_ell = E;
            return;
        }
        Mat3Df obs_inside = obs;
        int max_iter = 100;
        while (max_iter--) {
            Vec3f p_e = Ri.transpose() * (pw - E.d());
            const double roll = atan2(p_e(2), p_e(1));
            Rf = Ri * Eigen::Quaterniond(cos(roll / 2), sin(roll / 2), 0, 0);
            p_e = Rf.transpose() * (pw - E.d());
            if (p_e(0) < r(0)) {
                r(1) = std::abs(p_e(1)) / std::sqrt(1 - std::pow(p_e(0) / r(0), 2));
            }
            E = Ellipsoid(Rf, r, center);
            if (E.pointsInside(obs_inside, obs_inside, min_dis_id)) {
                pw = obs_inside.col(min_dis_id);
            }
            else {
                break;
            }
        }
        // Post-decrement makes an exhausted loop end at -1.  Checking for
        // exactly zero misses the only case in which all iterations ran.
        if (max_iter < 0) {
            cout << YELLOW << " -- [CIRI] Find Ellipsoid reach max iteration, may cause error." << endl;
        }
        max_iter = 100;


        if (E.pointsInside(obs, obs_inside, min_dis_id)) {
            pw = obs_inside.col(min_dis_id);
        }
        else {
            out_ell = E;
            return;
        }

        while (max_iter--) {
            Vec3f p = Rf.transpose() * (pw - E.d());
            double dd = 1 - std::pow(p(0) / r(0), 2) -
                        std::pow(p(1) / r(1), 2);
            if (dd > epsilon_) {
                r(2) = std::abs(p(2)) / std::sqrt(dd);
            }
            E = Ellipsoid(Rf, r, center);
            if (E.pointsInside(obs_inside, obs_inside, min_dis_id)) {
                pw = obs_inside.col(min_dis_id);
            }
            else {
                out_ell = E;
                break;
            }
        }

        if (max_iter < 0) {
            cout << YELLOW << " -- [CIRI] Find Ellipsoid reach max iteration, may cause error." << endl;
        }
        E = Ellipsoid(Rf, r, center);
        out_ell = E;
    }
}
