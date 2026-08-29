/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#include <data_structure/base/ellipsoid.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <limits>

using namespace geometry_utils;

namespace {

constexpr float kMinimumAxis = 1.0e-7F;

bool validShape(const Mat3f& shape, const Vec3f& center) {
    if (!shape.allFinite() || !center.allFinite() ||
        !std::isfinite(shape.cwiseAbs().maxCoeff())) {
        return false;
    }
    const Eigen::SelfAdjointEigenSolver<Mat3f> solver(
        0.5 * (shape + shape.transpose()));
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
        (solver.eigenvalues().array() <= kMinimumAxis).any()) {
        return false;
    }
    return (shape - shape.transpose()).cwiseAbs().maxCoeff() <= 1.0e-5F;
}

bool validFrame(const Mat3f& rotation, const Vec3f& radii,
                const Vec3f& center) {
    if (!rotation.allFinite() || !radii.allFinite() || !center.allFinite() ||
        (radii.array() <= kMinimumAxis).any()) {
        return false;
    }
    const Mat3f orthogonality = rotation.transpose() * rotation;
    return orthogonality.allFinite() &&
           (orthogonality - Mat3f::Identity()).cwiseAbs().maxCoeff() <=
               1.0e-4F && std::isfinite(rotation.determinant()) &&
           std::abs(rotation.determinant()) > 1.0e-4F;
}

Vec3f invalidPoint() {
    return Vec3f::Constant(std::numeric_limits<double>::quiet_NaN());
}

}  // namespace

bool Ellipsoid::empty() const {
    return undefined;
}

Ellipsoid::Ellipsoid(const Mat3f &C, const Vec3f &d) : C_(C), d_(d) {
    if (!validShape(C_, d_)) {
        undefined = true;
        return;
    }
    C_inv_ = C_.inverse();
    if (!C_inv_.allFinite()) {
        undefined = true;
        return;
    }

    Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::FullPivHouseholderQRPreconditioner> svd(C_, Eigen::ComputeFullU);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Vector3d S = svd.singularValues();
    if (U.determinant() < 0.0) {
        R_.col(0) = U.col(1);
        R_.col(1) = U.col(0);
        R_.col(2) = U.col(2);
        r_(0) = S(1);
        r_(1) = S(0);
        r_(2) = S(2);
    } else {
        R_ = U;
        r_ = S;
    }
    undefined = !validFrame(R_, r_, d_);
}

Ellipsoid::Ellipsoid(const Mat3f &R, const Vec3f &r, const Vec3f &d)
        : R_(R), r_(r), d_(d) {
    if (!validFrame(R_, r_, d_)) {
        undefined = true;
        return;
    }
    C_ = R_ * r_.asDiagonal() * R_.transpose();
    C_inv_ = C_.inverse();
    undefined = !C_.allFinite() || !C_inv_.allFinite();
}

double Ellipsoid::pointDistaceToEllipsoid(const Vec3f &pt, Vec3f &closest_pt_on_ellip) const {
    if (empty() || !pt.allFinite()) {
        closest_pt_on_ellip = invalidPoint();
        return std::numeric_limits<double>::quiet_NaN();
    }
    /// step one: transform the point to the ellipsoid frame
    Vec3f pt_ellip_frame = R_.transpose() * (pt - d_);
    double dist = geometry_utils::DistancePointEllipsoid(r_(0), r_(1), r_(2),
                                                         pt_ellip_frame.x(),
                                                         pt_ellip_frame.y(),
                                                         pt_ellip_frame.z(),
                                                         closest_pt_on_ellip.x(),
                                                         closest_pt_on_ellip.y(),
                                                         closest_pt_on_ellip.z());
    /// step two: transform the closest point back to the world frame
    closest_pt_on_ellip = R_ * closest_pt_on_ellip + d_;
    if (!std::isfinite(dist) || !closest_pt_on_ellip.allFinite()) {
        closest_pt_on_ellip = invalidPoint();
        return std::numeric_limits<double>::quiet_NaN();
    }
    return dist;
}

int Ellipsoid::nearestPointId(const Eigen::Matrix3Xd &pc) const {
    if (empty() || pc.cols() <= 0 || !pc.allFinite()) return -1;
    Eigen::VectorXd dists = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
    if (!dists.allFinite()) return -1;
    int np_id = -1;
    dists.minCoeff(&np_id);
    return np_id;
}

Vec3f Ellipsoid::nearestPoint(const Eigen::Matrix3Xd &pc) const {
    if (empty() || pc.cols() <= 0 || !pc.allFinite()) return invalidPoint();
    Eigen::VectorXd dists = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
    if (!dists.allFinite()) return invalidPoint();
    int np_id = -1;
    dists.minCoeff(&np_id);
    return pc.col(np_id);
}

double Ellipsoid::nearestPointDis(const Eigen::Matrix3Xd &pc, int &np_id) const {
    np_id = -1;
    if (empty() || pc.cols() <= 0 || !pc.allFinite()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    Eigen::VectorXd dists = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
    if (!dists.allFinite()) return std::numeric_limits<double>::quiet_NaN();
    double np_dist = dists.minCoeff(&np_id);
    return np_dist;
}

Mat3f Ellipsoid::C() const {
    return C_;
}

Vec3f Ellipsoid::d() const {
    return d_;
}

Mat3f Ellipsoid::R() const {
    return R_;
}

Vec3f Ellipsoid::r() const {
    return r_;
}


Vec3f Ellipsoid::toEllipsoidFrame(const Vec3f &pt_w) const {
    return C_inv_ * (pt_w - d_);
}

Eigen::Matrix3Xd Ellipsoid::toEllipsoidFrame(const Eigen::Matrix3Xd &pc_w) const {
    return C_inv_ * (pc_w.colwise() - d_);
}

Vec3f Ellipsoid::toWorldFrame(const Vec3f &pt_e) const {
    return C_ * pt_e + d_;
}

Eigen::Matrix3Xd Ellipsoid::toWorldFrame(const Eigen::Matrix3Xd &pc_e) const {
    return (C_ * pc_e).colwise() + d_;
}

Eigen::Vector4d Ellipsoid::toEllipsoidFrame(const Eigen::Vector4d &plane_w) const {
    Eigen::Vector4d plane_e;
    plane_e.head(3) = plane_w.head(3).transpose() * C_;
    plane_e(3) = plane_w(3) + plane_w.head(3).dot(d_);
    return plane_e;
}

Eigen::Vector4d Ellipsoid::toWorldFrame(const Eigen::Vector4d &plane_e) const {
    Eigen::Vector4d plane_w;
    plane_w.head(3) = plane_e.head(3).transpose() * C_inv_;
    plane_w(3) = plane_e(3) - plane_w.head(3).dot(d_);
    return plane_w;
}

Eigen::MatrixX4d Ellipsoid::toEllipsoidFrame(const Eigen::MatrixX4d &planes_w) const {
    Eigen::MatrixX4d planes_e(planes_w.rows(), planes_w.cols());
    planes_e.leftCols(3) = planes_w.leftCols(3) * C_;
    planes_e.rightCols(1) = planes_w.rightCols(1) + planes_w.leftCols(3) * d_;
    return planes_e;
}

Eigen::MatrixX4d Ellipsoid::toWorldFrame(const Eigen::MatrixX4d &planes_e) const {
    Eigen::MatrixX4d planes_w(planes_e.rows(), planes_e.cols());
    planes_w.leftCols(3) = planes_e.leftCols(3) * C_inv_;
    planes_w.rightCols(1) = planes_e.rightCols(1) - planes_w.leftCols(3) * d_;
    return planes_w;
}

double Ellipsoid::dist(const Vec3f &pt_w) const {
    return (C_inv_ * (pt_w - d_)).norm();
}

Eigen::VectorXd Ellipsoid::dist(const Eigen::Matrix3Xd &pc_w) const {
    return (C_inv_ * (pc_w.colwise() - d_)).colwise().norm();
}

bool Ellipsoid::noPointsInside(vec_Vec3f &pc, const Eigen::Matrix3d &R, const Vec3f &r, const Vec3f &p) const {
    if (!R.allFinite() || !r.allFinite() || !p.allFinite() ||
        (r.array() <= kMinimumAxis).any()) {
        return false;
    }
    Eigen::Matrix3d C_inv;
    C_inv = r.cwiseInverse().asDiagonal() * R.transpose();
    for (auto pt_w: pc) {
        if (!pt_w.allFinite()) return false;
        double d = (C_inv * (pt_w - p)).norm();
        if (!std::isfinite(d)) return false;
        if (d <= 1) {
            return false;
        }
    }
    return true;
}

bool Ellipsoid::pointsInside(const Eigen::Matrix3Xd &pc, Mat3Df &out, int &min_pt_id) const {
    out.resize(3, 0);
    min_pt_id = -1;
    if (empty() || pc.cols() <= 0 || !pc.allFinite()) return false;
    Eigen::VectorXd vec = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
    if (!vec.allFinite()) return false;
    vec_E<Vec3f> pts;
    pts.reserve(pc.cols());
    int cnt = 0;
    double min_dis = std::numeric_limits<double>::max();
    for (long int i = 0; i < vec.size(); i++) {
        if (vec(i) <= 1) {
            pts.push_back(pc.col(i));
            if (vec(i) <= min_dis) {
                min_pt_id = static_cast<int>(i);
                min_dis = vec(i);
            }
            cnt++;
        }
    }
    if (!pts.empty()) {
        out = Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>>(pts[0].data(), 3, pts.size());
        return true;
    } else {
        return false;
    }
}

bool Ellipsoid::inside(const Vec3f &pt) const {
    if (empty() || !pt.allFinite()) return false;
    return dist(pt) <= 1;
}
