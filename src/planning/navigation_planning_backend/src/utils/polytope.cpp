/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#include <data_structure/base/polytope.h>
#include <random>

using namespace geometry_utils;
using namespace color_text;
using namespace std;

namespace {

bool validPlanes(const MatD4f& planes) {
    if (planes.rows() <= 0 || planes.cols() != 4 || !planes.allFinite()) {
        return false;
    }
    for (Eigen::Index row = 0; row < planes.rows(); ++row) {
        const double normal_norm = planes.block<1, 3>(row, 0).norm();
        if (!std::isfinite(normal_norm) || normal_norm <= 1.0e-12) {
            return false;
        }
    }
    return true;
}

}  // namespace

Polytope::Polytope(MatD4f _planes) {
    SetPlanes(std::move(_planes));
}

bool Polytope::empty() const {
    return undefined;
}

bool Polytope::HaveSeedLine() const  {
    return have_seed_line;
}

void Polytope::SetSeedLine(const std::pair<Vec3f, Vec3f> &_seed_line, double r) {
    if (!_seed_line.first.allFinite() || !_seed_line.second.allFinite() ||
        !std::isfinite(r) || r < 0.0) {
        have_seed_line = false;
        seed_line = {};
        robot_r = std::numeric_limits<double>::quiet_NaN();
        return;
    }
    robot_r = r;
    seed_line = _seed_line;
    have_seed_line = true;
}

int Polytope::SurfNum() const {
    if (undefined) {
        return 0;
    }
    return planes.rows();
}

Vec3f Polytope::CrossCenter(const Polytope &b) const {
    if (empty() || b.empty()) {
        return Vec3f::Constant(std::numeric_limits<float>::quiet_NaN());
    }
    MatD4f curIH;
    curIH.resize(this->SurfNum() + b.SurfNum(), 4);
    curIH << this->planes, b.GetPlanes();
    Mat3Df curIV; // 走廊的顶点
    if (!geometry_utils::enumerateVs(curIH, curIV)) {
        printf(" -- [processCorridor] Failed to get Overlap enumerateVs .\n");
        return Vec3f::Constant(std::numeric_limits<float>::quiet_NaN());
    }
    if (curIV.cols() <= 0 || !curIV.allFinite()) {
        return Vec3f::Constant(std::numeric_limits<float>::quiet_NaN());
    }
    double x = (curIV.row(0).maxCoeff() + curIV.row(0).minCoeff()) * 0.5;
    double y = (curIV.row(1).maxCoeff() + curIV.row(1).minCoeff()) * 0.5;
    double z = (curIV.row(2).maxCoeff() + curIV.row(2).minCoeff()) * 0.5;
    return Vec3f(x, y, z);
}


Polytope Polytope::CrossWith(const Polytope &b) const {
    if (empty() || b.empty()) {
        return Polytope{};
    }
    MatD4f curIH;
    curIH.resize(this->SurfNum() + b.SurfNum(), 4);
    curIH << this->planes, b.GetPlanes();
    Polytope out;
    out.SetPlanes(curIH);
    return out;
}

bool Polytope::HaveOverlapWith(const Polytope& cmp, double eps) const {
    if (empty() || cmp.empty() || !std::isfinite(eps) || eps < 0.0) {
        return false;
    }
    return geometry_utils::overlap(this->planes, cmp.GetPlanes(), eps);
}


MatD4f Polytope::GetPlanes() const {
    return planes;
}

void Polytope::Reset() {
    undefined = true;
    is_known_free = false;
    planes.resize(0, 0);
    have_seed_line = false;
    seed_line = {};
    robot_r = std::numeric_limits<double>::quiet_NaN();
    overlap_depth_with_last_one = 0.0;
    interior_pt_with_last_one.setConstant(
        std::numeric_limits<float>::quiet_NaN());
    ellipsoid_ = Ellipsoid{};
    route_boundary_gate_ = false;
    route_boundary_point_.setConstant(
        std::numeric_limits<float>::quiet_NaN());
    route_boundary_radius_m_ = std::numeric_limits<double>::quiet_NaN();
}

bool Polytope::IsKnownFree() {
    if (undefined) {
        return false;
    }
    return is_known_free;
}

void Polytope::SetKnownFree(bool is_free) {
    is_known_free = is_free;
}

void Polytope::SetPlanes(MatD4f _planes) {
    Reset();
    if (!validPlanes(_planes)) {
        return;
    }
    planes = std::move(_planes);
    undefined = false;
}

void Polytope::SetEllipsoid(const Ellipsoid &ellip) {
    ellipsoid_ = ellip;
}

bool Polytope::PointIsInside(const Vec3f &pt, const double & margin) const {
    if (undefined || !validPlanes(planes) || !pt.allFinite() ||
        !std::isfinite(margin) || margin < 0.0) {
        return false;
    }
    Eigen::Vector4d pt_e;
    pt_e.head(3) = pt;
    pt_e(3) = 1;
    const Eigen::VectorXd plane_values = planes * pt_e;
    if (!plane_values.allFinite() || plane_values.maxCoeff() > margin) {
        return false;
    }
    return true;
}

double Polytope::GetVolume() const {
    // 首先，我们需要获取多面体的顶点
    Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
    MatD4f planes = GetPlanes();
    if (!geometry_utils::enumerateVs(planes, vPoly)) {
        printf("Failed to compute volume: cannot enumerate vertices.\n");
        return 0;
    }

    // 使用QuickHull库计算凸包
    geometry_utils::QuickHull<double> qh;
    const auto convexHull = qh.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
    const auto &indexBuffer = convexHull.getIndexBuffer();

    // 确保我们至少有四个顶点，这样才能构成一个四面体
    if (indexBuffer.size() < 4) {
        printf("Not enough vertices to compute volume.\n");
        return 0;
    }

    // 计算多面体的重心
    Vec3f centroid = Vec3f::Zero();
    for (long int i = 0; i < vPoly.cols(); i++) {
        centroid += vPoly.col(i);
    }
    centroid /= static_cast<double>(vPoly.cols());

    // 对每个三角形面，计算其四面体相对于重心的体积
    double volume = 0.0;
    for (size_t i = 0; i < indexBuffer.size(); i += 3) {
        Vec3f A = vPoly.col(indexBuffer[i]);
        Vec3f B = vPoly.col(indexBuffer[i + 1]);
        Vec3f C = vPoly.col(indexBuffer[i + 2]);

        // 使用标量三重积计算四面体的有向体积
        double tetrahedronVolume = (A - centroid).dot((B - centroid).cross(C - centroid)) / 6.0;
        volume += std::abs(tetrahedronVolume);
    }

    return volume;
}
