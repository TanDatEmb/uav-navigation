/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */


#pragma once
#include <utils/header/type_utils.hpp>
#include <utils/geometry/geometry_utils.h>
#include <utils/header/color_msg_utils.hpp>

namespace geometry_utils {
    using navigation_math::Mat3f;
    using navigation_math::Vec3f;
    using navigation_math::vec_Vec3f;
    using navigation_math::Mat3Df;
    

    class Ellipsoid {
        /// If the ellipsoid is empty
        bool undefined{true};

        /// The ellipsoid is defined by shape C and center d
        Mat3f C_{}, C_inv_{};
        Mat3f R_{};
        Vec3f r_{}, d_{};

    public:
        Ellipsoid() = default;

        template <class Archive>
        void serialize(Archive& archive) {
            archive(undefined, C_, C_inv_, R_, r_, d_);
        }

        Ellipsoid(const Mat3f& C, const Vec3f& d);

        Ellipsoid(const Mat3f& R, const Vec3f& r, const Vec3f& d);

        /// If this ellipsoid is empty
        bool empty() const;

        double pointDistaceToEllipsoid(const Vec3f& pt, Vec3f& closest_pt_on_ellip) const;

        /// Find the closestPoint in a point set
        int nearestPointId(const Eigen::Matrix3Xd& pc) const;

        /// Find the closestPoint in a point set
        Vec3f nearestPoint(const Eigen::Matrix3Xd& pc) const;

        /// Find the closestPoint in a point set
        double nearestPointDis(const Eigen::Matrix3Xd& pc, int& np_id) const;

        /// Get the shape of the ellipsoid
        Mat3f C() const;

        /// Get the center of the ellipsoid
        Vec3f d() const;

        Mat3f R() const;

        Vec3f r() const;


        /// Convert a point to the ellipsoid frame
        Vec3f toEllipsoidFrame(const Vec3f& pt_w) const;

        /// Convert a set of points to the ellipsoid frame
        Eigen::Matrix3Xd toEllipsoidFrame(const Eigen::Matrix3Xd& pc_w) const;

        /// Convert a point to the world frame
        Vec3f toWorldFrame(const Vec3f& pt_e) const;

        /// Convert a set of points to the world frame
        Eigen::Matrix3Xd toWorldFrame(const Eigen::Matrix3Xd& pc_e) const;

        /// Convert a plane to the ellipsoid frame
        Eigen::Vector4d toEllipsoidFrame(const Eigen::Vector4d& plane_w) const;

        /// Convert a plane to the ellipsoid frame
        Eigen::Vector4d toWorldFrame(const Eigen::Vector4d& plane_e) const;

        /// Convert a set of planes to ellipsoid frame
        Eigen::MatrixX4d toEllipsoidFrame(const Eigen::MatrixX4d& planes_w) const;

        /// Convert a set of planes to ellipsoid frame
        Eigen::MatrixX4d toWorldFrame(const Eigen::MatrixX4d& planes_e) const;

        /// Calculate the distance of a point in world frame
        double dist(const Vec3f& pt_w) const;

        /// Calculate the distance of a point in world frame
        Eigen::VectorXd dist(const Eigen::Matrix3Xd& pc_w) const;

        bool noPointsInside(vec_Vec3f& pc, const Eigen::Matrix3d& R,
                            const Vec3f& r, const Vec3f& p) const;

        bool pointsInside(const Eigen::Matrix3Xd& pc,
                          Mat3Df& out,
                          int& min_pt_id) const;

        /// Check if the point is inside, non-exclusive
        bool inside(const Vec3f& pt) const;

    };
}
