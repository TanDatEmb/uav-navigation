// Minimal SE3 implementation using Eigen only
// Used when Sophus is not available

#ifndef FAST_LIO_EIGEN_SE3_HPP_
#define FAST_LIO_EIGEN_SE3_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>

namespace fast_lio {

// Minimal SO3 and SE3 using Eigen - compatible with Sophus API
struct SO3d {
    Eigen::Matrix3d R_;

    SO3d() : R_(Eigen::Matrix3d::Identity()) {}
    explicit SO3d(const Eigen::Matrix3d& matrix) : R_(matrix) {}
    explicit SO3d(const Eigen::Quaterniond& q) : R_(q.toRotationMatrix()) {}

    static SO3d exp(const Eigen::Vector3d& omega) {
        const double theta_squared = omega.squaredNorm();
        const Eigen::Matrix3d omega_skew = skew(omega);

        double sinc = 1.0;
        double one_minus_cos_over_theta_squared = 0.5;
        if (theta_squared < 1e-12) {
            // Preserve tiny rotations instead of quantizing every increment
            // below 1e-6 rad to identity.
            sinc -= theta_squared / 6.0;
            one_minus_cos_over_theta_squared -= theta_squared / 24.0;
        } else {
            const double theta = std::sqrt(theta_squared);
            sinc = std::sin(theta) / theta;
            one_minus_cos_over_theta_squared = (1.0 - std::cos(theta)) / theta_squared;
        }

        return SO3d(Eigen::Matrix3d::Identity() + sinc * omega_skew +
                    one_minus_cos_over_theta_squared * omega_skew * omega_skew);
    }

    SO3d operator*(const SO3d& other) const {
        return SO3d(R_ * other.R_);
    }

    SO3d inverse() const {
        return SO3d(R_.transpose());
    }

    Eigen::Vector3d operator*(const Eigen::Vector3d& v) const {
        return R_ * v;
    }

    // Sophus-compatible API
    const Eigen::Matrix3d& matrix() const {
        return R_;
    }
    Eigen::Matrix3d rotationMatrix() const {
        return R_;
    }

   private:
    static Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d M;
        M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
        return M;
    }
};

// SE3d with Sophus-compatible API
struct SE3d {
    SO3d so3_;
    Eigen::Vector3d translation_;

    // Default constructor
    SE3d() : so3_(), translation_(Eigen::Vector3d::Zero()) {}

    // Constructor from rotation and translation
    SE3d(const SO3d& R, const Eigen::Vector3d& t) : so3_(R), translation_(t) {}

    // Constructor from 4x4 matrix
    explicit SE3d(const Eigen::Matrix4d& T)
        : so3_(SO3d(T.block<3, 3>(0, 0))), translation_(T.block<3, 1>(0, 3)) {}

    // Accessors - match Sophus API
    const SO3d& rotation() const {
        return so3_;
    }
    SO3d& rotation() {
        return so3_;
    }

    const Eigen::Vector3d& translation() const {
        return translation_;
    }
    Eigen::Vector3d& translation() {
        return translation_;
    }

    // Get 4x4 matrix representation
    Eigen::Matrix4d matrix() const {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = so3_.matrix();
        T.block<3, 1>(0, 3) = translation_;
        return T;
    }

    // SE3 composition
    SE3d operator*(const SE3d& other) const {
        return SE3d(so3_ * other.so3_, so3_ * other.translation_ + translation_);
    }

    SE3d inverse() const {
        SO3d inv_r = so3_.inverse();
        return SE3d(inv_r, inv_r * (-translation_));
    }

    // Transform point
    Eigen::Vector3d operator*(const Eigen::Vector3d& v) const {
        return so3_ * v + translation_;
    }
};

}  // namespace fast_lio

// Make available in global namespace for compatibility
using fast_lio::SE3d;
using fast_lio::SO3d;

#endif  // FAST_LIO_EIGEN_SE3_HPP_
