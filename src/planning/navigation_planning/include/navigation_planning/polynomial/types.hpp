#pragma once

#include <Eigen/Eigen>

#include <vector>

namespace navigation_planning::polynomial_types {

using decimal_t = double;

template<typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;

template<int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;

template<int M, int N>
using Matf = Eigen::Matrix<decimal_t, M, N>;

template<int N>
using MatDNf = Eigen::Matrix<decimal_t, Eigen::Dynamic, N>;

template<int M>
using MatMDf = Eigen::Matrix<decimal_t, M, Eigen::Dynamic>;

using Vec3f = Vecf<3>;
using Vec4f = Vecf<4>;
using vec_Vec3f = vec_E<Vec3f>;
using Mat3f = Matf<3, 3>;
using VecDf = Vecf<Eigen::Dynamic>;
using MatD4f = MatDNf<4>;
using Mat3Df = MatMDf<3>;
using MatDf = Matf<Eigen::Dynamic, Eigen::Dynamic>;
using Quatf = Eigen::Quaternion<decimal_t>;
using StatePVA = Eigen::Matrix<double, 3, 3>;
using StatePVAJ = Eigen::Matrix<double, 3, 4>;

}  // namespace navigation_planning::polynomial_types
