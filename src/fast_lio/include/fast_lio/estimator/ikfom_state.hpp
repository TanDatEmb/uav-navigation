// Copyright 2026 TanDatEmb.
//
// IKFoM manifold state definitions for the FAST-LIO 15-DOF UAV estimator.
//
// State: [pos, rot, vel, bg, ba] ∈ ℝ³ × SO(3) × ℝ³ × ℝ³ × ℝ³
// Input: [acc, gyro] ∈ ℝ³ × ℝ³
// Process noise: [ng, na, nbg, nba] ∈ ℝ¹²
//
// Gravity is treated as a constant world-frame vector (z-up, −9.80665 m/s²)
// matching the existing 15-DOF contract in docs/ieskf_design.md.

#ifndef FAST_LIO_ESTIMATOR_IKFOM_STATE_HPP_
#define FAST_LIO_ESTIMATOR_IKFOM_STATE_HPP_

#include "fast_lio/commons.hpp"

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

namespace fast_lio {

// Primitive manifolds.
using ikfom_vect3 = MTK::vect<3, double>;
using ikfom_so3 = MTK::SO3<double>;

// Compound state manifold (15-DOF).
MTK_BUILD_MANIFOLD(
    IkfomState,
    ((ikfom_vect3, pos))
    ((ikfom_so3, rot))
    ((ikfom_vect3, vel))
    ((ikfom_vect3, bg))
    ((ikfom_vect3, ba))
);

// Compound input manifold.
MTK_BUILD_MANIFOLD(
    IkfomInput,
    ((ikfom_vect3, acc))
    ((ikfom_vect3, gyro))
);

// Compound process-noise manifold.
MTK_BUILD_MANIFOLD(
    IkfomProcessNoise,
    ((ikfom_vect3, ng))
    ((ikfom_vect3, na))
    ((ikfom_vect3, nbg))
    ((ikfom_vect3, nba))
);

/**
 * @brief Convert the project-wide State15 to an IKFoM state manifold.
 */
inline IkfomState ToIkfomState(const State15& state) {
    IkfomState s;
    s.pos = state.p_w;
    s.rot = ikfom_so3(Eigen::Quaterniond(state.R_wb.matrix()));
    s.vel = state.v_w;
    s.bg = state.b_w;
    s.ba = state.b_a;
    return s;
}

/**
 * @brief Convert an IKFoM state manifold to the project-wide State15.
 */
inline State15 FromIkfomState(const IkfomState& s) {
    State15 state;
    state.p_w = s.pos;
    state.R_wb = SO3d(s.rot.toRotationMatrix());
    state.v_w = s.vel;
    state.b_w = s.bg;
    state.b_a = s.ba;
    return state;
}

/**
 * @brief Convert IMUData to the IKFoM input manifold.
 */
inline IkfomInput ToIkfomInput(const IMUData& imu) {
    IkfomInput in;
    in.acc = imu.acc;
    in.gyro = imu.gyro;
    return in;
}

}  // namespace fast_lio

#endif  // FAST_LIO_ESTIMATOR_IKFOM_STATE_HPP_
