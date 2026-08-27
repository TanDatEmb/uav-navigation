#pragma once

#include <Eigen/Core>

#include <esekfom/esekfom.hpp>
#include <mtk/build_manifold.hpp>
#include <mtk/types/S2.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>

namespace uav::nav::lio {

using IkfomVector3 = MTK::vect<3, double>;
using IkfomSo3 = MTK::SO3<double>;
using IkfomGravity = MTK::S2<double, 98090, 10000, 1>;

// Ported selectively from the pinned estimator reference; exact provenance is
// recorded in the vendor documentation.
// 7cc4175de6f8ba2edf34bab02a42195b141027e9. The ordering is the upstream
// FAST-LIO2/IKFoM state ordering and yields a 23-DoF covariance.
MTK_BUILD_MANIFOLD(
    IkfomState,
    ((IkfomVector3, pos))((IkfomSo3, rot))((IkfomSo3, offset_R_L_I))(
        (IkfomVector3, offset_T_L_I))((IkfomVector3, vel))(
        (IkfomVector3, bg))((IkfomVector3, ba))((IkfomGravity, grav)));

MTK_BUILD_MANIFOLD(
    IkfomInput,
    ((IkfomVector3, acc))((IkfomVector3, gyro)));

MTK_BUILD_MANIFOLD(
    IkfomProcessNoise,
    ((IkfomVector3, ng))((IkfomVector3, na))((IkfomVector3, nbg))(
        (IkfomVector3, nba)));

using IkfomFilter = esekfom::esekf<IkfomState, 12, IkfomInput>;

[[nodiscard]] Eigen::Matrix<double, 24, 1> ikfomProcessModel(
    IkfomState& state, const IkfomInput& input);
[[nodiscard]] Eigen::Matrix<double, 24, 23> ikfomProcessJacobianState(
    IkfomState& state, const IkfomInput& input);
[[nodiscard]] Eigen::Matrix<double, 24, 12> ikfomProcessJacobianNoise(
    IkfomState& state, const IkfomInput& input);

}  // namespace uav::nav::lio
