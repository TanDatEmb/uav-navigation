#include <gtest/gtest.h>

#include <esekfom/esekfom.hpp>
#include <mtk/build_manifold.hpp>
#include <mtk/types/vect.hpp>

namespace {

using Vector3 = MTK::vect<3, double>;

MTK_BUILD_MANIFOLD(SmokeState, ((Vector3, position)));
MTK_BUILD_MANIFOLD(SmokeInput, ((Vector3, acceleration)));

TEST(IkfomUpstreamSmoke, InstantiatesActualEsekfType) {
  esekfom::esekf<SmokeState, 3, SmokeInput> filter;
  const auto state = filter.get_x();
  EXPECT_EQ(SmokeState::DOF, 3);
  EXPECT_EQ(state.position.size(), 3);
}

}  // namespace
