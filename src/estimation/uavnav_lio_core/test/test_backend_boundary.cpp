// Compile-time guard of the LIO frontend/backend boundary (SYSTEM_DESIGN §3.5, D32): the backend's public
// headers must not reach the lifecycle header, not even transitively. If one does, this file fails to
// build (the lifecycle header defines UAVNAV_LIO_LIFECYCLE_HPP_INCLUDED). The backend sources carry the same
// check in their last lines.
#include <gtest/gtest.h>

#include "uavnav/lio/backend.hpp"
#include "uavnav/lio/backend_config.hpp"
#include "uavnav/lio/channels.hpp"
#include "uavnav/lio/types.hpp"

#ifdef UAVNAV_LIO_LIFECYCLE_HPP_INCLUDED
#error "a LIO backend header reaches the lifecycle header (D32)"
#endif

TEST(LioSplit, BackendHeadersCompileWithoutTheLifecycle) { SUCCEED(); }
