// Uses repo geometric_jump_continuity.cpp + geometric_jump_latch.cpp unchanged.
// A persistent 5 m frame jump delivered with dt = 50 us after the previous sample.
#include "px4_odometry_bridge/geometric_jump_continuity.hpp"
#include "px4_odometry_bridge/geometric_jump_latch.hpp"
#include <cstdio>
using namespace px4_odometry_bridge;
static ExternalOdometryFrame f(long long t_ns, double x){ExternalOdometryFrame r; r.timestamp_ns=t_ns; r.position_ned=Eigen::Vector3d(x,0,0); r.frame_valid=true; r.covariance_valid=true; return r;}
int main(){
  GeometricJumpContinuityConfig cfg; GeometricJumpContinuityState st; GeometricJumpLatch latch;
  long long t=1'000'000'000; double x=0;
  auto step=[&](long long tt,double xx,const char* tag){auto o=observe_geometric_jump_continuity(f(tt,xx),true,true,7,cfg,st); latch.observeGeometricJump(o.jumped);
    printf("%-22s t=%lld x=%.2f reason=%s jumped=%d trusted=%d latched=%d -> gate_publishes=%d\n",tag,tt,xx,to_string(o.reason),o.jumped,st.continuity_trusted,latch.latched(),st.continuity_trusted&&!latch.latched());};
  for(int i=0;i<3;++i){t+=10'000'000; x+=0.01; step(t,x,"normal");}
  step(t+50'000, x+5.0, "jump dt=50us");
  t+=10'000'000; step(t, x+5.01, "next (persistent jump)");
  // control: same jump with normal dt
  GeometricJumpContinuityState st2=st; st=GeometricJumpContinuityState{}; latch=GeometricJumpLatch{};
  t+=10'000'000; step(t,0,"ctrl baseline"); t+=10'000'000; step(t,0.01,"ctrl normal"); t+=10'000'000; step(t,5.02,"ctrl jump dt=10ms");
}
