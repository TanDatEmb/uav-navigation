// Uses repo tracking_envelope.hpp unchanged. Error of norm 1.3*L (0.919 L along, 0.919 L lateral), phase window 0.
#include "px4_navigation_external_mode/tracking_envelope.hpp"
#include <cstdio>
using namespace px4_navigation_external_mode;
int main(){
  const double L=1.0; Eigen::Vector3d measured(0,0,0); Eigen::Vector3d cmd(0.919,0.919,0);
  for(double v: {0.0009, 0.0011, 1.0}){
    auto r=evaluateTrackingEnvelope(measured,cmd,Eigen::Vector3d(v,0,0),L,0.0);
    printf("speed=%.4f |err|=%.3f valid=%d long=%.3f lat=%.3f\n",v,cmd.norm(),r.valid,r.longitudinal_error_m,r.lateral_error_m);
  }
}
