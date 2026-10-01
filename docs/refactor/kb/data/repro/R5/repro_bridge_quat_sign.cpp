// Compiles repo frame_converter.cpp unchanged; sweeps NED yaw and checks output ENU quaternion sign continuity
// and per-axis variance handling.
#include "px4_odometry_bridge/frame_converter.hpp"
#include <cstdio>
#include <cmath>
using namespace px4_odometry_bridge;
int main(){
  FrameConverter c; Eigen::Quaterniond prev; bool have=false; int flips=0;
  for(int d=-180; d<=180; ++d){
    Px4OdometrySample s; s.timestamp_ns=1000+d+180; s.pose_frame=PoseFrame::kNed; s.velocity_frame=VelocityFrame::kNed;
    double y=d*M_PI/180.0; s.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(y,Eigen::Vector3d::UnitZ()));
    auto r=c.convert(s); auto q=r.value->orientation;
    if(have && prev.dot(q)<0){ ++flips; printf("flip at ned_yaw=%d deg: prev(w=%.3f z=%.3f) now(w=%.3f z=%.3f) dot=%.3f\n",d,prev.w(),prev.z(),q.w(),q.z(),prev.dot(q)); }
    prev=q; have=true;
  }
  printf("flips=%d\n",flips);
  Px4OdometrySample s; s.timestamp_ns=1; s.pose_frame=PoseFrame::kNed; s.velocity_frame=VelocityFrame::kNed;
  s.position_variance=Eigen::Vector3d(0.1,-1.0,0.3);
  auto r=c.convert(s);
  printf("pos var NED(0.1,-1,0.3) -> ENU (%.2f %.2f %.2f) avail=%d (expected ENU (-1,0.1,0.3))\n",r.value->position_variance.x(),r.value->position_variance.y(),r.value->position_variance.z(),r.value->position_covariance_available);
  s.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(0.6,Eigen::Vector3d::UnitX()));
  s.velocity_variance=Eigen::Vector3d(0.5,0.5,-1.0);
  r=c.convert(s);
  printf("vel var NED(0.5,0.5,-1) roll0.6 -> body (%.3f %.3f %.3f) avail=%d\n",r.value->velocity_variance.x(),r.value->velocity_variance.y(),r.value->velocity_variance.z(),r.value->velocity_covariance_available);
}
