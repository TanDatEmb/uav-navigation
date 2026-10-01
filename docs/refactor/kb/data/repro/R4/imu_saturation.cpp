#include "fast_lio_core/synchronization/measurement_synchronizer.hpp"
#include <cstdio>
using namespace uav::nav::lio;
int main(){
  MeasurementBuffer buf; MeasurementSynchronizer sync;
  const std::int64_t dt=5'000'000; // 200 Hz
  std::int64_t t=1'000'000'000; int rejected=0;
  for(int i=0;i<9000;++i){ ImuSample s; s.time=Timestamp(t, ClockDomain::kSimulationTime); s.linear_acceleration_imu_m_s2={0,0,9.8};
    if(!buf.pushImu(s).ok()) ++rejected; t+=dt; }
  std::printf("imu buffered=%zu rejected=%d last_ok_t=%.3f s\n", buf.imuSize(), rejected, (1e9+8191.0*dt)/1e9);
  // LiDAR starts now (IMU continues)
  int waits=0, groups=0, lidar_rej=0;
  for(int k=0;k<50;++k){
    LidarScan sc; sc.start_time=Timestamp(t, ClockDomain::kSimulationTime); sc.end_time=Timestamp(t+100'000'000, ClockDomain::kSimulationTime);
    LidarPoint p; p.position_lidar_m={1,0,0}; sc.points.push_back(p);
    if(!buf.pushLidar(sc).ok()) ++lidar_rej;
    for(int j=0;j<20;++j){ ImuSample s; s.time=Timestamp(t, ClockDomain::kSimulationTime); (void)buf.pushImu(s); t+=dt; }
    auto r=sync.synchronizeNext(buf);
    if(!r.ok()) std::printf("err %s\n", r.status().message().c_str());
    else if(r.value().has_value()) ++groups; else ++waits;
  }
  std::printf("after 5 s of LiDAR: groups=%d waits=%d lidar_rejected=%d imu_size=%zu lidar_size=%zu\n",groups,waits,lidar_rej,buf.imuSize(),buf.lidarSize());
}
