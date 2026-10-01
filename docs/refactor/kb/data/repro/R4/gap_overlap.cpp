#include "fast_lio_core/synchronization/measurement_synchronizer.hpp"
#include <cstdio>
using namespace uav::nav::lio;
static Timestamp T(std::int64_t ms){return Timestamp(ms*1'000'000, ClockDomain::kSimulationTime);}
static LidarScan scan(std::int64_t a,std::int64_t b){LidarScan s; s.start_time=T(a); s.end_time=T(b); LidarPoint p; p.position_lidar_m={1,0,0}; s.points.push_back(p); return s;}
int main(){
  MeasurementBuffer buf; MeasurementSynchronizer sync;
  // IMU every 5 ms from 1000 to 1195 ms, then gap to 1230 (35 ms > 20 ms), then continuous to 1500
  for(std::int64_t ms=1000; ms<=1195; ms+=5){ImuSample s; s.time=T(ms); (void)buf.pushImu(s);}
  for(std::int64_t ms=1230; ms<=1500; ms+=5){ImuSample s; s.time=T(ms); (void)buf.pushImu(s);}
  (void)buf.pushLidar(scan(1000,1100)); (void)buf.pushLidar(scan(1100,1200)); (void)buf.pushLidar(scan(1200,1300)); (void)buf.pushLidar(scan(1300,1400));
  for(int i=0;i<4;++i){ auto r=sync.synchronizeNext(buf);
    if(!r.ok()) std::printf("scan %d: ERROR %s\n", i, r.status().message().c_str());
    else if(r.value().discontinuity) std::printf("scan %d: discontinuity gap_end=%lld\n", i,(long long)r.value().discontinuity->gap_end.nanoseconds()/1000000);
    else if(r.value().has_value()) std::printf("scan %d: group start=%lld\n", i,(long long)r.value()->scan.start_time.nanoseconds()/1000000);
    else std::printf("scan %d: wait\n", i);}
}
