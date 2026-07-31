#!/usr/bin/env bash
set -u
SESSION_DIR="${SESSION_DIR:-.artifacts/simulation/latest}"
while true; do
  clear
  python3 - "${SESSION_DIR}" <<'PY'
import json,sys,time
from pathlib import Path
s=Path(sys.argv[1]); latest=s/"latest"
def load(name):
 try:return json.loads((latest/name).read_text())
 except (OSError,ValueError):return {}
o=load("observer_state.json"); streams=load("stream_state.json")
p=load("pointcloud_state.json"); proc=load("process_state.json"); gz=load("gazebo_state.json")
state=o.get("state","STARTING"); color="\033[32m" if state=="HEALTHY" else ("\033[33m" if state in ("STARTING","DEGRADED") else "\033[31m")
print(f"{color}STATE: {state}\033[0m")
print(f"Session: {s.resolve()}")
for label,key in (("ROS IMU","imu"),("ROS LiDAR","lidar"),("Odom","odometry"),("Registered","registered_points")):
 x=streams.get(key,{})
 print(f"{label:22} {x.get('receive_rate_hz_short_window',0):7.2f} Hz age={x.get('age_wall','n/a')}")
print(f"Gazebo raw/points:     seq={gz.get('raw_scan_sequence',0)}/{gz.get('pointcloud_sequence',0)}")
print(f"LiDAR finite ratio:    {p.get('finite_ratio','n/a')}")
d=load("diagnostics.json")
values={}
for status in d.values(): values.update(status.get("values",{}))
print(f"FAST-LIO accepted:     lidar={values.get('core_accepted_lidar_count','n/a')}")
print(f"Correction success:    {values.get('correction_success_count','n/a')}")
print(f"Queues IMU/LiDAR:      {values.get('current_imu_queue_depth','?')}/{values.get('imu_queue_capacity','?')}  {values.get('current_lidar_queue_depth','?')}/{values.get('lidar_queue_capacity','?')}")
h=proc.get("host",{})
print(f"RTF / CPU / Swap:      {gz.get('real_time_factor','n/a')} / {h.get('cpu_total_percent','n/a')}% / {h.get('swap_used_kib','n/a')} KiB")
event=o.get("last_event")
print(f"Last event:            {event.get('code') if event else 'none'}")
PY
  sleep 2
done
