#!/usr/bin/env bash
# Adapted from audit-runtime-observability 84abeeb2. Experiment-only recorder.
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo 'usage: record_hold_topics.sh OUTPUT_DIR' >&2
  exit 2
fi
out=$1
mkdir -p "$out"
if ! ros2 interface show px4_msgs/msg/VehicleCommand >/dev/null; then
  echo 'px4_msgs type support unavailable; source the product install/setup.bash' >&2
  exit 4
fi
required=(/fmu/in/vehicle_command /fmu/in/vehicle_command_mode_executor /fmu/out/vehicle_command_ack /fmu/out/mode_completed /fmu/out/vehicle_status_v1 /navigation/audit_event /navigation/navigation_command /navigation/diagnostics)
ready=0
for attempt in {1..30}; do
  ros2 topic list -t > "$out/topic_inventory.txt"
  ready=1
  for topic in "${required[@]}"; do
    if ! grep -Fq "$topic [" "$out/topic_inventory.txt"; then
      ready=0
    fi
  done
  if (( ready )); then break; fi
  sleep 1
done
if (( ! ready )); then
  echo "required Hold topic unavailable after 30 attempts; see $out/topic_inventory.txt" >&2
  exit 3
fi
ros2 bag record -o "$out/hold_protocol_bag" --topics "${required[@]}" /clock /navigation/mode_status
