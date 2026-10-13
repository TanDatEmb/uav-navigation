#!/usr/bin/env python3
"""rclpy probe of the risky SITL assumptions (S1b Task 3; D33 O20, O24; D29).

Started by tools/uavnav/sitl_hover.sh --probe once PX4 SITL, MicroXRCEAgent and the two gz bridges
run. It flies one short hover in the GPS profile (PX4 has a position without EV, so neither LIO
nor the PX4 bridge is needed) and records what the checks in sitl_checks.py judge:

1. wait_ready  (<= 90 s): vehicle_local_position xy_valid (and z_valid), then the ground z.
2. mode + arm: centred sticks on /fmu/in/manual_control_input (SOURCE_MAVLINK_0, 20 Hz of /clock
   time) from the start; VEHICLE_CMD_DO_SET_MODE 1/3 (POSCTL, O24), then arm. With
   --hover-mode hold (the approved O24 fallback) it arms, takes off (NAV_TAKEOFF, MIS_TAKEOFF_ALT)
   and sets Hold with DO_SET_MODE 1/4/3.
3. climb to 2.5 m above the ground z (posctl: throttle stick; takeoff phase for hold).
4. hover for 20 s of /clock time with centred sticks (the evaluation window).
5. land (NAV_LAND) and wait for the auto-disarm.

Records (JSONL, one file per topic, in --run-dir; formats in sitl_checks.py): /lidar/points
(fields, point_step, frame, non-finite counts, stamp, newest IMU stamp at arrival), /lidar/imu,
/clock, /fmu/out/vehicle_local_position_v1, /fmu/out/vehicle_status_v1 and
/fmu/out/vehicle_command_ack; probe_meta.json holds the phases and the hover window.
A message is recorded only after the first /clock; arrival_ns is the newest /clock value then.

Exit: 0 every phase ok, 1 a phase failed (the verdict is sitl_checks.py's), 130 interrupted.
Every wait is bounded by its phase timeout and by --budget-s (wall time); the land phase keeps
LAND_RESERVE_S of the budget.
"""
import argparse
import json
import math
import pathlib
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from px4_msgs.msg import (ManualControlSetpoint, VehicleCommand, VehicleCommandAck,
                          VehicleLocalPosition, VehicleStatus)
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import Imu, PointCloud2, PointField

TARGET_HEIGHT_M = 2.5
STICK_PERIOD_NS = 50_000_000          # 20 Hz of /clock time (O24)
WAIT_READY_S = 90.0
GROUND_SAMPLE_S = 2.0                 # /clock seconds of z averaged as the ground reference
MODE_S, ARM_S = 15.0, 20.0
CLIMB_S = 40.0
HOVER_SIM_S, HOVER_WALL_MAX_S = 20.0, 45.0
LAND_S, LAND_RESERVE_S = 40.0, 30.0
MIN_RECORD_SIM_S = 30.0               # keep recording at least this long even when the flight stops early
TAKEOFF_STICK, CLIMB_GAIN, CLIMB_DONE_M = 0.6, 0.8, 0.15

NAV_STATE = {"posctl": VehicleStatus.NAVIGATION_STATE_POSCTL, "hold": VehicleStatus.NAVIGATION_STATE_AUTO_LOITER}
MODE_PARAMS = {"posctl": (1.0, 3.0, 0.0), "hold": (1.0, 4.0, 3.0)}  # MAV_MODE custom, PX4 main/sub mode
NUMPY_TYPE = {PointField.FLOAT32: "f4", PointField.FLOAT64: "f8"}


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def non_finite_counts(msg: PointCloud2):
    """(n_nan, n_inf) over the x, y, z of every point, or (None, None) if the layout is unreadable."""
    fields = {f.name: f for f in msg.fields}
    if not all(a in fields and fields[a].datatype in NUMPY_TYPE and fields[a].count == 1 for a in "xyz"):
        return None, None
    endian = ">" if msg.is_bigendian else "<"
    try:
        dtype = np.dtype({"names": ["x", "y", "z"],
                          "formats": [endian + NUMPY_TYPE[fields[a].datatype] for a in "xyz"],
                          "offsets": [fields[a].offset for a in "xyz"], "itemsize": msg.point_step})
    except (ValueError, TypeError):
        return None, None  # a field outside point_step
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        return None, None
    data = bytes(msg.data)
    n_nan = n_inf = 0
    for row in range(msg.height):
        pts = np.frombuffer(data, dtype=dtype, count=msg.width, offset=row * msg.row_step)
        xyz = np.stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
        nan = np.isnan(xyz).any(axis=0)
        non_finite = ~np.isfinite(xyz).all(axis=0)
        n_nan += int(nan.sum())
        n_inf += int((non_finite & ~nan).sum())
    return n_nan, n_inf


class Probe(Node):
    def __init__(self, run_dir: pathlib.Path):
        super().__init__("uavnav_sitl_probe")
        self._lock = threading.Lock()
        self._files = {name: open(run_dir / f"{name}.jsonl", "w", encoding="utf-8", buffering=1)
                       for name in ("lidar_points", "lidar_imu", "clock", "vehicle_local_position",
                                    "vehicle_status", "vehicle_command_ack")}
        self.clock_ns = None
        self.newest_imu_ns = None
        self.position = None
        self.status = None
        self.throttle = 0.0
        self._last_stick_ns = None

        sensor = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(ClockMsg, "/clock", self._on_clock, sensor)
        self.create_subscription(Imu, "/lidar/imu", self._on_imu, sensor)
        self.create_subscription(PointCloud2, "/lidar/points", self._on_points,
                                 QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                            reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_position, sensor)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, sensor)
        self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self._on_ack, sensor)
        px4_in = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sticks = self.create_publisher(ManualControlSetpoint, "/fmu/in/manual_control_input", px4_in)
        self._commands = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_in)
        # Steady 100 Hz poll; a stick message goes out every 50 ms of /clock time.
        self.create_timer(0.01, self._stick_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _write(self, name, record):
        self._files[name].write(json.dumps(record, separators=(",", ":")) + "\n")

    def close(self):
        for f in self._files.values():
            f.close()

    # ---- callbacks (executor thread) ----
    def _on_clock(self, msg):
        now = stamp_ns(msg.clock)
        with self._lock:
            self.clock_ns = now
        self._write("clock", {"clock_ns": now})

    def _arrival(self):
        with self._lock:
            return self.clock_ns

    def _on_imu(self, msg):
        arrival = self._arrival()
        stamp = stamp_ns(msg.header.stamp)
        with self._lock:
            if self.newest_imu_ns is None or stamp > self.newest_imu_ns:
                self.newest_imu_ns = stamp
        if arrival is not None:
            self._write("lidar_imu", {"arrival_ns": arrival, "stamp_ns": stamp})

    def _on_points(self, msg):
        arrival = self._arrival()
        if arrival is None:
            return
        with self._lock:
            newest_imu = self.newest_imu_ns
        n_nan, n_inf = non_finite_counts(msg)
        self._write("lidar_points", {
            "arrival_ns": arrival, "stamp_ns": stamp_ns(msg.header.stamp), "newest_imu_stamp_ns": newest_imu,
            "frame_id": msg.header.frame_id, "point_step": msg.point_step,
            "fields": [{"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count}
                       for f in msg.fields],
            "n_points": msg.width * msg.height, "n_nan": n_nan, "n_inf": n_inf})

    def _on_position(self, msg):
        arrival = self._arrival()
        record = {"arrival_ns": arrival, "timestamp_us": int(msg.timestamp),
                  "timestamp_sample_us": int(msg.timestamp_sample), "xy_valid": bool(msg.xy_valid),
                  "z_valid": bool(msg.z_valid), "x": float(msg.x), "y": float(msg.y), "z": float(msg.z),
                  "vz": float(msg.vz)}
        with self._lock:
            self.position = record
        if arrival is not None:
            self._write("vehicle_local_position", {k: (v if not isinstance(v, float) or math.isfinite(v) else None)
                                                   for k, v in record.items()})

    def _on_status(self, msg):
        arrival = self._arrival()
        record = {"arrival_ns": arrival, "timestamp_us": int(msg.timestamp), "nav_state": int(msg.nav_state),
                  "arming_state": int(msg.arming_state)}
        with self._lock:
            self.status = record
        if arrival is not None:
            self._write("vehicle_status", record)

    def _on_ack(self, msg):
        arrival = self._arrival()
        if arrival is not None:
            self._write("vehicle_command_ack", {"arrival_ns": arrival, "command": int(msg.command),
                                                "result": int(msg.result), "result_param1": int(msg.result_param1)})

    def _stick_tick(self):
        with self._lock:
            now, throttle = self.clock_ns, self.throttle
        if now is None or (self._last_stick_ns is not None and 0 <= now - self._last_stick_ns < STICK_PERIOD_NS):
            return
        self._last_stick_ns = now
        msg = ManualControlSetpoint()
        msg.timestamp = msg.timestamp_sample = now // 1000  # PX4 time = /clock in SITL (D29)
        msg.valid = True
        msg.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
        msg.roll = msg.pitch = msg.yaw = 0.0
        msg.throttle = float(throttle)
        msg.flaps = msg.aux1 = msg.aux2 = msg.aux3 = msg.aux4 = msg.aux5 = msg.aux6 = 0.0
        msg.sticks_moving = throttle != 0.0
        self._sticks.publish(msg)

    # ---- main thread API ----
    def snapshot(self):
        with self._lock:
            return self.clock_ns, self.position, self.status

    def set_throttle(self, value: float):
        with self._lock:
            self.throttle = max(-1.0, min(1.0, value))

    def command(self, command: int, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        now, _, _ = self.snapshot()
        msg = VehicleCommand()
        msg.timestamp = (now or 0) // 1000
        msg.command = command
        msg.param1, msg.param2, msg.param3, msg.param4 = float(p1), float(p2), float(p3), float(p4)
        msg.param5, msg.param6, msg.param7 = float(p5), float(p6), float(p7)
        msg.target_system = msg.target_component = 1
        msg.source_system = msg.source_component = 1
        msg.from_external = True
        self._commands.publish(msg)


class Flight:
    """The phase sequence on the main thread; every wait is bounded."""

    def __init__(self, node: Probe, mode: str, budget_s: float, stop: threading.Event):
        self.node, self.mode, self.stop = node, mode, stop
        self.deadline = time.monotonic() + budget_s
        self.phases = []
        self.ground_z = None
        self.hover_window = None
        self.start_clock = None

    def log(self, text):
        print(f"[sitl_probe] {text}", flush=True)

    def wait(self, predicate, timeout_s, reserve_s=LAND_RESERVE_S, poll_s=0.05, action=None, every_s=None):
        """Poll predicate until true, the timeout, the budget (minus reserve) or a stop. Returns bool."""
        end = min(time.monotonic() + timeout_s, self.deadline - reserve_s)
        last_action = None
        while not self.stop.is_set() and time.monotonic() < end:
            if predicate():
                return True
            if action is not None and (last_action is None or time.monotonic() - last_action >= every_s):
                action()
                last_action = time.monotonic()
            time.sleep(poll_s)
        return predicate()

    def phase(self, name, body):
        clock, _, _ = self.node.snapshot()
        wall = time.monotonic()
        ok, detail = body()
        end_clock, _, _ = self.node.snapshot()
        self.phases.append({"name": name, "ok": bool(ok), "detail": detail, "start_ns": clock, "end_ns": end_clock,
                            "wall_s": round(time.monotonic() - wall, 3)})
        self.log(f"phase {name}: {'ok' if ok else 'FAILED'} {detail}".rstrip())
        return ok

    def height(self):
        _, position, _ = self.node.snapshot()
        if position is None or self.ground_z is None or not position["z_valid"]:
            return None
        return self.ground_z - position["z"]

    def armed(self):
        _, _, status = self.node.snapshot()
        return status is not None and status["arming_state"] == VehicleStatus.ARMING_STATE_ARMED

    def nav_state_is(self, wanted):
        _, _, status = self.node.snapshot()
        return status is not None and status["nav_state"] == wanted

    # ---- phases ----
    def wait_ready(self):
        def ready():
            clock, position, _ = self.node.snapshot()
            return clock is not None and position is not None and position["xy_valid"] and position["z_valid"]
        if not self.wait(ready, WAIT_READY_S):
            return False, f"no xy_valid/z_valid within {WAIT_READY_S} s"
        self.start_clock = self.node.snapshot()[0]
        samples = []
        first = self.node.snapshot()[0]

        def sampled():
            clock, position, _ = self.node.snapshot()
            if position is not None and position["z_valid"] and math.isfinite(position["z"]):
                samples.append(position["z"])
            return clock - first >= GROUND_SAMPLE_S * 1e9
        if not self.wait(sampled, GROUND_SAMPLE_S * 3 + 5.0, poll_s=0.02) or not samples:
            return False, "no ground z samples"
        self.ground_z = sum(samples) / len(samples)
        return True, f"ground z {self.ground_z:.3f} m from {len(samples)} samples"

    def set_mode(self):
        wanted = NAV_STATE[self.mode]
        p1, p2, p3 = MODE_PARAMS[self.mode]
        ok = self.wait(lambda: self.nav_state_is(wanted), MODE_S,
                       action=lambda: self.node.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1, p2, p3), every_s=1.0)
        _, _, status = self.node.snapshot()
        seen = None if status is None else status["nav_state"]
        return ok, f"nav_state {seen}, wanted {wanted}"

    def arm(self):
        ok = self.wait(self.armed, ARM_S,
                       action=lambda: self.node.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0),
                       every_s=2.0)
        return ok, "" if ok else f"not armed within {ARM_S} s (see vehicle_command_ack.jsonl)"

    def climb_posctl(self):
        def step():
            h = self.height()
            if h is None or not self.armed():
                return False
            if h >= TARGET_HEIGHT_M - CLIMB_DONE_M:
                return True
            # Above the 0.3 takeoff threshold until clear of the ground, then proportional.
            stick = TAKEOFF_STICK if h < 1.0 else min(TAKEOFF_STICK, max(0.0, CLIMB_GAIN * (TARGET_HEIGHT_M - h)))
            self.node.set_throttle(stick)
            return False
        ok = self.wait(step, CLIMB_S, poll_s=0.02)
        self.node.set_throttle(0.0)
        h = self.height()
        return ok, f"height {h if h is None else round(h, 3)} m"

    def climb_hold(self):
        nan = float("nan")
        self.node.command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, nan, nan, nan, nan)

        def reached():
            h = self.height()
            return h is not None and h >= TARGET_HEIGHT_M - CLIMB_DONE_M and \
                self.nav_state_is(NAV_STATE["hold"])
        ok = self.wait(reached, CLIMB_S)
        if ok:
            p1, p2, p3 = MODE_PARAMS["hold"]
            self.node.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1, p2, p3)
        h = self.height()
        return ok, f"height {h if h is None else round(h, 3)} m (MIS_TAKEOFF_ALT)"

    def hover(self):
        self.node.set_throttle(0.0)
        start = self.node.snapshot()[0]
        ok = self.wait(lambda: self.node.snapshot()[0] - start >= HOVER_SIM_S * 1e9 or not self.armed(),
                       HOVER_WALL_MAX_S)
        end = self.node.snapshot()[0]
        self.hover_window = {"start_ns": start, "end_ns": end}
        sim_s = (end - start) / 1e9
        return ok and self.armed() and sim_s >= HOVER_SIM_S, f"{sim_s:.2f} s of /clock time"

    def land(self):
        nan = float("nan")
        self.node.set_throttle(0.0)
        ok = self.wait(lambda: not self.armed(), LAND_S, reserve_s=0.0,
                       action=lambda: self.node.command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0, 0.0, nan,
                                                        nan, nan, nan), every_s=3.0)
        return ok, "disarmed" if ok else f"still armed after {LAND_S} s"

    def record_tail(self):
        """Keep the recording long enough for the sensor and time checks when the flight stopped early."""
        if self.start_clock is None:
            return
        self.wait(lambda: self.node.snapshot()[0] - self.start_clock >= MIN_RECORD_SIM_S * 1e9,
                  MIN_RECORD_SIM_S * 2, reserve_s=0.0)

    def run(self) -> bool:
        ok = self.phase("wait_ready", self.wait_ready)
        flown = False
        if ok:
            if self.mode == "posctl":
                ok = self.phase("posctl", self.set_mode) and self.phase("arm", self.arm)
                ok = ok and self.phase("climb", self.climb_posctl)
            else:
                ok = self.phase("arm", self.arm) and self.phase("climb", self.climb_hold)
            flown = self.armed()
            ok = ok and self.phase("hover", self.hover)
        if flown or self.armed():
            ok = self.phase("land", self.land) and ok
        self.record_tail()
        return ok


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="SITL probe of the S1b assumptions (O20, O24, D29).")
    parser.add_argument("--run-dir", required=True, type=pathlib.Path)
    parser.add_argument("--profile", required=True, choices=("gps", "no_gps"))
    parser.add_argument("--hover-mode", default="posctl", choices=tuple(NAV_STATE))
    parser.add_argument("--budget-s", required=True, type=float, help="wall-time budget of the whole probe")
    args = parser.parse_args(argv)
    if not args.run_dir.is_dir() or not args.budget_s > 0:
        parser.error("--run-dir must exist and --budget-s must be > 0")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Probe(args.run_dir)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    flight = Flight(node, args.hover_mode, args.budget_s, stop)
    ok = False
    try:
        ok = flight.run()
    finally:
        node.set_throttle(0.0)
        executor.shutdown(timeout_sec=2.0)
        spinner.join(timeout=5.0)
        node.close()
        meta = {"profile": args.profile, "hover_mode": args.hover_mode, "target_height_m": TARGET_HEIGHT_M,
                "ground_z_m": flight.ground_z, "hover_window": flight.hover_window, "phases": flight.phases,
                "interrupted": stop.is_set()}
        (args.run_dir / "probe_meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.try_shutdown()
    if stop.is_set():
        return 130
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
