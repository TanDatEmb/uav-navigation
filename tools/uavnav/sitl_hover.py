#!/usr/bin/env python3
"""rclpy driver and recorder of the S1 hover (S1b Task 11; SYSTEM_DESIGN §7.2; D20, D29; D33 O21, O24).

Started by tools/uavnav/sitl_hover.sh --hover once PX4 SITL, MicroXRCEAgent and
`ros2 launch uavnav_bringup s1_estimation.launch.py` (gz bridges, lio, px4_bridge) run. Phases, each bounded:

1. wait_ready (<= 90 s): LIO TRACKING on /lio/health, /alignment VALID, cs_ev_pos on
   /fmu/out/estimator_status_flags and xy_valid/z_valid on vehicle_local_position; then the ground z.
2. mode + arm, as the Task 3 probe fixed it (POSCTL with centred sticks on /fmu/in/manual_control_input,
   SOURCE_MAVLINK_0, 20 Hz of /clock time; DO_SET_MODE only when PX4 is not in POSCTL already). With
   --hover-mode hold (the approved O24 fallback): arm, NAV_TAKEOFF (climb), then the "hold" phase.
3. climb (<= 20 s of /clock) to 2.5 m above the ground z (posctl: throttle stick).
4. hover for 60 s of /clock time with centred sticks: the evaluation window of sitl_checks.evaluate_hover.
5. land (NAV_LAND once, again only if PX4 is not in AUTO_LAND), <= 30 s to the auto-disarm.
   On an interruption or an exception while armed, one best-effort NAV_LAND is sent before exit.

--pattern altitude_sweep (S1b Task 13; gps profile and POSCTL only): phases 1-2 as above, then "sweep": climb in
2.5 m steps to 20 m above the ground z (each climb <= 15 s of /clock, throttle stick as in 3), hold each step 10 s
of /clock with centred sticks; then land (NAV_LAND from 20 m, <= 60 s to the auto-disarm). hover_meta.json records
the steps (target, climb start, hold start/end, height reached) and no hover window; sitl_checks.evaluate_sweep
judges the data only, tools/uavnav/lio_calibration.py analyses it.

Records (JSONL, one file per topic, in --run-dir; formats in sitl_checks.py): /lio/state, /lio/health,
/lio/odometry, /alignment, /fmu/in/vehicle_visual_odometry (the EV the bridge sends),
/fmu/out/vehicle_local_position_v1, /fmu/out/estimator_status_flags, /fmu/out/vehicle_status_v1,
/fmu/out/vehicle_command_ack and /sim/ground_truth/odometry. A message is recorded only after the first /clock;
arrival_ns is the newest /clock value then. hover_meta.json holds the phases and the hover window.

Exit: 0 every phase ok, 1 a phase failed (the verdict is sitl_checks.py's), 2 usage, 130 interrupted.
Every wait is bounded by its phase timeout and by --budget-s (wall time); the land phase keeps 30 s of it.
"""
import argparse
import json
import math
import pathlib
import signal
import sys
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from nav_msgs.msg import Odometry
from px4_msgs.msg import (EstimatorStatusFlags, ManualControlSetpoint, VehicleCommand, VehicleCommandAck,
                          VehicleLocalPosition, VehicleOdometry, VehicleStatus)
from rosgraph_msgs.msg import Clock as ClockMsg
from uavnav_interfaces.msg import Alignment, LioHealth, LioOdometry, LioState

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import sitl_probe  # noqa: E402  (the Task 3 flight phases: bounded waits, arm, POSCTL climb, land)

HOVER_SIM_S, HOVER_WALL_MAX_S = 60.0, 120.0   # the evaluation window; wall bound for RTF >= 0.5
CLIMB_SIM_S, CLIMB_WALL_MAX_S = 20.0, 40.0
LAND_S = 30.0
SWEEP_TARGETS_M = tuple(2.5 * (k + 1) for k in range(8))  # 2.5 .. 20 m above the ground z (plan Task 13)
SWEEP_HOLD_SIM_S = 10.0
SWEEP_CLIMB_SIM_S, SWEEP_CLIMB_WALL_MAX_S = 15.0, 30.0    # one 2.5 m step
SWEEP_LAND_S = 60.0                                       # NAV_LAND from 20 m (descent ~1 m/s, then disarm)
RECORDS = ("lio_state", "lio_health", "lio_odometry", "alignment", "vehicle_visual_odometry",
           "vehicle_local_position", "estimator_status_flags", "vehicle_status", "vehicle_command_ack",
           "ground_truth")


def _finite(value: float):
    value = float(value)
    return value if math.isfinite(value) else None


class HoverRecorder(Node):
    """Records the topics and flies the sticks; the snapshot/command API is sitl_probe.Probe's."""

    def __init__(self, run_dir: pathlib.Path):
        super().__init__("uavnav_sitl_hover")
        self._lock = threading.Lock()
        self._files = {name: open(run_dir / f"{name}.jsonl", "w", encoding="utf-8", buffering=1) for name in RECORDS}
        self.clock_ns = None
        self.position = None
        self.status = None
        self.lio_state = None
        self.alignment_state = None
        self.ev_pos = None
        self.throttle = 0.0
        self._last_stick_ns = None

        best_effort = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.RELIABLE)
        subscriptions = (
            (ClockMsg, "/clock", self._on_clock, best_effort),
            (LioState, "/lio/state", self._on_lio_state, best_effort),
            (LioHealth, "/lio/health", self._on_lio_health, reliable),
            (LioOdometry, "/lio/odometry", self._on_lio_odometry, reliable),
            (Alignment, "/alignment", self._on_alignment, reliable),
            (VehicleOdometry, "/fmu/in/vehicle_visual_odometry", self._on_ev, best_effort),
            (VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_position, best_effort),
            (EstimatorStatusFlags, "/fmu/out/estimator_status_flags", self._on_flags, best_effort),
            (VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, best_effort),
            (VehicleCommandAck, "/fmu/out/vehicle_command_ack", self._on_ack, best_effort),
            (Odometry, "/sim/ground_truth/odometry", self._on_truth, best_effort),
        )
        for msg_type, topic, callback, qos in subscriptions:
            self.create_subscription(msg_type, topic, callback, qos)
        px4_in = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sticks = self.create_publisher(ManualControlSetpoint, "/fmu/in/manual_control_input", px4_in)
        self._commands = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_in)
        # Steady 100 Hz poll; a stick message goes out every 50 ms of /clock time.
        self.create_timer(0.01, self._stick_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def close(self):
        for f in self._files.values():
            f.close()

    def _arrival(self):
        with self._lock:
            return self.clock_ns

    def _record(self, name: str, arrival, record: dict):
        if arrival is not None:
            self._files[name].write(json.dumps(dict(arrival_ns=arrival, **record), separators=(",", ":")) + "\n")

    # ---- callbacks (executor thread) ----
    def _on_clock(self, msg):
        with self._lock:
            self.clock_ns = sitl_probe.stamp_ns(msg.clock)

    def _on_lio_state(self, msg):
        p, q, v = msg.pose.position, msg.pose.orientation, msg.velocity
        self._record("lio_state", self._arrival(), {
            "wall_ns": time.monotonic_ns(), "stamp_ns": sitl_probe.stamp_ns(msg.stamp), "epoch": int(msg.epoch),
            "lio_state": int(msg.lio_state), "x": _finite(p.x), "y": _finite(p.y), "z": _finite(p.z),
            "qw": _finite(q.w), "qx": _finite(q.x), "qy": _finite(q.y), "qz": _finite(q.z),
            "vx": _finite(v.x), "vy": _finite(v.y), "vz": _finite(v.z)})

    def _on_lio_health(self, msg):
        with self._lock:
            self.lio_state = int(msg.state)
        self._record("lio_health", self._arrival(), {
            "stamp_ns": sitl_probe.stamp_ns(msg.stamp), "state": int(msg.state), "epoch": int(msg.epoch),
            "reason_name": msg.reason_name, "correction_age_s": _finite(msg.correction_age_s),
            "translation_min_eigenvalue": _finite(msg.translation_min_eigenvalue),
            "rotation_min_eigenvalue": _finite(msg.rotation_min_eigenvalue),
            "output_error_attitude_rad": _finite(msg.output_error_attitude_rad),
            "output_error_velocity_mps": _finite(msg.output_error_velocity_mps),
            "output_error_position_m": _finite(msg.output_error_position_m)})

    def _on_lio_odometry(self, msg):
        p = msg.pose.position
        self._record("lio_odometry", self._arrival(), {
            "stamp_ns": sitl_probe.stamp_ns(msg.stamp), "epoch": int(msg.epoch),
            "reset_counter": int(msg.reset_counter), "quality": int(msg.quality),
            "x": _finite(p.x), "y": _finite(p.y), "z": _finite(p.z)})

    def _on_alignment(self, msg):
        with self._lock:
            self.alignment_state = int(msg.state)
        self._record("alignment", self._arrival(), {
            "stamp_ns": sitl_probe.stamp_ns(msg.stamp), "state": int(msg.state),
            **{k: _finite(getattr(msg, k)) for k in ("x_m", "y_m", "z_m", "yaw_rad", "raw_x_m", "raw_y_m", "raw_z_m",
                                                      "raw_yaw_rad", "age_s", "residual_position_m",
                                                      "residual_yaw_rad")}})

    def _on_ev(self, msg):
        self._record("vehicle_visual_odometry", self._arrival(), {
            "timestamp_us": int(msg.timestamp), "timestamp_sample_us": int(msg.timestamp_sample),
            "x": _finite(msg.position[0]), "y": _finite(msg.position[1]), "z": _finite(msg.position[2]),
            "reset_counter": int(msg.reset_counter), "quality": int(msg.quality)})

    def _on_position(self, msg):
        arrival = self._arrival()
        record = {"timestamp_us": int(msg.timestamp), "timestamp_sample_us": int(msg.timestamp_sample),
                  "xy_valid": bool(msg.xy_valid), "z_valid": bool(msg.z_valid), "x": _finite(msg.x),
                  "y": _finite(msg.y), "z": _finite(msg.z), "vz": _finite(msg.vz), "heading": _finite(msg.heading),
                  "xy_reset_counter": int(msg.xy_reset_counter), "z_reset_counter": int(msg.z_reset_counter),
                  "heading_reset_counter": int(msg.heading_reset_counter)}
        with self._lock:  # raw floats for the flight logic, as the probe keeps them
            self.position = dict(record, arrival_ns=arrival, x=float(msg.x), y=float(msg.y), z=float(msg.z))
        self._record("vehicle_local_position", arrival, record)

    def _on_flags(self, msg):
        with self._lock:
            self.ev_pos = bool(msg.cs_ev_pos)
        self._record("estimator_status_flags", self._arrival(), {
            "timestamp_us": int(msg.timestamp), "cs_ev_pos": bool(msg.cs_ev_pos), "cs_ev_yaw": bool(msg.cs_ev_yaw),
            "cs_ev_hgt": bool(msg.cs_ev_hgt), "cs_ev_vel": bool(msg.cs_ev_vel), "cs_gnss_pos": bool(msg.cs_gnss_pos)})

    def _on_status(self, msg):
        arrival = self._arrival()
        record = {"timestamp_us": int(msg.timestamp), "nav_state": int(msg.nav_state),
                  "arming_state": int(msg.arming_state)}
        with self._lock:
            self.status = dict(record, arrival_ns=arrival)
        self._record("vehicle_status", arrival, record)

    def _on_ack(self, msg):
        self._record("vehicle_command_ack", self._arrival(), {
            "command": int(msg.command), "result": int(msg.result), "result_param1": int(msg.result_param1)})

    def _on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._record("ground_truth", self._arrival(), {
            "stamp_ns": sitl_probe.stamp_ns(msg.header.stamp), "x": _finite(p.x), "y": _finite(p.y),
            "z": _finite(p.z), "qw": _finite(q.w), "qx": _finite(q.x), "qy": _finite(q.y), "qz": _finite(q.z)})

    # The stick stream, snapshot, throttle and command API are the probe's (same fields, same rules).
    _stick_tick = sitl_probe.Probe._stick_tick
    snapshot = sitl_probe.Probe.snapshot
    set_throttle = sitl_probe.Probe.set_throttle
    command = sitl_probe.Probe.command

    def ready_state(self):
        with self._lock:
            return self.lio_state, self.alignment_state, self.ev_pos


class HoverFlight(sitl_probe.Flight):
    """The Task 11 phase sequence; the waits, arm, mode and land are the probe's."""

    def __init__(self, node: HoverRecorder, mode: str, budget_s: float, stop: threading.Event,
                 pattern: str = "hover"):
        super().__init__(node, mode, budget_s, stop)
        self.ready_detail = None
        self.pattern = pattern
        self.sweep_steps = []
        # The sweep keeps time for a landing from 20 m; the hover keeps the probe's reserve.
        self.reserve_s = SWEEP_LAND_S if pattern == "altitude_sweep" else sitl_probe.LAND_RESERVE_S

    def wait(self, predicate, timeout_s, reserve_s=None, **kwargs):
        return super().wait(predicate, timeout_s, self.reserve_s if reserve_s is None else reserve_s, **kwargs)

    def log(self, text):
        print(f"[sitl_hover] {text}", flush=True)

    def wait_ready(self):
        def ready():
            clock, position, _ = self.node.snapshot()
            lio, alignment, ev_pos = self.node.ready_state()
            return (clock is not None and lio == LioHealth.TRACKING and alignment == Alignment.VALID
                    and ev_pos is True and position is not None and position["xy_valid"] and position["z_valid"])
        if not self.wait(ready, sitl_probe.WAIT_READY_S):
            clock, position, _ = self.node.snapshot()
            lio, alignment, ev_pos = self.node.ready_state()
            xy = None if position is None else (position["xy_valid"], position["z_valid"])
            return False, (f"not ready within {sitl_probe.WAIT_READY_S} s: lio state {lio}, alignment {alignment}, "
                           f"cs_ev_pos {ev_pos}, xy/z valid {xy}")
        self.start_clock = self.node.snapshot()[0]
        ok, detail = super().wait_ready()  # xy/z valid holds already: samples the ground z
        return ok, f"LIO TRACKING, alignment VALID, cs_ev_pos, xy_valid; {detail}"

    def climb(self):
        return self.climb_to(sitl_probe.TARGET_HEIGHT_M, CLIMB_SIM_S, CLIMB_WALL_MAX_S)

    def climb_to(self, height_m: float, sim_s: float, wall_s: float):
        start = self.node.snapshot()[0]
        target = height_m - sitl_probe.CLIMB_DONE_M

        def step():
            h = self.height()
            if h is None or not self.armed():
                return False
            if h >= target:
                return True
            if self.node.snapshot()[0] - start > sim_s * 1e9:
                return False
            if self.mode == "posctl":  # above the 0.3 takeoff threshold until clear of the ground, then proportional
                stick = sitl_probe.TAKEOFF_STICK if h < 1.0 else min(
                    sitl_probe.TAKEOFF_STICK, max(0.0, sitl_probe.CLIMB_GAIN * (height_m - h)))
                self.node.set_throttle(stick)
            return False
        if self.mode == "hold":
            nan = float("nan")
            self.node.command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, nan, nan, nan, nan)
        ok = self.wait(lambda: step() or self.node.snapshot()[0] - start > sim_s * 1e9, wall_s, poll_s=0.02)
        self.node.set_throttle(0.0)
        h = self.height()
        ok = ok and h is not None and h >= target
        sim_s = (self.node.snapshot()[0] - start) / 1e9
        return ok, f"height {h if h is None else round(h, 3)} m after {sim_s:.2f} s of /clock time"

    def hover(self):
        self.node.set_throttle(0.0)
        start = self.node.snapshot()[0]
        ok = self.wait(lambda: self.node.snapshot()[0] - start >= HOVER_SIM_S * 1e9 or not self.armed(),
                       HOVER_WALL_MAX_S)
        end = self.node.snapshot()[0]
        self.hover_window = {"start_ns": start, "end_ns": end}
        sim_s = (end - start) / 1e9
        return ok and self.armed() and sim_s >= HOVER_SIM_S, f"{sim_s:.2f} s of /clock time"

    def sweep(self):
        """Climb in 2.5 m steps to 20 m, holding each step 10 s of /clock with centred sticks (Task 13)."""
        for target in SWEEP_TARGETS_M:
            climb_start = self.node.snapshot()[0]
            ok, detail = self.climb_to(target, SWEEP_CLIMB_SIM_S, SWEEP_CLIMB_WALL_MAX_S)
            self.node.set_throttle(0.0)
            hold_start = self.node.snapshot()[0]
            step = {"target_m": target, "ok": False, "climb_start_ns": climb_start, "hold_start_ns": hold_start,
                    "hold_end_ns": hold_start, "reached_m": self.height(), "detail": detail}
            self.sweep_steps.append(step)
            if not ok:
                return False, f"step {target} m: climb failed: {detail}"
            held = self.wait(lambda: self.node.snapshot()[0] - hold_start >= SWEEP_HOLD_SIM_S * 1e9
                             or not self.armed(), HOVER_WALL_MAX_S)
            step["hold_end_ns"] = self.node.snapshot()[0]
            hold_s = (step["hold_end_ns"] - hold_start) / 1e9
            step["ok"] = held and self.armed() and hold_s >= SWEEP_HOLD_SIM_S
            step["detail"] = f"{detail}; held {hold_s:.2f} s of /clock time"
            self.log(f"sweep step {target} m: {'ok' if step['ok'] else 'FAILED'} {step['detail']}")
            if not step["ok"]:
                return False, f"step {target} m: hold failed after {hold_s:.2f} s"
        return True, f"{len(self.sweep_steps)} steps to {SWEEP_TARGETS_M[-1]} m"

    def land(self):
        self.node.set_throttle(0.0)
        landing = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        limit_s = SWEEP_LAND_S if self.pattern == "altitude_sweep" else LAND_S

        def resend_unless_landing():  # NAV_LAND once; again only if PX4 is not in AUTO_LAND 3 s later
            if not self.nav_state_is(landing):
                self.send_land()
        ok = self.wait(lambda: not self.armed(), limit_s, reserve_s=0.0, action=resend_unless_landing, every_s=3.0)
        return ok, "disarmed" if ok else f"still armed after {limit_s} s"

    def run(self) -> bool:
        ok = self.phase("wait_ready", self.wait_ready)
        flown = False
        if ok and self.pattern == "altitude_sweep":  # POSCTL only (checked in main)
            ok = self.phase("posctl", self.set_mode) and self.phase("arm", self.arm)
            flown = self.armed()
            ok = ok and self.phase("sweep", self.sweep)
        elif ok:
            if self.mode == "posctl":
                ok = self.phase("posctl", self.set_mode) and self.phase("arm", self.arm)
                ok = ok and self.phase("climb", self.climb)
            else:  # takeoff, then Hold (DO_SET_MODE 1/4/3, the probe's set_mode)
                ok = self.phase("arm", self.arm) and self.phase("climb", self.climb)
                ok = ok and self.phase("hold", self.set_mode)
            flown = self.armed()
            ok = ok and self.phase("hover", self.hover)
        if self.arm_sent and not flown and not self.armed():
            self.wait(self.armed, 2.0, reserve_s=0.0)  # an arm sent just before a phase gave up
        if flown or self.armed():
            ok = self.phase("land", self.land) and ok
        return ok


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="S1 SITL hover driver and recorder (S1b Task 11).")
    parser.add_argument("--run-dir", required=True, type=pathlib.Path)
    parser.add_argument("--profile", required=True, choices=("gps", "no_gps"))
    parser.add_argument("--spawn-yaw-rad", type=float, default=0.0)
    parser.add_argument("--pattern", default="hover", choices=("hover", "altitude_sweep"),
                        help="hover (Task 11) or altitude_sweep (Task 13: gps profile, posctl only)")
    parser.add_argument("--hover-mode", default="posctl", choices=tuple(sitl_probe.NAV_STATE))
    parser.add_argument("--budget-s", required=True, type=float, help="wall-time budget of the whole driver")
    args = parser.parse_args(argv)
    if not args.run_dir.is_dir() or not args.budget_s > 0 or not math.isfinite(args.spawn_yaw_rad):
        parser.error("--run-dir must exist, --budget-s must be > 0 and --spawn-yaw-rad finite")
    if args.pattern == "altitude_sweep" and (args.profile != "gps" or args.hover_mode != "posctl"):
        parser.error("--pattern altitude_sweep is flown with --profile gps and --hover-mode posctl only")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = HoverRecorder(args.run_dir)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    flight = HoverFlight(node, args.hover_mode, args.budget_s, stop, args.pattern)
    ok = False
    try:
        ok = flight.run()
    finally:
        node.set_throttle(0.0)
        if flight.armed():  # interrupted or failed in the air: best-effort land before PX4 is torn down
            flight.send_land()
            time.sleep(0.2)
        executor.shutdown(timeout_sec=2.0)
        spinner.join(timeout=5.0)
        node.close()
        meta = {"profile": args.profile, "spawn_yaw_rad": args.spawn_yaw_rad, "pattern": args.pattern,
                "hover_mode": args.hover_mode, "target_height_m": sitl_probe.TARGET_HEIGHT_M,
                "ground_z_m": flight.ground_z, "hover_window": flight.hover_window, "phases": flight.phases,
                "interrupted": stop.is_set()}
        if args.pattern == "altitude_sweep":
            meta["sweep_steps"] = flight.sweep_steps
        (args.run_dir / "hover_meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.try_shutdown()
    if stop.is_set():
        return 130
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
