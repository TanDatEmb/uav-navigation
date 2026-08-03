#!/usr/bin/env python3
"""Run the P0.8 fault matrix against a real supervisor process.

Every process is started in its own process group and is terminated through
that group only; this runner never uses a global process-name kill.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from odometry_supervisor_fault_injector import EXPECTED_FINAL_HEALTH, SCENARIOS


DURATIONS = {
    "healthy": 6.0,
    "single_position_jump": 6.0,
    "slow_xy_drift": 12.0,
    "slow_yaw_drift": 12.0,
    "velocity_bias": 6.0,
    "px4_stale": 8.0,
    "px4_diagnostics_stale": 8.0,
    "lio_propagated_stale": 8.0,
    "lio_corrected_stale": 8.0,
    "lio_diagnostics_stale": 8.0,
    "px4_reset_generation": 8.0,
    "px4_time_generation": 8.0,
    "clock_pause": 8.0,
    "diagnostic_schema_corruption": 8.0,
    "lio_lost": 6.0,
    "correlated_unhealthy": 6.0,
}


def stop_process(process: subprocess.Popen | None) -> int | None:
    if process is None or process.poll() is not None:
        return None if process is None else process.returncode
    try:
        os.killpg(process.pid, signal.SIGINT)
        return process.wait(timeout=5)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGTERM)
            return process.wait(timeout=3)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            return process.wait(timeout=3)


def run_scenario(scenario: str, output: Path, domain_id: int) -> dict:
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain_id)
    injector_command = [
        sys.executable,
        str(Path(__file__).with_name("odometry_supervisor_fault_injector.py")),
        "--scenario",
        scenario,
        "--duration",
        str(DURATIONS[scenario]),
        "--output",
        str(output),
    ]
    if scenario == "clock_pause":
        injector_command += ["--use-sim-time"]
    injector = subprocess.Popen(
        injector_command,
        env=environment,
        start_new_session=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    supervisor_command = [
        "ros2",
        "run",
        "odometry_supervisor",
        "odometry_supervisor_node",
        "--ros-args",
        "-p",
        "use_sim_time:=true" if scenario == "clock_pause" else "use_sim_time:=false",
    ]
    if scenario == "correlated_unhealthy":
        supervisor_command += ["-p", "reference_mode:=correlated"]
    supervisor = None
    try:
        # With simulated time, wait for the first /clock and diagnostic samples
        # before the supervisor's persistence timers begin at zero.
        time.sleep(0.9 if scenario == "clock_pause" else 0.25)
        supervisor = subprocess.Popen(
            supervisor_command,
            env=environment,
            start_new_session=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            injector_output, _ = injector.communicate(timeout=DURATIONS[scenario] + 12)
        except subprocess.TimeoutExpired:
            stop_process(injector)
            injector_output = "injector timeout\n"
        supervisor_exit = stop_process(supervisor)
        if not output.exists():
            return {"scenario": scenario, "oracle": {"pass": False,
                    "failure_reason": "injector did not produce artifact"},
                    "injector_exit_code": injector.returncode,
                    "supervisor_exit_code": supervisor_exit,
                    "injector_output_tail": injector_output[-2000:]}
        artifact = json.loads(output.read_text(encoding="utf-8"))
        artifact["runtime"] = {
            "injector_exit_code": injector.returncode,
            "supervisor_exit_code": supervisor_exit,
            "cleanup": "completed",
            "ros_domain_id": domain_id,
        }
        artifact["runtime"]["expected_final_health"] = sorted(EXPECTED_FINAL_HEALTH[scenario])
        output.write_text(json.dumps(artifact, indent=2) + "\n", encoding="utf-8")
        return artifact
    finally:
        stop_process(supervisor)
        stop_process(injector)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--scenario", action="append", choices=SCENARIOS)
    args = parser.parse_args()
    scenarios = args.scenario or list(SCENARIOS)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    failures = []
    for offset, scenario in enumerate(scenarios):
        output = args.output_dir / f"{scenario}.json"
        artifact = run_scenario(scenario, output, 80 + (os.getpid() + offset) % 100)
        if not artifact.get("oracle", {}).get("pass", False):
            failures.append(scenario)
        print(json.dumps({"scenario": scenario, "oracle": artifact.get("oracle"),
                          "runtime": artifact.get("runtime")}, sort_keys=True))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
