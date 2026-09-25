#!/usr/bin/env python3
"""Guard the explicit SITL tracking mode and pre-mission witness boundary."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
loader = (ROOT / "src/contracts/navigation_contracts/include/navigation_contracts/tracking_experiment.hpp").read_text()
runner = (ROOT / "tools/runtime/runner.py").read_text()
runtime = (ROOT / "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp").read_text()
adapter = (ROOT / "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp").read_text()

checks = {
    "explicit mode parameter": '"tracking_experiment.mode", "off"' in loader,
    "sim time cannot enable experiment": "p.enabled = simulated" not in loader
    and 'p.enabled = mode != "off"' in loader,
    "runner writes mode": '"mode": str(experiment["mode"])' in runner,
    "runner checks live Core": '_check_effective_tracking_configuration(\n                session, "mapping"' in runner,
    "runner checks live adapter": '_check_effective_tracking_configuration(\n                    session, "external_mode"' in runner,
    "Core emits effective witness": "RUNTIME_CONFIG_EFFECTIVE tracking_mode=" in runtime,
    "adapter emits effective witness": "RUNTIME_CONFIG_EFFECTIVE tracking_mode=" in adapter,
    "Core and adapter expose numeric bounds":
        "RUNTIME_CONFIG_EFFECTIVE tracking_bounds" in runtime
        and "RUNTIME_CONFIG_EFFECTIVE tracking_bounds" in adapter,
    "Runtime exposes planner configuration":
        "RUNTIME_CONFIG_EFFECTIVE planner_fault" in runtime
        and "RUNTIME_CONFIG_EFFECTIVE dynamics" in runtime
        and "_check_effective_planner_configuration(" in runner,
}
for label, passed in checks.items():
    print(f"{'PASS' if passed else 'FAIL'} {label}")
sys.exit(0 if all(checks.values()) else 1)
