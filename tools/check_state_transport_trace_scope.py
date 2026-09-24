#!/usr/bin/env python3
"""Keep the per-sequence state timing trace diagnostic and opt-in."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PRODUCER = (ROOT / "src/estimation/fast_lio_ros/src/ros_propagated_odometry_publisher.cpp").read_text()
ESTIMATOR_PARAMETERS = (ROOT / "src/estimation/fast_lio_ros/src/parameter_loader.cpp").read_text()
ADAPTER = (ROOT / "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp").read_text()
RUNNER = (ROOT / "tools/runtime/runner.py").read_text()
MESSAGE = (ROOT / "src/contracts/navigation_contracts/msg/OdometryTransportTrace.msg").read_text()

assert '"diagnostics.state_transport_trace_enabled", false' in ESTIMATOR_PARAMETERS
for name, source, publisher in (("producer", PRODUCER, "timing_publisher_"),
                                ("adapter", ADAPTER, "odometry_timing_publisher_")):
    assert '"diagnostics.state_transport_trace_enabled"' in source, name
    if name == "adapter":
        assert '"diagnostics.state_transport_trace_enabled", false' in source
    assert '"use_sim_time"' in source, name
    assert "state transport trace is SITL/test only" in source, name
    assert publisher in source, name

assert "state_transport_trace: bool = False" in RUNNER
assert '"state_transport_trace": bool(state_transport_trace)' in RUNNER
assert "state_transport_trace and control_interface != \"external_mode\"" in RUNNER
assert "Diagnostic-only timing witness. Never consumed by navigation admission." in MESSAGE

for path in (ROOT / "src").rglob("*.cpp"):
    if path.name in {"navigation_mode_node.cpp", "ros_propagated_odometry_publisher.cpp"}:
        continue
    assert "OdometryTransportTrace" not in path.read_text(), path

print("state transport trace scope guard: PASS")
