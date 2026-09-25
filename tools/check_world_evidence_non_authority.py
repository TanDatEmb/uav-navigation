#!/usr/bin/env python3
"""Ensure World evidence plumbing cannot become a product authority path."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def between(text: str, start: str, end: str) -> str:
    first = text.find(start)
    if first < 0:
        raise ValueError(f"missing source marker: {start}")
    last = text.find(end, first + len(start))
    if last < 0:
        raise ValueError(f"missing source marker: {end}")
    return text[first:last]


def main() -> int:
    errors: list[str] = []
    runtime = (ROOT / "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp").read_text(
        encoding="utf-8")
    evaluator = (ROOT / "tools/runtime/evaluation.py").read_text(encoding="utf-8")
    recorder = (ROOT / "tools/runtime/external_mode_scenario.py").read_text(encoding="utf-8")
    gate = (ROOT / "tools/runtime/world_observation_gate.py").read_text(encoding="utf-8")
    runner = (ROOT / "tools/runtime/runner.py").read_text(encoding="utf-8")
    try:
        producer = between(
            runtime,
            "void NavigationRuntimeNode::publishWorldTransactionWitness(",
            "std::optional<PlanningKey> NavigationRuntimeNode::currentPlanningKey()",
        )
        reducer = between(
            evaluator,
            "def reduce_world_transactions(",
            "def load_evaluation_inputs(",
        )
        observer = between(
            recorder,
            'if status.name == "navigation_runtime/world_transaction_witness":',
            'elif status.name == "navigation_mapping/world_model":',
        )
    except ValueError as error:
        errors.append(str(error))
    else:
        for forbidden in (
            "command_publisher_", "execution_authority_.", "world_snapshot_store_.",
            "failClosed", "invalidateIfCurrent", "publishWorldIdentity",
        ):
            if forbidden in producer:
                errors.append(f"World witness producer reaches authority API: {forbidden}")
        if "publisher->publish(message)" not in producer:
            errors.append("World witness is not emitted through the diagnostic publisher")
        for forbidden in (
            "ExecutionAuthority(", "ExecutionAuthority.", "WorldSnapshotStore(",
            "NavigationCommand(", "create_publisher", "create_client",
            "failClosed(", "invalidateIfCurrent(",
        ):
            if forbidden in reducer:
                errors.append(f"World evidence reducer references authority API: {forbidden}")
        for forbidden in ("create_publisher", "create_client", "create_service", ".publish("):
            if forbidden in observer:
                errors.append(f"World evidence observer has control side effect: {forbidden}")
        if 'create_publisher(\n            RegisteredScan' not in gate:
            errors.append("test gate must publish only RegisteredScan")
        if 'NavigationCommand, str(self.get_parameter("command_topic")' not in gate:
            errors.append("test gate must trigger from explicit NavigationCommand witness")
        if "NavigationCommand, " in gate and "create_publisher(\n            NavigationCommand" in gate:
            errors.append("test gate must never publish NavigationCommand")
        if "registered_scan_topic\"] = \"/test/world_gate/mapping_observation\"" not in runner:
            errors.append("runner does not remap only the Core mapping input for the opt-in fault")
        if "world_observation_fault_duration_ms: int = 0" not in runner:
            errors.append("World fault harness is not default-disabled")
    if errors:
        print("WORLD_EVIDENCE_NON_AUTHORITY_GUARD: FAIL")
        for error in errors:
            print(f"- {error}")
        return 1
    print("WORLD_EVIDENCE_NON_AUTHORITY_GUARD: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
