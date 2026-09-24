#!/usr/bin/env python3
"""Static guard for the diagnostic-only exact planner-status injection seam."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
node = (ROOT / "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp").read_text()
runner = (ROOT / "tools/runtime/runner.py").read_text()
header = (ROOT / "src/runtime/navigation_runtime/include/navigation_runtime/runtime_boundaries.hpp").read_text()
checks = {
    "default-off runtime parameter":
        '"navigation_runtime.inject_exact_optimization_failed_once", false' in node,
    "SITL-only runtime gate": 'deployment_profile_ != "sitl"' in node,
    "explicit runner enable":
        'if inject_exact_optimization_failed_once:' in runner and
        'planner_parameters["inject_exact_optimization_failed_once"] = True' in runner,
    "exact owner witness":
        'execution_authority_.isCurrentSnapshot(execution_at_solve)' in node,
    "status-only substitution":
        'result = navigation_planning::PlannerStatus::kOptimizationFailed;' in node,
    "actual product classifier":
        node.index('result = navigation_planning::PlannerStatus::kOptimizationFailed;') <
        node.index('const auto disposition = classifyPlannerResult('),
    "HG-023 validator downstream":
        node.index('const auto disposition = classifyPlannerResult(') <
        node.index('validateRetainedCommand(goal, goal_epoch, localization_epoch_at_solve,'),
    "one-shot diagnostic state": 'consumeIfEligible' in header,
    "no production YAML enable": not any(
        "inject_exact_optimization_failed_once" in p.read_text()
        for p in (ROOT / "src").rglob("*.yaml")
        if "test" not in p.parts
    ),
}
for name, passed in checks.items():
    print(f"{'PASS' if passed else 'FAIL'} {name}")
if not all(checks.values()):
    raise SystemExit(1)
