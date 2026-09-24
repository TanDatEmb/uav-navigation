#!/usr/bin/env python3
"""Focused long_featured diagnostic run with explicit injection/arming assertion."""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/runtime"))
import runner  # noqa: E402

ANALYZER = (ROOT / "artifacts/exact_optimization_failed_evidence/"
            "20260924T004835Z-b640a25d/analyze_run.py")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nominal", action="store_true")
    parser.add_argument("--label", required=True)
    parser.add_argument("--session", type=Path,
                        help="analyze an existing session without running SITL")
    args = parser.parse_args()
    if args.session is None:
        previous = set(runner._runtime_session_paths(runner.ARTIFACT_ROOT))
        kwargs = dict(control_interface="external_mode", map_profile="long_featured",
                      map_seed=0, tracking_experiment_mode="off",
                      sitl_dynamics_profile="off", experiment_id=args.label)
        if not args.nominal:
            kwargs.update(inject_exact_optimization_failed_once=True,
                          inject_exact_optimization_predecessor_request=2,
                          inject_exact_optimization_successor_request=3)
        # The runner's versioned flight-qualification verdict can be FAIL even
        # for a completed focused mission. Analyze the raw bag independently.
        runner_status = runner.run_sim(True, **kwargs)
        created = set(runner._runtime_session_paths(runner.ARTIFACT_ROOT)) - previous
        if len(created) != 1:
            print(json.dumps({"result":"SESSION_COUNT_ERROR","created":list(map(str,created)),
                              "runner_status":runner_status}), file=sys.stderr)
            return 2
        session = created.pop()
    else:
        session = args.session.resolve()
        runner_status = None
    spec = importlib.util.spec_from_file_location("exact_failure_analyzer", ANALYZER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load analyzer {ANALYZER}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = module.analyze(session, injected=not args.nominal)
    output = session / "exact_optimization_failure_analysis.json"
    output.write_text(json.dumps(result,indent=2,sort_keys=True)+"\n")
    print(json.dumps({"label":args.label,"session":str(session),
                      "focused_verdict":result["verdict"],
                      "injection_state":result["injection_state"],
                      "runner_status":runner_status,
                      "checks":result["checks"],
                      "analysis":str(output)},sort_keys=True))
    return 0 if result["verdict"] == "FOCUSED_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
