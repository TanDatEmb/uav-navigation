#!/usr/bin/env python3
"""Rebuild reports for every saved runtime-evidence session without mutating evidence.

Each session is copied to a temporary directory before ``report.build`` runs,
because report generation writes report.json, REPORT.html, and qualification
timelines beside its input. The CSV contains the runtime verdict, reasons, and
all evaluator dimensions for reproducible before/after comparisons.
"""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import report  # noqa: E402


def _json_file(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _workflow_config(workflow: str) -> Path:
    if workflow == "dataset":
        return ROOT / "config/runtime/dataset.yaml"
    if workflow in {"sim", "external-mode"}:
        return ROOT / "config/runtime/sim.yaml"
    raise ValueError(f"unsupported saved workflow: {workflow!r}")


def _dimension_axes(evaluation: Any) -> dict[str, Any]:
    if not isinstance(evaluation, dict):
        return {}
    dimensions = evaluation.get("dimensions")
    if isinstance(dimensions, dict):
        return dimensions
    # Older evaluator output has named top-level axes. Preserve the full axis
    # mapping when present instead of silently dropping it from comparisons.
    axes = evaluation.get("axes")
    return axes if isinstance(axes, dict) else {}


def rejudge_session(session: Path, workspace: Path) -> dict[str, str]:
    runtime = _json_file(session / "runtime.json")
    metadata = _json_file(session / "metadata.json")
    previous = _json_file(session / "report.json")
    workflow = str(runtime.get("workflow") or previous.get("workflow") or "")
    if not workflow:
        return {
            "session": session.as_posix(),
            "workflow": "",
            "verdict": "NOT_EVALUABLE",
            "reasons": json.dumps(["workflow missing from runtime.json/report.json"]),
            "evaluation_axes": "{}",
            "error": "workflow missing",
        }
    try:
        config_path = _workflow_config(workflow)
    except ValueError as error:
        return {
            "session": session.as_posix(),
            "workflow": workflow,
            "verdict": "NOT_EVALUABLE",
            "reasons": json.dumps([str(error)]),
            "evaluation_axes": "{}",
            "error": str(error),
        }

    with tempfile.TemporaryDirectory(prefix="uav-rejudge-") as temporary:
        copied_session = Path(temporary) / session.name
        shutil.copytree(session, copied_session, symlinks=True)
        try:
            built = report.build(
                copied_session,
                workflow,
                config_path,
                workspace,
                Path(str(runtime["px4_dir"])) if runtime.get("px4_dir") else None,
            )
            result = _json_file(copied_session / "report.json")
            if not result:
                result = built if isinstance(built, dict) else {}
            evaluation = result.get("evaluation", {})
            axes = _dimension_axes(evaluation)
            reasons = result.get("reasons", [])
            return {
                "session": session.as_posix(),
                "workflow": workflow,
                "verdict": str(result.get("verdict", "NOT_EVALUABLE")),
                "reasons": json.dumps(reasons, sort_keys=True, ensure_ascii=False),
                "evaluation_axes": json.dumps(axes, sort_keys=True, ensure_ascii=False),
                "error": "",
            }
        except Exception as error:  # keep one damaged session from hiding others
            return {
                "session": session.as_posix(),
                "workflow": workflow,
                "verdict": "NOT_EVALUABLE",
                "reasons": json.dumps([f"rejudge error: {type(error).__name__}: {error}"]),
                "evaluation_axes": "{}",
                "error": f"{type(error).__name__}: {error}",
            }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--roots", nargs="*", type=Path,
        help="evidence roots (default: runtime_evidence and artifacts)",
    )
    args = parser.parse_args()
    roots = args.roots or [ROOT / "runtime_evidence", ROOT / "artifacts"]
    sessions = sorted({path.parent.resolve() for root in roots for path in root.rglob("report.json")})
    rows = [rejudge_session(session, args.workspace.resolve()) for session in sessions]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=("session", "workflow", "verdict", "reasons", "evaluation_axes", "error"),
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)
    errors = [row for row in rows if row["error"]]
    print(f"sessions={len(rows)} errors={len(errors)} output={args.output}")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
