"""Re-evaluate the pinned ten-run cohort from raw JSONL, never report metrics.

Run from repository root: python3 artifacts/qualification_evidence_contract/
20260924T153924Z-748b8e39/reevaluate_raw.py
"""

import csv
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools/runtime"))
from evaluation import evaluate_session, load_evaluation_inputs  # noqa: E402

ARTIFACT = Path(__file__).resolve().parent
COHORT = ROOT / (
    "artifacts/qualification_product_stability/"
    "20260924T140134Z-dee89a7e/NOMINAL_COHORT.csv"
)
FIELDS = (
    "run", "original_outcome", "original_assessment", "reevaluated_assessment",
    "eligible", "valid_lifecycle", "unresolved_lifecycle", "conflicting_lifecycle",
    "unbound_goal_requests", "unbound_activations",
    "reference_lineage", "position_p95_m", "position_max_m",
    "velocity_p95_mps", "velocity_max_mps", "blocking_reasons", "session",
)


def main() -> None:
    rows = []
    lifecycle_rows = []
    with COHORT.open(newline="", encoding="utf-8") as stream:
        cohort = list(csv.DictReader(stream))
    for item in cohort:
        session = Path(item["session"])
        if not session.is_dir():
            raise FileNotFoundError(session)
        original = json.loads((session / "report.json").read_text(encoding="utf-8"))
        old_evaluation = original["evaluation"]
        current = evaluate_session(load_evaluation_inputs(session))
        lifecycle = current["lifecycle_reduction"]
        for transaction in lifecycle["unresolved"]:
            reasons = sorted(set(transaction["reasons"]))
            if "BUNDLE_OWNER_MISSING_EXPORT" in reasons:
                structural_class = "MISSING_PRODUCER_EXPORT_IDENTITY"
            elif "BUNDLE_OWNER_AMBIGUOUS_EXPORT" in reasons:
                structural_class = "AMBIGUOUS_EXPORT_IDENTITY"
            elif "BUNDLE_IDENTITY_CONFLICT" in reasons:
                structural_class = "CROSS_PHASE_BUNDLE_IDENTITY_CONFLICT"
            elif "BUNDLE_IDENTITY_MISSING" in reasons:
                structural_class = "MISSING_BUNDLE_IDENTITY"
            elif any(reason.startswith("LIFECYCLE_PHASE_INCOMPLETE:") for reason in reasons):
                structural_class = "MISSING_CONSUMER_OR_SUPERSESSION_WITNESS"
            else:
                structural_class = "OTHER_UNRESOLVED_EVIDENCE"
            lifecycle_rows.append({
                "run": item["run"],
                "request_id": transaction["identity"].get("request_id"),
                "planning_cycle_id": transaction["identity"].get(
                    "causal_planning_cycle_id"),
                "structural_class": structural_class,
                "observed_phases": "|".join(sorted(transaction["events"])),
                "reasons": "|".join(reasons),
            })
        position = current["metrics"].get("tracking.navigation_reference_vs_truth", {})
        velocity = current["metrics"].get(
            "tracking.navigation_reference_vs_truth.velocity", {})
        rows.append({
            "run": item["run"],
            "original_outcome": item["result"],
            "original_assessment": old_evaluation["assessment_status"],
            "reevaluated_assessment": current["assessment_status"],
            "eligible": current["qualification_eligible"],
            "valid_lifecycle": lifecycle["valid_transaction_count"],
            "unresolved_lifecycle": len(lifecycle["unresolved"]),
            "conflicting_lifecycle": len(lifecycle["conflicts"]),
            "unbound_goal_requests": sum(
                event["phase"] == "request" for event in lifecycle["unbound_events"]),
            "unbound_activations": sum(
                event["phase"] == "activate" for event in lifecycle["unbound_events"]),
            "reference_lineage": position.get("qualification_checks", {}).get(
                "reference_lineage_valid"),
            "position_p95_m": position.get("p95"),
            "position_max_m": position.get("maximum"),
            "velocity_p95_mps": velocity.get("p95"),
            "velocity_max_mps": velocity.get("maximum"),
            "blocking_reasons": "|".join(current["blocking_reasons"]),
            "session": str(session),
        })
    with (ARTIFACT / "RAW_SESSION_REEVALUATION.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    with (ARTIFACT / "LIFECYCLE_UNRESOLVED_CLASSES.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            "run", "request_id", "planning_cycle_id", "structural_class",
            "observed_phases", "reasons",
        ), lineterminator="\n")
        writer.writeheader()
        writer.writerows(lifecycle_rows)
    print(f"runs={len(rows)} unresolved={sum(int(r['unresolved_lifecycle']) for r in rows)} "
          f"eligible={sum(r['eligible'] for r in rows)}")


if __name__ == "__main__":
    main()
