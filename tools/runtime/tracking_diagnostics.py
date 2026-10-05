"""Shared metadata for non-authoritative tracking diagnostics."""

from __future__ import annotations

from copy import deepcopy
from typing import Any


TRACKING_AUTHORITY = "diagnostic"
TRACKING_FRAME_STATUS = "diagnostic_frame_unverified"


def _annotate_tree(value: Any) -> Any:
    """Return an annotated copy, leaving evaluator-owned input untouched."""
    annotated = deepcopy(value)

    def visit(current: Any) -> None:
        if isinstance(current, dict):
            for child in list(current.values()):
                visit(child)
            current["authority"] = TRACKING_AUTHORITY
            current["frame_status"] = TRACKING_FRAME_STATUS
        elif isinstance(current, list):
            for child in current:
                visit(child)

    visit(annotated)
    return annotated


def _is_non_gate_metric(metric_id: Any) -> bool:
    metric_name = str(metric_id)
    return metric_name.startswith("tracking.") and not metric_name.startswith(
        "tracking.navigation_reference_vs_truth"
    )


def _is_cross_track_metric(metric_id: Any) -> bool:
    metric_name = str(metric_id).lower()
    return "cross_track" in metric_name or "guidance_deviation" in metric_name


def annotate_tracking_metrics(metrics: dict[str, Any]) -> dict[str, Any]:
    """Tag every nested tracking metric as diagnostic and frame-unverified."""
    tracking = metrics.get("tracking")
    if isinstance(tracking, dict):
        metrics["tracking"] = _annotate_tree(tracking)
    return metrics


def annotate_tracking_report(report: dict[str, Any]) -> dict[str, Any]:
    """Tag report.json tracking data without changing verdict inputs."""
    tracking = report.get("tracking")
    if isinstance(tracking, dict):
        report["tracking"] = _annotate_tree(tracking)

    acceptance = report.get("acceptance")
    if isinstance(acceptance, dict):
        value = acceptance.get("cross_track_error_p95_m")
        acceptance["cross_track_diagnostic"] = {
            "value_m": value,
            "authority": TRACKING_AUTHORITY,
            "frame_status": TRACKING_FRAME_STATUS,
        }
        acceptance["cross_track_error_p95_m_authority"] = TRACKING_AUTHORITY
        acceptance["cross_track_error_p95_m_frame_status"] = TRACKING_FRAME_STATUS

    evaluation = report.get("evaluation")
    if isinstance(evaluation, dict):
        metrics = evaluation.get("metrics")
        if isinstance(metrics, dict):
            for metric_id, metric in metrics.items():
                if (
                    isinstance(metric, dict)
                    and (_is_non_gate_metric(metric_id) or _is_cross_track_metric(metric_id))
                ):
                    metrics[metric_id] = _annotate_tree(metric)

    mission_outcome = report.get("mission_outcome")
    if isinstance(mission_outcome, dict):
        mission_acceptance = mission_outcome.get("acceptance")
        if isinstance(mission_acceptance, dict):
            value = mission_acceptance.get("cross_track_error_p95_m")
            mission_outcome["acceptance"] = deepcopy(mission_acceptance)
            mission_acceptance = mission_outcome["acceptance"]
            mission_acceptance["cross_track_diagnostic"] = {
                "value_m": value,
                "authority": TRACKING_AUTHORITY,
                "frame_status": TRACKING_FRAME_STATUS,
            }
            mission_acceptance["cross_track_error_p95_m_authority"] = TRACKING_AUTHORITY
            mission_acceptance["cross_track_error_p95_m_frame_status"] = TRACKING_FRAME_STATUS
    return report
