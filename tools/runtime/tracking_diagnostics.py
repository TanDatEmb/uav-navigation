"""Shared metadata for non-authoritative tracking diagnostics."""

from __future__ import annotations

from typing import Any


TRACKING_AUTHORITY = "diagnostic"
TRACKING_FRAME_STATUS = "diagnostic_frame_unverified"


def _annotate_tree(value: Any) -> None:
    if isinstance(value, dict):
        for child in list(value.values()):
            _annotate_tree(child)
        value["authority"] = TRACKING_AUTHORITY
        value["frame_status"] = TRACKING_FRAME_STATUS
    elif isinstance(value, list):
        for child in value:
            _annotate_tree(child)


def annotate_tracking_metrics(metrics: dict[str, Any]) -> dict[str, Any]:
    """Tag every nested tracking metric as diagnostic and frame-unverified."""
    tracking = metrics.get("tracking")
    if isinstance(tracking, dict):
        _annotate_tree(tracking)
    return metrics


def annotate_tracking_report(report: dict[str, Any]) -> dict[str, Any]:
    """Tag report.json tracking data without changing verdict inputs."""
    tracking = report.get("tracking")
    if isinstance(tracking, dict):
        _annotate_tree(tracking)

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
                    and (str(metric_id).startswith("tracking.") or "cross_track" in str(metric_id))
                ):
                    _annotate_tree(metric)

    mission_outcome = report.get("mission_outcome")
    if isinstance(mission_outcome, dict):
        mission_acceptance = mission_outcome.get("acceptance")
        if isinstance(mission_acceptance, dict):
            value = mission_acceptance.get("cross_track_error_p95_m")
            mission_acceptance["cross_track_diagnostic"] = {
                "value_m": value,
                "authority": TRACKING_AUTHORITY,
                "frame_status": TRACKING_FRAME_STATUS,
            }
            mission_acceptance["cross_track_error_p95_m_authority"] = TRACKING_AUTHORITY
            mission_acceptance["cross_track_error_p95_m_frame_status"] = TRACKING_FRAME_STATUS
    return report
