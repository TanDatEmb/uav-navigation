"""Fail-closed parsing for runtime waypoint acceptance evidence."""

from __future__ import annotations

from typing import Any


def parse_waypoint_acceptance(event: Any) -> tuple[str, int | None]:
    """Return (status, waypoint index); absent/ambiguous evidence is NOT_EVALUABLE."""
    if isinstance(event, dict):
        accepted = event.get("waypoint_accepted")
        if type(accepted) is not bool:
            return "NOT_EVALUABLE", None
        if not accepted:
            return "REJECTED", None
        value = event.get("accepted_waypoint_index")
    elif isinstance(event, int) and not isinstance(event, bool):
        # Compact report artifacts encode an accepted index directly.
        return "ACCEPTED", event
    else:
        return "NOT_EVALUABLE", None
    if isinstance(value, bool):
        return "NOT_EVALUABLE", None
    try:
        index = int(value)
    except (TypeError, ValueError, OverflowError):
        return "NOT_EVALUABLE", None
    if str(index) != str(value).strip() and not isinstance(value, int):
        return "NOT_EVALUABLE", None
    return "ACCEPTED", index
