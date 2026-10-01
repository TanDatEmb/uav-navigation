#!/usr/bin/env python3
"""Print src create_* calls absent from the WP-A2 topic manifest."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_icd import OUT, scan_ros_calls, topic_endpoint_locations  # noqa: E402

import yaml  # noqa: E402


def main() -> int:
    try:
        inventory = yaml.safe_load((OUT / "icd_topics.yaml").read_text(encoding="utf-8"))
    except Exception as exc:  # pragma: no cover - command-line diagnostic
        print(f"extract_check: cannot parse icd_topics.yaml: {exc}", file=sys.stderr)
        return 2
    anchors = topic_endpoint_locations(inventory)
    sites = scan_ros_calls()
    missing = [(site, kind) for site, kind in sites if site not in anchors]
    for site, kind in missing:
        print(f"MISSING {kind}: {site}")
    if missing:
        print(f"extract_check: {len(missing)} missing of {len(sites)} create_* sites")
        return 1
    print(f"extract_check: no missing interfaces ({len(sites)} create_* sites)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
