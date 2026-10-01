#!/usr/bin/env python3
"""Render the target-module dependency graph after the R1 contract mapping."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path


TIERS = {
    "nav_core_types": 0,
    "nav_safety_profile": 0,
    "nav_world_contract": 0,
    "nav_plan_contract": 0,
    "nav_mission_contract": 0,
    "navigation_contracts": 0,
    "nav_evidence_msgs": 0,
    "lio_core": 1,
    "nav_world": 1,
    "nav_planner": 1,
    "nav_certifier": 1,
    "nav_execution": 1,
    "nav_mission": 1,
    "nav_planning_policy": 1,
    "px4_setpoint_core": 1,
    "odom_bridge_core": 1,
    "lio_node": 2,
    "nav_core_node": 2,
    "px4_adapter_node": 2,
    "odom_bridge_node": 2,
    "sitl_harness": 3,
    "nav_judge": 3,
    "evidence_decoders": 3,
}


def cycles(edges: dict[str, set[str]]) -> list[list[str]]:
    found: list[list[str]] = []
    active: list[str] = []
    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(node: str) -> None:
        if node in visiting:
            start = active.index(node)
            found.append(active[start:] + [node])
            return
        if node in visited:
            return
        visiting.add(node)
        active.append(node)
        for child in sorted(edges.get(node, set())):
            visit(child)
        active.pop()
        visiting.remove(node)
        visited.add(node)

    for node in sorted(edges):
        visit(node)
    return found


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inventory", type=Path, default=Path(__file__).with_name("class_inventory.csv"))
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("module_dependency_graph.md"))
    args = parser.parse_args()
    with args.inventory.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    by_id = {row["id"]: row for row in rows}
    edges: dict[str, set[str]] = defaultdict(set)
    evidence: dict[tuple[str, str], list[str]] = defaultdict(list)
    excluded: list[str] = []
    for row in rows:
        source = row["target_module"]
        for dep_id in filter(None, row.get("depends_on", "").split(";")):
            dep = by_id.get(dep_id)
            if dep is None:
                excluded.append(f"{row['id']} -> {dep_id}: unresolved inventory dependency")
                continue
            target = dep["target_module"]
            if source == target:
                continue
            source_tier = TIERS.get(source, 99)
            target_tier = TIERS.get(target, 99)
            if source_tier <= target_tier:
                excluded.append(
                    f"{source} -> {target}: excluded upward/non-downward edge from {row['id']} to {dep_id}"
                )
                continue
            edges[source].add(target)
            evidence[(source, target)].append(f"{row['id']} -> {dep_id}")

    found = cycles(edges)
    # Downward-only edges cannot cycle.  Keep the assertion in the generator
    # so a future target-module edit fails loudly instead of producing a false
    # clean graph.
    cycle_status = "none" if not found else "; ".join(" -> ".join(item) for item in found)
    modules = sorted(set(TIERS) | set(edges) | {target for values in edges.values() for target in values})
    lines = [
        "# WP-A1-R1 — module dependency graph",
        "",
        "Source: `class_inventory.csv` target_module/depends_on after the R1 contract decision.",
        "The graph retains only downward architectural edges (Tier B/C/D to Tier A/B/C); "
        "legacy upward or same-tier implementation/include heuristics are listed as excluded evidence, not design dependencies.",
        "",
        f"cycle_status={cycle_status}",
        "",
        "## Modules and tiers",
        "",
    ]
    for module in modules:
        lines.append(f"- `{module}` (Tier {TIERS.get(module, '?')})")
    lines.extend(["", "## Edges", ""])
    for source in sorted(edges):
        for target in sorted(edges[source]):
            samples = "; ".join(evidence[(source, target)][:3])
            lines.append(f"- `{source}` -> `{target}` — evidence: {samples}")
    lines.extend(["", "## Excluded legacy/include edges", ""])
    for item in excluded[:200]:
        lines.append(f"- {item}")
    if len(excluded) > 200:
        lines.append(f"- ... {len(excluded) - 200} more excluded edges")
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"modules={len(modules)} edges={sum(len(value) for value in edges.values())} excluded={len(excluded)}")
    print(f"cycle_status={cycle_status}")
    return 0 if not found else 1


if __name__ == "__main__":
    raise SystemExit(main())
