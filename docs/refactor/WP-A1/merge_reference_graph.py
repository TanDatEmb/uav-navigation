#!/usr/bin/env python3
"""Merge isolated reference_graph.py runs without losing caller counts."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--inventory", type=Path)
    parser.add_argument("graphs", type=Path, nargs="+")
    args = parser.parse_args()
    merged: dict[str, dict[str, str]] = {}
    fields: list[str] = []
    for graph in args.graphs:
        with graph.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        if not fields:
            fields = list(rows[0])
        for row in rows:
            if row["id"] not in merged:
                merged[row["id"]] = dict(row)
                continue
            item = merged[row["id"]]
            for field in ("prod_callers_other_tu", "prod_callers_same_tu", "test_callers"):
                item[field] = str(int(item.get(field, 0) or 0) + int(row.get(field, 0) or 0))
            item["definition_seen_in_ast"] = str(
                item.get("definition_seen_in_ast") == "True"
                or row.get("definition_seen_in_ast") == "True"
            )
            sites = []
            for value in (item.get("reference_sites", ""), row.get("reference_sites", "")):
                sites.extend(item for item in value.split(";") if item)
            item["reference_sites"] = ";".join(list(dict.fromkeys(sites))[:40])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(merged.values())
    if args.inventory:
        with args.inventory.open(newline="", encoding="utf-8") as stream:
            inventory_rows = list(csv.DictReader(stream))
        inventory_fields = [
            field
            for field in list(inventory_rows[0])
            if field not in {"prod_callers", "prod_callers_other_tu", "prod_callers_same_tu", "test_callers", "reference_sites", "definition_seen_in_ast"}
        ]
        insert_at = inventory_fields.index("depends_on") + 1
        inventory_fields[insert_at:insert_at] = [
            "prod_callers_other_tu",
            "prod_callers_same_tu",
            "test_callers",
            "reference_sites",
            "definition_seen_in_ast",
        ]
        for row in inventory_rows:
            graph = merged[row["id"]]
            row.pop("prod_callers", None)
            for field in ("prod_callers_other_tu", "prod_callers_same_tu", "test_callers", "reference_sites", "definition_seen_in_ast"):
                row[field] = graph[field]
        with args.inventory.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=inventory_fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(inventory_rows)
        print(f"inventory_updated={args.inventory}")
    print(f"graphs_merged={len(args.graphs)} symbols={len(merged)} output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
