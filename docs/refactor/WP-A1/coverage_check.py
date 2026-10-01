#!/usr/bin/env python3
"""Check that every in-scope product C/C++ file is represented in the inventory.

The inventory's ``file`` column is the definition anchor.  A C++ implementation
file for an out-of-line member is additionally represented by the ``evidence``
column of the owning class row; this avoids inventing a second symbol for every
member implementation file while still making coverage auditable.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path


SUFFIXES = {".h", ".hpp", ".cpp", ".cc", ".cxx"}


def product_files(repo: Path) -> set[str]:
    result: set[str] = set()
    for path in (repo / "src").rglob("*"):
        if path.suffix not in SUFFIXES or not path.is_file():
            continue
        parts = path.parts
        if "external" in parts or "test" in parts or any("_vendor" in p for p in parts):
            continue
        result.add(path.relative_to(repo).as_posix())
    return result


def inventory_files(inventory: Path) -> set[str]:
    result: set[str] = set()
    with inventory.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("file"):
                result.add(row["file"])
            for evidence in row.get("evidence", "").split(";"):
                if ":" not in evidence:
                    continue
                result.add(evidence.rsplit(":", 1)[0])
    return result


def expected_symbol_rows(repo: Path) -> int:
    """Re-run the same type/free-function grouping used by the builder.

    Keeping this check source-based makes the ±10% guard meaningful without
    requiring ROS headers.  The builder remains the authority for the richer
    metadata columns; this check only counts rows and file coverage.
    """
    builder_dir = Path(__file__).resolve().parent
    sys.path.insert(0, str(builder_dir))
    import inventory_builder  # pylint: disable=import-outside-toplevel

    files = inventory_builder.product_files()
    count = 0
    for path in files:
        text = path.read_text(errors="ignore")
        found = inventory_builder.find_type_definitions(path, text)
        count += len(found)
        if not found or inventory_builder.top_level_function_group(path, text, found):
            count += 1
    return count


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path(__file__).parents[3])
    parser.add_argument(
        "--inventory",
        type=Path,
        default=Path(__file__).with_name("class_inventory.csv"),
    )
    parser.add_argument(
        "--graph",
        type=Path,
        default=Path(__file__).with_name("reference_graph.csv"),
    )
    parser.add_argument(
        "--module-graph",
        type=Path,
        default=Path(__file__).with_name("module_dependency_graph.md"),
    )
    args = parser.parse_args()
    expected = product_files(args.repo.resolve())
    covered = inventory_files(args.inventory.resolve())
    missing = sorted(expected - covered)
    inventory_rows = sum(1 for _ in csv.DictReader(args.inventory.open(newline="", encoding="utf-8")))
    expected_rows = expected_symbol_rows(args.repo.resolve())
    delta = (inventory_rows - expected_rows) / expected_rows if expected_rows else 0.0
    print(
        f"product_files={len(expected)} covered_files={len(expected - set(missing))} "
        f"expected_symbols={expected_rows} inventory_rows={inventory_rows} "
        f"delta={delta:+.2%}"
    )
    for path in missing:
        print(path)

    failures: list[str] = []
    with args.inventory.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    required_fields = {
        "prod_callers_other_tu",
        "prod_callers_same_tu",
        "test_callers",
        "reference_sites",
        "definition_seen_in_ast",
    }
    missing_fields = sorted(required_fields - set(rows[0])) if rows else sorted(required_fields)
    if missing_fields:
        failures.append("missing_inventory_fields=" + ",".join(missing_fields))

    generic = re.compile(
        r"^(Stores|Generates|Coordinates|Classifies|Carries|Implements stateful or decision logic in) \S+\.?$"
    )
    for row in rows:
        sentence = row.get("responsibility", "").strip()
        tail = row["id"].rsplit("::", 1)[-1]
        if not sentence or generic.match(sentence) or sentence.rstrip(".").endswith(tail):
            failures.append(f"responsibility={row['id']}")
        if row.get("kind") in {"class", "struct_logic", "free_fn_group", "template"} and not sentence:
            failures.append(f"required_responsibility={row['id']}")
        if row.get("kind") in {"struct_pod", "enum"}:
            if len(re.findall(r"`[^`]+`", sentence)) < 2 or "producer" not in sentence.lower():
                failures.append(f"pod_enum_fields_or_producer={row['id']}")
        try:
            production = int(row.get("prod_callers_other_tu", 0)) + int(row.get("prod_callers_same_tu", 0))
        except ValueError:
            production = -1
            failures.append(f"non_integer_callers={row['id']}")
        if production == 0 and row.get("action") != "DELETE":
            rationale = row.get("rationale", "").lower()
            mechanism = ("template instantiation", "macro", "adl", "entry point", "entry-point", "public header contract", "aggregate initialization", "serialization", "registration")
            if not any(token in rationale for token in mechanism):
                failures.append(f"zero_non_delete_without_mechanism={row['id']}")
        if row.get("confidence") == "high" and row.get("definition_seen_in_ast") != "True":
            failures.append(f"high_without_ast_definition={row['id']}")

    def exact(suffix: str) -> list[dict[str, str]]:
        return [row for row in rows if row["id"].endswith("::" + suffix)]

    mission = exact("MissionController")
    if len(mission) != 1 or mission[0]["prod_callers_other_tu"] != "0" or mission[0]["action"] != "DELETE":
        failures.append("oracle_MissionController")
    execution = exact("ExecutionAuthority")
    if len(execution) != 1 or execution[0]["target_module"] != "nav_execution":
        failures.append("oracle_ExecutionAuthority")
    candidate = exact("CandidateBundle")
    if len(candidate) != 1 or candidate[0]["target_module"] != "nav_plan_contract":
        failures.append("oracle_CandidateBundle")
    runtime_node = exact("NavigationRuntimeNode")
    if len(runtime_node) != 1 or runtime_node[0]["target_module"] != "nav_core_node" or runtime_node[0]["action"] != "SPLIT":
        failures.append("oracle_NavigationRuntimeNode")
    for basename in ("trajectory_world_validator.hpp", "corridor_plane_validation.hpp"):
        if not any(Path(row["file"]).name == basename and row["target_module"] == "nav_certifier" for row in rows):
            failures.append("oracle_file_target=" + basename)

    try:
        with args.graph.open(newline="", encoding="utf-8") as stream:
            graph_rows = list(csv.DictReader(stream))
        if len(graph_rows) != len(rows) or {row["id"] for row in graph_rows} != {row["id"] for row in rows}:
            failures.append("reference_graph_inventory_mismatch")
    except OSError as exc:
        failures.append(f"reference_graph_unreadable={exc}")

    try:
        graph_text = args.module_graph.read_text(encoding="utf-8")
        if "cycle_status=none" not in graph_text:
            failures.append("module_dependency_cycle_status")
    except OSError as exc:
        failures.append(f"module_graph_unreadable={exc}")

    print(f"semantic_rows={len(rows)}")
    print(f"reference_graph_rows={len(graph_rows) if 'graph_rows' in locals() else 0}")
    print(f"semantic_failures={len(failures)}")
    for failure in failures:
        print(failure)
    return 1 if missing or abs(delta) > 0.10 or failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
