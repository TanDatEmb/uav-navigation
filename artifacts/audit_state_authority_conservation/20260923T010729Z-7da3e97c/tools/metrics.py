#!/usr/bin/env python3
"""Reproducible scoped proxies; not a whole-product architecture metric."""
import csv
import json
import re
import subprocess
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parents[1]
TARGET_SHA = "7da3e97cb399c2e39d62cfe60213a45e8a92300e"
with (OUT / "STATE_CONSERVATION_MATRIX.csv").open() as f: rows = list(csv.DictReader(f))
with (OUT / "DECLARATION_TRIAGE.csv").open() as f: declarations = list(csv.DictReader(f))
with (OUT / "CONTROL_PROTOCOL_FIELD_AUDIT.csv").open() as f: command = list(csv.DictReader(f))
sources = [
 "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp",
 "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp",
 "src/execution/navigation_execution/include/navigation_execution/committed_bundle_store.hpp",
]
for path in sources:
 current_blob = subprocess.check_output(["git", "hash-object", str(ROOT / path)], text=True, cwd=ROOT).strip()
 target_blob = subprocess.check_output(["git", "rev-parse", f"{TARGET_SHA}:{path}"], text=True, cwd=ROOT).strip()
 if current_blob != target_blob:
  raise RuntimeError(f"input differs from pinned TARGET: {path}")
identity_call_pattern = re.compile(r"\b(?:sameGoalIdentity|sameWorldSnapshotIdentity|executingCommandIdentityMatchesLocked|bundleIdentityMatchesEpisode|activeBundleIdentityMatches|sameGoalOwner|sameIdentity)\s*\(")
identity_helper_occurrences = {p: len(identity_call_pattern.findall((ROOT / p).read_text())) for p in sources}
metrics = {
 "scope": "16 selected C++ owner types, 3 identity-source files, one command message; not whole product",
 "behavioral_candidate_fields": len(rows),
 "dispositions": dict(Counter(r["Target action"] for r in rows)),
 "selected_writer_reader_rows": sum(r["Writer(s)"] != "UNRESOLVED" for r in rows),
 "diagnostic_or_test_candidate_declarations": sum("diagnostic" in r["Category"] for r in declarations),
 "bool_like_behavioral_candidates": sum("bool" in r["Type"] for r in rows),
 "optional_like_behavioral_candidates": sum("optional" in r["Type"] for r in rows),
 "mutex_declarations_in_scanned_types": sum("mutex" in r["Type"] for r in declarations),
 "identity_helper_occurrences_selected_sources": identity_helper_occurrences,
 "identity_helper_occurrence_total": sum(identity_helper_occurrences.values()),
 "navigation_command_nonconstant_fields": len(command),
 "navigation_command_preliminary_classes": dict(Counter(r["preliminary_class"] for r in command)),
 "verified_nested_runtime_lock_depth_lower_bound": 4,
 "whole_product_manual_comparison_sites": "UNRESOLVED",
 "whole_product_cross_owner_atomic_transitions": "UNRESOLVED",
 "whole_product_authority_owners": "UNRESOLVED",
}
(OUT / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
print(json.dumps(metrics, indent=2))
