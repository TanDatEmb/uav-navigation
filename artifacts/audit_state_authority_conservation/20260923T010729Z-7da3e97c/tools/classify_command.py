#!/usr/bin/env python3
"""Schema-level field register; consumer evidence still needs manual review."""
import csv
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parents[1]
PATH = ROOT / "src/contracts/navigation_contracts/msg/NavigationCommand.msg"
SAFETY = {"localization_epoch", "world_generation", "world_revision", "world_observation_stamp", "bundle_generation", "certified_main_continuation", "continuation_boundary_stamp_ns", "sample_id", "state_source_stamp", "valid_until", "role", "status", "jerk"}
PROVENANCE = {"goal_epoch", "mission_id", "waypoint_index", "request_id", "trajectory_time_s", "execution_authorization", "execution_authorization_steady_ns"}
CONTROL = {"header", "position", "velocity", "acceleration", "yaw", "yaw_rate"}
with (OUT / "CONTROL_PROTOCOL_FIELD_AUDIT.csv").open("w", newline="") as f:
 writer = csv.writer(f, lineterminator="\n")
 writer.writerow(["field", "type", "source", "preliminary_class", "consumer_proof", "split_action"])
 for i, line in enumerate(PATH.read_text().splitlines(), 1):
  m = re.match(r"^(\S+)\s+([A-Za-z][A-Za-z0-9_]*)(?:=.*)?$", line)
  if not m or "=" in line: continue
  typ, name = m.groups()
  category = ("CONTROL_REQUIRED" if name in CONTROL else
              "SAFETY_ADMISSION_REQUIRED" if name in SAFETY else
              "PROVENANCE_REQUIRED" if name in PROVENANCE else
              "DIAGNOSTIC_ONLY" if i >= 49 or name == "reason_code" else "LEGACY/REDUNDANT_CANDIDATE")
  proof = "schema and selected adapter paths; full consumer map pending" if i <= 47 else "schema comment; all consumers unverified"
  action = "KEEP pending consumer audit" if i <= 47 else "TRACE candidate; do not remove before consumer audit"
  writer.writerow([name, typ, f"src/contracts/navigation_contracts/msg/NavigationCommand.msg:{i}", category, proof, action])
