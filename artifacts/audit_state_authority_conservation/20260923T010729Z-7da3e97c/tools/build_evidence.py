#!/usr/bin/env python3
"""Pin selected source ranges to TARGET blobs; no runtime claims."""
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parents[1] / "EVIDENCE_INDEX.md"
TARGET_SHA = "7da3e97cb399c2e39d62cfe60213a45e8a92300e"
REFS = [
 ("Safety contract", "docs/safety/runtime_safety_current.md", "40-69,84-130"),
 ("Safety index targeted", "docs/safety/runtime_safety_index.md", "37,43,55,77,766"),
 ("Safety archive targeted gate rows", "docs/safety/archive/runtime_safety_legacy_full.md", "3378,3384,3396"),
 ("Safety archive PX4 ownership/STOP lineage", "docs/safety/archive/runtime_safety_legacy_full.md", "321-349,22114-22142"),
 ("Parameter contract", "docs/architecture/parameter_contract.md", "1-80"),
 ("Waypoint design", "docs/architecture/continuous_waypoint_trajectory_plan.md", "193-225,294-325"),
 ("Mission fields", "src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/mission_controller.hpp", "110-130"),
 ("Mission crossing", "src/px4/px4_navigation_external_mode/src/mission_controller.cpp", "340-371,465-590"),
 ("Mission lifecycle, checkpoint, readiness and temporal gates", "src/px4/px4_navigation_external_mode/src/mission_controller.cpp", "28-89,101-338,374-445,520-680"),
 ("Pre-stop nominal recovery", "src/px4/px4_navigation_external_mode/src/mission_controller.cpp", "144-165"),
 ("Route cursor/tie", "src/contracts/navigation_mission/src/route_progress.cpp", "256-345"),
 ("Episode", "src/runtime/navigation_runtime/include/navigation_runtime/execution_episode.hpp", "41-260"),
 ("Store", "src/execution/navigation_execution/include/navigation_execution/committed_bundle_store.hpp", "30-35,821-829"),
 ("Runtime fields/locks", "src/runtime/navigation_runtime/include/navigation_runtime/navigation_runtime_node.hpp", "429-560"),
 ("World suspension", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", "2983-3004,3670-3690,7704-7719"),
 ("Command publication", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", "7935-7950,8595-8640"),
 ("Sampled BACKUP", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", "8200-8229"),
 ("PX4 fields", "src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/navigation_mode.hpp", "135-261,281-290"),
 ("PX4 command/mission", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", "1328-1410"),
 ("PX4 Hold", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", "2846-2937"),
 ("Candidate callbacks", "src/planning/navigation_planning/include/navigation_planning/candidate_bundle.hpp", "119-162"),
 ("Worker", "src/runtime/navigation_runtime/include/navigation_runtime/planning_worker.hpp", "268-350"),
 ("World owner", "src/mapping/navigation_mapping/include/navigation_mapping/world_snapshot_store.hpp", "131-178"),
 ("Execution state ingress", "src/execution/navigation_execution/include/navigation_execution/execution_state_store.hpp", "12-48"),
 ("Derivative history", "src/runtime/navigation_runtime/include/navigation_runtime/kinematic_derivative_estimator.hpp", "20-91"),
 ("Mapping worker lifecycle", "src/mapping/navigation_mapping/include/navigation_mapping/mapping_worker.hpp", "23-293"),
 ("Heading worker lifecycle", "src/runtime/navigation_runtime/include/navigation_runtime/heading_rebind_worker.hpp", "26-121"),
 ("Quality receipt", "src/runtime/navigation_runtime/include/navigation_runtime/baseline_refinement.hpp", "30-103"),
 ("Trace-only store", "src/runtime/navigation_runtime/include/navigation_runtime/execution_trace_snapshot.hpp", "135-177"),
 ("Command schema", "src/contracts/navigation_contracts/msg/NavigationCommand.msg", "1-116"),
 ("Planning key", "src/planning/navigation_planning/include/navigation_planning/planning_request.hpp", "1-130"),
 ("Speed governor", "src/planning/navigation_planning_backend/include/planner_core/evidence_speed_governor.hpp", "1-240"),
 ("Tracking experiment loader", "src/contracts/navigation_contracts/include/navigation_contracts/tracking_experiment.hpp", "58-88,147"),
 ("Runtime rest-recovery timer", "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp", "5400-5425"),
 ("Adapter recovery timer", "src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp", "1220-1250,1340-1355"),
]

def cmd(*args):
 return subprocess.check_output(args, cwd=ROOT, text=True).strip()

def main():
 lines = ["# Evidence index", "", "All source facts below are from TARGET `7da3e97cb399c2e39d62cfe60213a45e8a92300e`. Blob IDs allow verification after branch movement. Line spans are source locations, not executed behavior.", "", "| Fact | TARGET path and lines | Git blob |", "|---|---|---|"]
 for name, path, span in REFS:
  line_count = len((ROOT / path).read_text().splitlines())
  assert max(int(n) for n in re.findall(r"\d+", span)) <= line_count, (path, span, line_count)
  blob = cmd("git", "hash-object", str(ROOT / path))
  if blob != cmd("git", "rev-parse", f"{TARGET_SHA}:{path}"):
   raise RuntimeError(f"input differs from pinned TARGET: {path}")
  lines.append(f"| {name} | `{path}:{span}` | `{blob}` |")
 gitlink = cmd("git", "ls-tree", TARGET_SHA, "src/external/px4_ros2_interface_lib")
 submodule_sha = "4a3370f084ac6f1ef001a4afa2b007845ffd0837"
 if not gitlink.startswith(f"160000 commit {submodule_sha}\t"):
  raise RuntimeError("PX4 interface library gitlink differs from pinned TARGET")
 common_git_dir = Path(cmd("git", "rev-parse", "--git-common-dir"))
 submodule_git_dir = common_git_dir / "modules/src/external/px4_ros2_interface_lib"
 lib_blob = subprocess.check_output([
  "git", f"--git-dir={submodule_git_dir}", "rev-parse",
  f"{submodule_sha}:px4_ros2_cpp/src/components/mode_executor.cpp",
 ], text=True).strip()
 lines.append(f"| PX4 library `scheduleMode` and completion callback | `src/external/px4_ros2_interface_lib/px4_ros2_cpp/src/components/mode_executor.cpp:225-260,484-519` at gitlink `{submodule_sha}` | `{lib_blob}` |")
 lines += ["", "## Prior-artifact provenance and TARGET revalidation", "",
  "- AS-IS artifact `f2bd3f46f9936d622377ea4761f733f988273b66`: local dirty snapshot at recorded HEAD `9534d8dc`; used as hypothesis only. Its own report was `PARTIAL_AS_IS`.",
  "- H0-H7 artifact `f2ed429bef80b2c7a2964b3b32d3c00e8089e55f`: previous local counterexamples and source scope, not TARGET product-path proof.",
  "- State-contract artifact `6ff508217778aa42e60e208d4dd0a49785a4fae6`: 56-row selected contract inventory and delta; not whole-product coverage. Its A→TARGET delta already identified stale H5 grid and desired/active identity change; rechecked here on TARGET paths.", "",
  "| Prior claim | TARGET assessment | Evidence limit |", "|---|---|---|",
  "| H0 profile | CONFIRMED_WITH_SCOPE | TARGET loader still sets suppression from `use_sim_time` and tracking gate; no live TARGET parameter dump. |",
  "| H1 PASS_THROUGH | CONDITIONAL | TARGET mission code still overwrites previous sample and requires same-update witness; prior local helper counterexample is not full producer-path reachability. |",
  "| H2 world suspension | CONFIRMED_WITH_SCOPE | TARGET source suspends publication and exact-resume path; actual PX4 lease expiry not observed. |",
  "| H3 publish lock | CONFIRMED_WITH_SCOPE | TARGET still nests localization/input/command and Store publication; no workload bottleneck distribution. |",
  "| H4 recovery timers | SPECIFICATION_GAP | TARGET runtime steady failure timer and adapter ROS deadline still differ; no cross-process equivalence trace. |",
  "| H5 fixed 16-speed grid | REFUTED on TARGET | TARGET governor changed; no performance/completeness proof. |",
  "| H6 Hold order | CONDITIONAL | TARGET adapter separates `scheduleMode` completion from VehicleStatus; pinned library shows the callback can be a ModeCompleted result, not merely command ACK. Ordering was not run. |",
  "| H7 bottleneck ranking | UNRESOLVED | No matched target workload trace. |", "",
  "`FACT_FROM_EXISTING_TEST` means test source or earlier run only. This audit executed only its independent abstract model and structural scripts; no TARGET product binary, ROS pair, SITL or hardware run. Therefore all target runtime behavior and performance claims are `RUNTIME_UNVERIFIED`.", ""]
 OUT.write_text("\n".join(lines))
 print(f"wrote {OUT}")

if __name__ == "__main__": main()
