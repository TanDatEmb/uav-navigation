import importlib.util
import hashlib
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock
from contextlib import redirect_stderr

import yaml

RUNTIME = Path(__file__).resolve().parents[1]
ROOT = RUNTIME.parents[1]
sys.path.insert(0, str(RUNTIME))

import monitor
from monitor import StreamStats
import build_provenance
import dataset_shadow_planning
import gazebo_native_observer
import report
import runner
import process_group


def _valid_captured_provenance() -> dict[str, object]:
    artifact_path = RUNTIME / "report.py"
    artifact_bytes = artifact_path.read_bytes()
    manifest = {
        "schema_version": 1,
        "authoritative": True,
        "build_mode": "release",
        "source": {"sha256": "b" * 64, "git_head": "run-head"},
        "artifacts": [{
            "path": str(artifact_path),
            "resolved_path": str(artifact_path.resolve()),
            "size_bytes": len(artifact_bytes),
            "sha256": hashlib.sha256(artifact_bytes).hexdigest(),
        }],
    }
    manifest_sha = hashlib.sha256(
        (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
    ).hexdigest()
    return {"status": "VALID", "manifest_sha256": manifest_sha, "manifest": manifest}


def _mapping_outcomes(updated: int, **overrides: int) -> dict[str, int]:
    values = {
        "mapping_outcome_updated_count": updated,
        "mapping_outcome_accumulated_count": 0,
        "mapping_outcome_slide_only_count": 0,
        "mapping_outcome_empty_cloud_count": 0,
        "mapping_outcome_callback_owned_count": 0,
        "mapping_outcome_below_ground_count": 0,
        "mapping_outcome_above_ceiling_count": 0,
        "observation_stamp_ns": 1_000_000_000 + updated,
        "last_update_attempt_stamp_ns": 1_000_000_000 + updated,
    }
    values.update(overrides)
    return values


class RuntimeContractTest(unittest.TestCase):
    def test_canonical_python_rejects_virtualenv_and_non_system_interpreter(self) -> None:
        self.assertIsNone(
            runner.canonical_python_error("/usr/bin/python3", environment={})
        )
        self.assertIn(
            "VIRTUAL_ENV",
            runner.canonical_python_error(
                "/usr/bin/python3", environment={"VIRTUAL_ENV": "/tmp/venv"}
            ) or "",
        )
        self.assertIn(
            "/usr/bin/python3",
            runner.canonical_python_error("/tmp/venv/bin/python", environment={}) or "",
        )

    def test_manifest_rejects_an_unrecorded_runtime_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            install = root / "install"
            artifact = install / "product/bin/node"
            extra = install / "product/lib/libnew.so"
            artifact.parent.mkdir(parents=True)
            extra.parent.mkdir(parents=True)
            artifact.write_bytes(b"node")
            extra.write_bytes(b"new")
            (root / ".gitignore").write_text("install/\n", encoding="utf-8")
            subprocess = __import__("subprocess")
            subprocess.run(["git", "init", "-q"], cwd=root, check=True)
            subprocess.run(["git", "config", "user.email", "test@example.com"], cwd=root, check=True)
            subprocess.run(["git", "config", "user.name", "Test"], cwd=root, check=True)
            (root / "source").write_text("x\n", encoding="utf-8")
            subprocess.run(["git", "add", "."], cwd=root, check=True)
            subprocess.run(["git", "commit", "-qm", "base"], cwd=root, check=True)
            manifest = {
                "schema_version": build_provenance.SCHEMA_VERSION,
                "authoritative": True,
                "build_mode": "release",
                "source": build_provenance.source_fingerprint(root),
                "artifacts": [build_provenance._artifact_record(artifact, root)],
            }
            build_provenance.write_manifest_atomic(install, manifest)
            with mock.patch.object(
                build_provenance, "runtime_artifact_paths", return_value=[artifact, extra]
            ):
                with self.assertRaisesRegex(RuntimeError, "artifact set mismatch"):
                    build_provenance.validate_manifest(root, install)

    def test_build_provenance_rejects_stale_source_and_modified_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            install = root / "install"
            artifact = install / "product/bin/node"
            source = root / "product.cpp"
            artifact.parent.mkdir(parents=True)
            source.write_text("baseline\n", encoding="utf-8")
            artifact.write_bytes(b"binary-baseline")
            (root / ".gitignore").write_text("install/\n", encoding="utf-8")
            subprocess = __import__("subprocess")
            subprocess.run(["git", "init", "-q"], cwd=root, check=True)
            subprocess.run(["git", "config", "user.email", "test@example.com"], cwd=root, check=True)
            subprocess.run(["git", "config", "user.name", "Test"], cwd=root, check=True)
            subprocess.run(["git", "add", "product.cpp", ".gitignore"], cwd=root, check=True)
            subprocess.run(["git", "commit", "-qm", "base"], cwd=root, check=True)
            fingerprint = build_provenance.source_fingerprint(root)
            record = build_provenance._artifact_record(artifact, root)
            manifest = {
                "schema_version": build_provenance.SCHEMA_VERSION,
                "authoritative": True,
                "build_mode": "release",
                "source": fingerprint,
                "artifacts": [record],
            }
            build_provenance.write_manifest_atomic(install, manifest)
            with mock.patch.object(build_provenance, "runtime_artifact_paths", return_value=[artifact]):
                self.assertEqual(build_provenance.validate_manifest(root, install)["status"], "VALID")
            source.write_text("current\n", encoding="utf-8")
            with mock.patch.object(build_provenance, "runtime_artifact_paths", return_value=[artifact]):
                with self.assertRaisesRegex(RuntimeError, "source fingerprint"):
                    build_provenance.validate_manifest(root, install)
            source.write_text("baseline\n", encoding="utf-8")
            artifact.write_bytes(b"binary-current")
            with mock.patch.object(build_provenance, "runtime_artifact_paths", return_value=[artifact]):
                with self.assertRaisesRegex(RuntimeError, "artifact identity mismatch"):
                    build_provenance.validate_manifest(root, install)

    def test_build_provenance_rejects_missing_partial_and_symlink_swap(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            install = root / "install"
            install.mkdir()
            subprocess = __import__("subprocess")
            subprocess.run(["git", "init", "-q"], cwd=root, check=True)
            subprocess.run(["git", "config", "user.email", "test@example.com"], cwd=root, check=True)
            subprocess.run(["git", "config", "user.name", "Test"], cwd=root, check=True)
            tracked = root / "source"
            tracked.write_text("x", encoding="utf-8")
            (root / ".gitignore").write_text("install/\na\nb\n", encoding="utf-8")
            subprocess.run(["git", "add", "source", ".gitignore"], cwd=root, check=True)
            subprocess.run(["git", "commit", "-qm", "base"], cwd=root, check=True)
            with self.assertRaisesRegex(RuntimeError, "manifest unavailable"):
                build_provenance.validate_manifest(root, install)
            target_a = root / "a"
            target_b = root / "b"
            target_a.write_text("same", encoding="utf-8")
            target_b.write_text("same", encoding="utf-8")
            link = install / "node"
            link.symlink_to(target_a)
            manifest = {
                "schema_version": 1,
                "authoritative": False,
                "build_mode": "release",
                "source": build_provenance.source_fingerprint(root),
                "artifacts": [build_provenance._artifact_record(link, root)],
            }
            build_provenance.write_manifest_atomic(install, manifest)
            with self.assertRaisesRegex(RuntimeError, "authoritative full Release"):
                build_provenance.validate_manifest(root, install)
            manifest["authoritative"] = True
            build_provenance.write_manifest_atomic(install, manifest)
            link.unlink()
            link.symlink_to(target_b)
            with mock.patch.object(build_provenance, "runtime_artifact_paths", return_value=[link]):
                with self.assertRaisesRegex(RuntimeError, "resolved_path"):
                    build_provenance.validate_manifest(root, install)

    def test_source_fingerprint_covers_dirty_submodule_content(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            child = base / "child"
            root = base / "root"
            child.mkdir()
            root.mkdir()
            subprocess = __import__("subprocess")
            for repository in (child, root):
                subprocess.run(["git", "init", "-q"], cwd=repository, check=True)
                subprocess.run(["git", "config", "user.email", "test@example.com"], cwd=repository, check=True)
                subprocess.run(["git", "config", "user.name", "Test"], cwd=repository, check=True)
            (child / "value.txt").write_text("one\n", encoding="utf-8")
            subprocess.run(["git", "add", "value.txt"], cwd=child, check=True)
            subprocess.run(["git", "commit", "-qm", "child"], cwd=child, check=True)
            subprocess.run(
                ["git", "-c", "protocol.file.allow=always", "submodule", "add", "-q", str(child), "vendor"],
                cwd=root, check=True,
            )
            subprocess.run(["git", "commit", "-qam", "root"], cwd=root, check=True)
            vendor = root / "vendor" / "value.txt"
            vendor.write_text("dirty-one\n", encoding="utf-8")
            first = build_provenance.source_fingerprint(root)
            vendor.write_text("dirty-two\n", encoding="utf-8")
            second = build_provenance.source_fingerprint(root)
            self.assertTrue(first["git_dirty"])
            self.assertTrue(second["git_dirty"])
            self.assertNotEqual(first["sha256"], second["sha256"])
            self.assertNotEqual(
                first["submodules"][0]["sha256"], second["submodules"][0]["sha256"]
            )

    def test_manifest_covers_both_px4_odometry_bridge_executables(self) -> None:
        self.assertIn(
            "px4_odometry_bridge/lib/px4_odometry_bridge/px4_odometry_bridge_node",
            build_provenance.CRITICAL_ARTIFACTS,
        )

    def test_runtime_artifact_discovery_includes_workspace_shared_library(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            install = root / "install"
            library = install / "product" / "lib" / "libdependency.so"
            library.parent.mkdir(parents=True)
            library.write_bytes(b"dependency")
            with (
                mock.patch.object(build_provenance, "PRODUCT_RUNTIME_PREFIXES", ("product",)),
                mock.patch.object(build_provenance, "CRITICAL_ARTIFACTS", ()),
                mock.patch.object(build_provenance, "RUNTIME_SCRIPTS", ()),
            ):
                paths = build_provenance.runtime_artifact_paths(root, install)
            self.assertEqual(paths, [library])
        self.assertIn(
            "px4_odometry_bridge/lib/px4_odometry_bridge/px4_odometry_bridge_external_node",
            build_provenance.CRITICAL_ARTIFACTS,
        )

    def test_runtime_artifacts_use_git_common_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            checkout = Path(temporary) / "worktree"
            common = Path(temporary) / "main" / ".git"
            checkout.mkdir()
            common.mkdir(parents=True)
            completed = mock.Mock(returncode=0, stdout=str(common) + "\n")
            with mock.patch.dict(os.environ, {"UAV_NAV_ARTIFACT_ROOT": ""}):
                with mock.patch.object(runner.subprocess, "run", return_value=completed):
                    self.assertEqual(
                        runner._shared_artifact_root(checkout),
                        common.parent / ".artifacts/runtime",
                    )

    def test_runtime_artifact_override_is_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            override = Path(temporary) / "runtime"
            with mock.patch.dict(
                os.environ,
                {"UAV_NAV_ARTIFACT_ROOT": str(override)},
            ):
                self.assertEqual(runner._shared_artifact_root(), override.resolve())

    def test_clean_path_list_excludes_build_and_install_trees(self) -> None:
        for name in ("build", "install"):
            self.assertNotIn(ROOT / name, runner.GENERATED_CLEAN_PATHS)
        for name in ("build-debug", "build-gprof", "install-debug", "log-debug"):
            self.assertIn(ROOT / name, runner.GENERATED_CLEAN_PATHS)

    def test_runtime_lock_rejects_a_second_owner(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            lock_path = Path(temporary) / ".runtime.lock"
            first = runner.RuntimeLock(lock_path)
            with first:
                second = runner.RuntimeLock(lock_path)
                with self.assertRaises(runner.RuntimeBusyError) as context:
                    second.__enter__()
                self.assertIn("already owns", str(context.exception))
                self.assertIsNone(second._file)

    def test_runtime_lock_releases_after_context_exit(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            lock_path = Path(temporary) / ".runtime.lock"
            with runner.RuntimeLock(lock_path):
                pass
            with runner.RuntimeLock(lock_path):
                pass

    def test_build_runtime_lock_allows_isolated_runtime_readers(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = runner.BuildRuntimeLock(root, exclusive=False)
            second = runner.BuildRuntimeLock(root, exclusive=False)
            with first:
                with second:
                    self.assertTrue(True)

    def test_build_runtime_lock_rejects_build_while_runtime_reader_is_live(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with runner.BuildRuntimeLock(root, exclusive=False):
                with self.assertRaises(runner.BuildRuntimeBusyError):
                    with runner.BuildRuntimeLock(root, exclusive=True):
                        pass

    def test_sim_lock_ignores_an_isolated_dataset_session(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            root = Path(temporary)
            lock_path = root / ".runtime-sim.lock"

            def active_sessions(_root: Path, workflows: set[str] | None) -> list[str]:
                self.assertEqual(workflows, {"sim", "external-mode"})
                return []

            with mock.patch.object(runner, "_active_runtime_sessions", active_sessions):
                with runner.RuntimeLock(
                    lock_path,
                    artifact_root=root,
                    active_workflows={"sim", "external-mode"},
                ):
                    self.assertTrue(lock_path.is_file())

    def test_clean_preserves_build_runtime_lock_inode(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            parent = Path(temporary)
            artifact_root = parent / "runtime"
            artifact_root.mkdir()
            runtime_lock = artifact_root / ".runtime-sim.lock"
            build_lock = artifact_root / ".build-runtime.lock"
            runtime_lock.touch()
            build_lock.touch()
            with (
                mock.patch.object(runner, "ARTIFACT_ROOT", artifact_root),
                mock.patch.object(runner, "RUNTIME_LOCK_PATH", runtime_lock),
                mock.patch.object(runner, "BUILD_RUNTIME_LOCK_PATH", build_lock),
                mock.patch.object(runner, "GENERATED_CLEAN_PATHS", (parent,)),
            ):
                runner._clean_unlocked(clean_workspace_caches=False)
            self.assertTrue(runtime_lock.exists())
            self.assertTrue(build_lock.exists())

    def test_readiness_fails_immediately_for_an_exited_zombie_process(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            runtime_root = Path(temporary) / "runtime"
            session = runner.Session.create(runtime_root, "sim")
            process = session.start(
                "bridge_lidar", ["bash", "-lc", "exit 0"], cwd=ROOT
            )
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                try:
                    state = Path(f"/proc/{process.pid}/stat").read_text(
                        encoding="utf-8"
                    ).split(")", 1)[1].strip().split()[0]
                except OSError:
                    state = "gone"
                if state in {"Z", "gone"}:
                    break
                time.sleep(0.01)
            try:
                with self.assertRaisesRegex(
                    RuntimeError,
                    "runtime process bridge_lidar exited before readiness",
                ):
                    runner._wait_until(
                        session, lambda _: False, 1.0, "test readiness"
                    )
            finally:
                process.wait(timeout=2.0)

    def test_clean_preserves_incremental_build_and_install_trees(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            root = Path(temporary)
            build = root / "build"
            install = root / "install"
            stale_build = root / "build-debug"
            artifacts = root / ".artifacts"
            logs = root / "log"
            for directory in (build, install, stale_build, artifacts, logs):
                directory.mkdir()
                (directory / "sentinel").write_text("keep", encoding="utf-8")

            original_paths = runner.GENERATED_CLEAN_PATHS
            runner.GENERATED_CLEAN_PATHS = (artifacts, logs, stale_build)
            try:
                self.assertEqual(
                    runner._clean_unlocked(clean_workspace_caches=False), 0
                )
            finally:
                runner.GENERATED_CLEAN_PATHS = original_paths

            self.assertTrue((build / "sentinel").is_file())
            self.assertTrue((install / "sentinel").is_file())
            self.assertFalse(stale_build.exists())
            self.assertFalse(artifacts.exists())
            self.assertFalse(logs.exists())

    def test_clean_preserves_runtime_lock_while_removing_sessions(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            artifacts = Path(temporary) / ".artifacts"
            runtime_root = artifacts / "runtime"
            session = runtime_root / "sim-old"
            session.mkdir(parents=True)
            (session / "processes.json").write_text("{}", encoding="utf-8")
            lock = runtime_root / ".runtime-sim.lock"
            lock.write_text("{}\n", encoding="utf-8")
            (runtime_root / "latest").symlink_to(session.name)

            original_artifact_root = runner.ARTIFACT_ROOT
            original_lock_path = runner.RUNTIME_LOCK_PATH
            original_dataset_lock_path = runner.DATASET_RUNTIME_LOCK_PATH
            original_paths = runner.GENERATED_CLEAN_PATHS
            runner.ARTIFACT_ROOT = runtime_root
            runner.RUNTIME_LOCK_PATH = lock
            runner.DATASET_RUNTIME_LOCK_PATH = runtime_root / ".runtime-dataset.lock"
            runner.DATASET_RUNTIME_LOCK_PATH.write_text("{}\n", encoding="utf-8")
            runner.GENERATED_CLEAN_PATHS = (artifacts,)
            try:
                self.assertEqual(
                    runner._clean_unlocked(clean_workspace_caches=False), 0
                )
            finally:
                runner.ARTIFACT_ROOT = original_artifact_root
                runner.RUNTIME_LOCK_PATH = original_lock_path
                runner.DATASET_RUNTIME_LOCK_PATH = original_dataset_lock_path
                runner.GENERATED_CLEAN_PATHS = original_paths

            self.assertTrue(lock.is_file())
            self.assertTrue((runtime_root / ".runtime-dataset.lock").is_file())
            self.assertFalse(session.exists())
            self.assertFalse((runtime_root / "latest").exists())

    def test_mapping_config_uses_canonical_product_contract(self) -> None:
        navigation = runner.load_config("mapping.yaml")["navigation_runtime_node"]["ros__parameters"]["navigation_runtime"]
        self.assertEqual(navigation["registered_scan_topic"], "/lio/mapping_observation")
        self.assertEqual(navigation["propagated_odometry_topic"], "/lio/odometry_propagated")
        self.assertNotIn("cloud_topic", navigation)
        self.assertNotIn("corrected_odometry_topic", navigation)
        self.assertNotIn("odometry_topic", navigation)
        self.assertEqual(navigation["planner_watchdog_timeout_s"], 1.0)
        self.assertEqual(navigation["mapping_snapshot_publication_period_s"], 0.2)
        self.assertNotIn("planner_solve_timeout_s", navigation)
        self.assertEqual(navigation["command_topic"], "/navigation/navigation_command")
        runtime_source = (
            ROOT / "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("onRegisteredScan", runtime_source)
        self.assertNotIn("navigation_runtime.cloud_topic", runtime_source)
        self.assertNotIn("navigation_runtime.corrected_odometry_topic", runtime_source)
        rviz = runner.RVIZ_CONFIG.read_text(encoding="utf-8")
        self.assertIn("/lio/registered_points", rviz)

    def test_package_and_workspace_runtime_profiles_have_one_navigation_contract(self) -> None:
        workspace_profile = yaml.safe_load(
            (ROOT / "config/runtime/mapping.yaml").read_text(encoding="utf-8")
        )["navigation_runtime_node"]["ros__parameters"]["navigation_runtime"]
        cmake = (ROOT / "src/runtime/navigation_runtime/CMakeLists.txt").read_text(
            encoding="utf-8"
        )
        self.assertIn("config/runtime/mapping.yaml", cmake)
        self.assertIn("configure_file", cmake)
        self.assertIn("config/planner.yaml", cmake)
        self.assertNotIn(
            "src/runtime/navigation_runtime/config/navigation_runtime.yaml",
            cmake,
        )
        self.assertTrue(workspace_profile)
        installed_planner = ROOT / "install/navigation_runtime/share/navigation_runtime/config/planner.yaml"
        self.assertTrue(installed_planner.is_file())

    def test_corrected_mapping_precedes_propagated_planner_gates(self) -> None:
        source = (
            ROOT / "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp"
        ).read_text(encoding="utf-8")
        mapping_actor = (
            ROOT / "src/mapping/navigation_mapping/src/mapping_actor.cpp"
        ).read_text(encoding="utf-8")
        cycle = source[
            source.index("void NavigationRuntimeNode::runCycle()"):
                       source.index("void NavigationRuntimeNode::publishCommand()")]
        constructor = source[:source.index("void NavigationRuntimeNode::runCycle()")]
        self.assertIn("std::make_shared<navigation_mapping::MappingActor>", constructor)
        self.assertIn("const auto result = mapping_actor->process(observation);", constructor)
        self.assertIn("observation.corrected_odometry.pose.pose", mapping_actor)
        self.assertIn("const auto backend_outcome = map_->updateMap(", mapping_actor)
        self.assertIn("mapping_worker_->start()", constructor)
        self.assertIn("propagated_state_callback_group_", constructor)
        self.assertIn("propagated_state_options.callback_group", constructor)
        self.assertNotIn("updateMap(", cycle)
        self.assertNotIn("map_->", cycle)
        self.assertIn("if (!propagated_state) return;", cycle)
        self.assertIn("const bool completed_trajectory", cycle)

    def test_mapping_profile_is_independent_of_visualization_mode(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            target = runner._mapping_params(
                session, ROOT / "config/runtime/mapping.yaml"
            )
            parameters = yaml.safe_load(target.read_text(encoding="utf-8"))["navigation_runtime_node"]["ros__parameters"]["navigation_runtime"]
            self.assertTrue(Path(parameters["config_path"]).is_file())

    def test_simulation_mapping_profile_preserves_collision_parameters(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            target = runner._mapping_params(
                session, ROOT / "config/runtime/mapping.yaml"
            )
            parameters = yaml.safe_load(target.read_text(encoding="utf-8"))["navigation_runtime_node"]["ros__parameters"]["navigation_runtime"]
            self.assertTrue(Path(parameters["config_path"]).is_file())


    def test_mission_planning_policy_is_applied_to_runtime_parameters(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            mission = ROOT / "config/runtime/missions/open.yaml"
            target = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                mission_file=mission,
            )
            parameters = yaml.safe_load(target.read_text(encoding="utf-8"))["navigation_runtime_node"]["ros__parameters"]["navigation_runtime"]
            planner = yaml.safe_load(Path(parameters["config_path"]).read_text(encoding="utf-8"))
            self.assertEqual(parameters["planner_rate_hz"], 5.0)
            self.assertEqual(parameters["mission_file"], str(mission.resolve()))
            self.assertEqual(planner["traj_opt"]["boundary"]["max_vel"], 1.0)
            self.assertEqual(planner["traj_opt"]["boundary"]["max_acc"], 2.0)
            self.assertEqual(planner["traj_opt"]["boundary"]["max_jerk"], 6.0)
            self.assertEqual(
                planner["traj_opt"]["exp_traj"]["objective"]["jerk_penalty_weight"], 0.0
            )
            self.assertGreater(
                planner["traj_opt"]["backup_traj"]["objective"]["jerk_penalty_weight"], 0.0
            )
            self.assertNotIn("frontend_in_known_free", planner["planner"])
            self.assertTrue(planner["rog_map"]["raycasting"]["enable"])
            self.assertNotIn("unk_inflation_en", planner["rog_map"])

            self.assertFalse(planner["rog_map"]["virtual_ground_ceiling_en"])
            ray_range_min = planner["rog_map"]["raycasting"]["ray_range"][0]
            self.assertEqual(ray_range_min, 0.8)
            planner_safety_radius = sum(
                planner["planner"][name]
                for name in (
                    "vehicle_radius_m",
                    "tracking_error_budget_m",
                    "localization_error_budget_m",
                    "mapping_error_budget_m",
                    "planning_margin_m",
                )
            )
            self.assertGreaterEqual(ray_range_min, planner_safety_radius)

            speed_mission = ROOT / "config/runtime/missions/long_three_pillars_speed.yaml"
            speed_target = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                mission_file=speed_mission,
            )
            speed_parameters = yaml.safe_load(speed_target.read_text(encoding="utf-8"))["navigation_runtime_node"]["ros__parameters"]["navigation_runtime"]
            speed_planner = yaml.safe_load(Path(speed_parameters["config_path"]).read_text(encoding="utf-8"))
            self.assertEqual(speed_planner["traj_opt"]["boundary"]["max_vel"], 5.0)
            self.assertEqual(speed_planner["traj_opt"]["exp_traj"]["pos_constraint_type"], 2)

            capped_target = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                mission_file=speed_mission,
                speed_cap_mps=2.0,
            )
            capped_parameters = yaml.safe_load(capped_target.read_text(encoding="utf-8"))["navigation_runtime_node"]["ros__parameters"]["navigation_runtime"]
            capped_planner = yaml.safe_load(Path(capped_parameters["config_path"]).read_text(encoding="utf-8"))
            self.assertEqual(capped_planner["traj_opt"]["boundary"]["max_vel"], 2.0)

            external = runner._external_mode_params(
                session, ROOT / "config/runtime/external_mode.yaml"
            )
            external_parameters = yaml.safe_load(external.read_text(encoding="utf-8"))["px4_navigation_external_mode"]["ros__parameters"]
            self.assertNotIn("velocity_tracker", external_parameters["navigation"])
            self.assertNotIn("prefer_velocity_output", external_parameters["navigation"])

    def test_blocked_policy_is_forwarded_to_planner(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            mission = Path(temporary) / "blocked.yaml"
            mission.write_text(
                "mission:\n"
                "  planning:\n"
                "    unknown_policy: blocked\n"
                "    max_velocity_mps: 1.0\n"
                "    max_acceleration_mps2: 2.0\n"
                "    max_jerk_mps3: 6.0\n",
                encoding="utf-8",
            )
            target = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                mission_file=mission,
            )
            parameters = yaml.safe_load(target.read_text(encoding="utf-8"))
            planner_path = Path(
                parameters["navigation_runtime_node"]["ros__parameters"]
                ["navigation_runtime"]["config_path"]
            )
            planner = yaml.safe_load(planner_path.read_text(encoding="utf-8"))
            self.assertNotIn("frontend_in_known_free", planner["planner"])

    def test_speed_cap_materializes_one_resolved_mission_contract(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            source = ROOT / "config/runtime/missions/long_three_pillars_speed.yaml"

            resolved = runner._resolved_mission_file(session, source, 10.0)
            planning = runner._mission_planning(resolved)
            self.assertEqual(planning["max_velocity_mps"], 10.0)
            document = yaml.safe_load(resolved.read_text(encoding="utf-8"))
            self.assertEqual(document["mission"]["planning"]["max_velocity_mps"], 10.0)
            self.assertEqual(
                yaml.safe_load(source.read_text(encoding="utf-8"))["mission"]["planning"][
                    "max_velocity_mps"
                ],
                5.0,
            )

            runtime = json.loads(
                (session.directory / "runtime.json").read_text(encoding="utf-8")
            )
            contract = runtime["mission_contract"]
            self.assertEqual(contract["source_path"], str(source.resolve()))
            self.assertEqual(contract["resolved_path"], str(resolved.resolve()))
            self.assertEqual(contract["speed_cap_mps"], 10.0)
            self.assertEqual(contract["planning"]["max_velocity_mps"], 10.0)

            mapping = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                mission_file=resolved,
            )
            parameters = yaml.safe_load(mapping.read_text(encoding="utf-8"))[
                "navigation_runtime_node"
            ]["ros__parameters"]["navigation_runtime"]
            planner = yaml.safe_load(
                Path(parameters["config_path"]).read_text(encoding="utf-8")
            )
            self.assertEqual(planner["traj_opt"]["boundary"]["max_vel"], 10.0)
            self.assertEqual(parameters["mission_file"], str(resolved.resolve()))
            launch_command = runner._navigation_runtime_launch_command(mapping, resolved)
            self.assertIn(f"mission_file:={resolved.resolve()}", launch_command)

    def test_navigation_runtime_launch_omits_only_absent_mission(self) -> None:
        command = runner._navigation_runtime_launch_command(Path("mapping.yaml"), None)
        self.assertEqual(
            [item for item in command if item.startswith("mission_file:=")], []
        )

    def test_resolved_mission_rejects_invalid_speed_cap(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            source = ROOT / "config/runtime/missions/long_three_pillars_speed.yaml"
            for invalid in (True, 0.0, -1.0, float("nan"), float("inf")):
                with self.subTest(invalid=invalid):
                    with self.assertRaisesRegex(ValueError, "finite and positive"):
                        runner._resolved_mission_file(session, source, invalid)

    def test_objective_zero_disables_exp_jerk_without_bypass_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            mission = ROOT / "config/runtime/missions/open.yaml"
            target = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                mission_file=mission,
            )

            parameters = yaml.safe_load(target.read_text(encoding="utf-8"))["navigation_runtime_node"]["ros__parameters"]["navigation_runtime"]
            planner = yaml.safe_load(Path(parameters["config_path"]).read_text(encoding="utf-8"))
            self.assertEqual(
                planner["traj_opt"]["exp_traj"]["objective"]["jerk_penalty_weight"], 0.0
            )
            runtime = json.loads((session.directory / "runtime.json").read_text(encoding="utf-8"))
            self.assertNotIn("experimental_bypasses", runtime)

    def test_speed_certification_requirement_tracks_each_requested_sweep_cap(self) -> None:
        for profile in (
            "long_three_pillars_speed",
            "long_three_pillars_multiwaypoint",
            "long_open_featured_speed",
        ):
            self.assertEqual(
                runner._required_measured_speed_mps(profile, 5.0, None), 5.0
            )
            for cap in (2.0, 3.0, 4.0, 5.0, 6.0, 8.0):
                self.assertEqual(
                    runner._required_measured_speed_mps(profile, 5.0, cap), cap
                )
        self.assertIsNone(
            runner._required_measured_speed_mps("long_route", 5.0, 8.0)
        )
        for invalid in (True, 0.0, -1.0, math.nan, math.inf):
            self.assertIsNone(
                runner._required_measured_speed_mps(
                    "long_three_pillars_speed", 5.0, invalid
                )
            )

    def test_allow_unknown_policy_is_forwarded_to_exploration_planner(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            mission = Path(temporary) / "allow_unknown.yaml"
            mission.write_text(
                "mission:\n"
                "  planning:\n"
                "    unknown_policy: allow_unknown\n"
                "    max_velocity_mps: 1.0\n"
                "    max_acceleration_mps2: 2.0\n"
                "    max_jerk_mps3: 6.0\n",
                encoding="utf-8",
            )
            target = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                mission_file=mission,
            )
            parameters = yaml.safe_load(target.read_text(encoding="utf-8"))
            planner_path = Path(
                parameters["navigation_runtime_node"]["ros__parameters"]
                ["navigation_runtime"]["config_path"]
            )
            planner = yaml.safe_load(planner_path.read_text(encoding="utf-8"))
            self.assertNotIn("frontend_in_known_free", planner["planner"])
            self.assertTrue(planner["rog_map"]["raycasting"]["enable"])
            self.assertNotIn("unk_inflation_en", planner["rog_map"])

    def test_runtime_rejects_mission_limits_above_x500_envelope(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            mission = Path(temporary) / "too_fast.yaml"
            mission.write_text(
                "mission:\n"
                "  planning:\n"
                "    unknown_policy: allow_unknown\n"
                "    max_velocity_mps: 12.1\n"
                "    max_acceleration_mps2: 12.0\n"
                "    max_jerk_mps3: 30.0\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "exceeds X500 limit 12 m/s"):
                runner._mapping_params(
                    runner.Session(Path(temporary) / "session"),
                    ROOT / "config/runtime/mapping.yaml",
                    mission_file=mission,
                )

    def test_stress_profiles_have_explicit_mission_limits_and_worlds(self) -> None:
        profiles = {
            "corridor": 1.0,
            "speed": 2.0,
            "long_open": 1.5,
            "long_open_slow": 0.8,
            "long_featured": 1.5,
            "long_three_pillars": 3.0,
            "long_three_pillars_speed": 5.0,
            "long_three_pillars_multiwaypoint": 5.0,
            "long_open_featured_speed": 5.0,
            "navigation_generalization": 5.0,
            "single_pillar_speed": 8.0,
            "no_path": 1.0,
            "occlusion_featured": 1.0,
            "occlusion_degenerate": 1.0,
            "tunnel_irregular": 1.5,
            "tunnel_smooth": 1.5,
            "forest_clutter": 1.5,
        }
        for profile, expected_velocity in profiles.items():
            registry_entry = runner._map_registry().get(profile, {})
            world_name = {
                "speed": "open",
                "long_open_slow": "long_open",
            }.get(profile, registry_entry.get("world", profile) if isinstance(registry_entry, dict) else profile)
            world = ROOT / "src/uav_simulation/worlds" / f"{world_name}.sdf"
            mission_name = registry_entry.get("mission", profile) if isinstance(registry_entry, dict) else profile
            mission = ROOT / "config/runtime/missions" / f"{mission_name}.yaml"
            self.assertTrue(world.is_file(), profile)
            self.assertTrue(mission.is_file(), profile)
            mission_value = yaml.safe_load(mission.read_text(encoding="utf-8"))
            self.assertEqual(mission_value["mission"]["planning"]["unknown_policy"], "allow_unknown")
            planning = runner._mission_planning(mission)
            self.assertEqual(planning["max_velocity_mps"], expected_velocity)
            self.assertGreater(planning["max_acceleration_mps2"], 0.0)

    def test_stress_profiles_have_ground_truth_collision_geometry(self) -> None:
        for profile in (
            "open", "speed", "long_open", "long_open_slow", "long_featured",
            "corridor", "pillar", "occlusion", "occlusion_featured", "occlusion_degenerate",
            "tunnel_irregular", "tunnel_smooth", "forest_clutter", "long_three_pillars", "long_three_pillars_speed", "long_three_pillars_multiwaypoint", "long_open_featured_speed", "single_pillar_speed", "navigation_generalization", "no_path",
        ):
            obstacles = runner._collision_obstacles(profile)
            self.assertTrue(obstacles, profile)
            names = [obstacle["name"] for obstacle in obstacles]
            self.assertEqual(len(names), len(set(names)))
            for obstacle in obstacles:
                self.assertIn(obstacle["type"], {"box", "cylinder"})
                self.assertEqual(len(obstacle["center"]), 3)

    def test_map_registry_is_deterministic_and_truth_names_are_unique(self) -> None:
        registry = runner._map_registry()
        for profile in ("occlusion_featured", "occlusion_degenerate", "tunnel_irregular", "tunnel_smooth", "forest_clutter", "long_three_pillars", "long_three_pillars_speed", "long_three_pillars_multiwaypoint", "long_open_featured_speed", "single_pillar_speed", "navigation_generalization", "no_path"):
            descriptor = registry[profile]
            self.assertIn("world", descriptor)
            self.assertIn("mission", descriptor)
            self.assertEqual(len(descriptor["collision_truth"]), len(set(descriptor["collision_truth"])))
            self.assertTrue((ROOT / "src/uav_simulation/worlds" / f"{descriptor['world']}.sdf").is_file())
            self.assertTrue((ROOT / "config/runtime/missions" / f"{descriptor['mission']}.yaml").is_file())
        self.assertEqual(registry["no_path"]["expected_outcome"], "fail_closed")

    def test_long_three_pillars_has_explicit_route_obstacle_contract(self) -> None:
        descriptor = runner._map_registry()["long_three_pillars"]
        self.assertEqual(
            descriptor["route_obstacles"],
            ["long_three_pillar_01", "long_three_pillar_02", "long_three_pillar_03"],
        )
        self.assertEqual(descriptor["route_segment_waypoints"], [0, 1])
        self.assertEqual(len(descriptor["collision_truth"]), 21)
        self.assertTrue(set(descriptor["route_obstacles"]).issubset(descriptor["collision_truth"]))

    def test_long_three_pillars_uses_bounded_receding_horizon(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            target = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                mission_file=ROOT / "config/runtime/missions/long_three_pillars.yaml",
            )
            parameters = yaml.safe_load(target.read_text(encoding="utf-8"))["navigation_runtime_node"]["ros__parameters"]["navigation_runtime"]
            self.assertTrue(Path(parameters["config_path"]).is_file())

    def test_single_pillar_speed_uses_full_sensing_and_receding_horizon(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            target = runner._mapping_params(
                session,
                ROOT / "config/runtime/mapping.yaml",
                mission_file=ROOT / "config/runtime/missions/single_pillar_speed_pv.yaml",
            )
            parameters = yaml.safe_load(target.read_text(encoding="utf-8"))["navigation_runtime_node"]["ros__parameters"]["navigation_runtime"]
            self.assertTrue(Path(parameters["config_path"]).is_file())

    def test_long_three_pillars_acceptance_allows_obstacle_detour(self) -> None:
        self.assertEqual(
            runner._map_registry()["long_three_pillars"]["route_segment_waypoints"],
            [0, 1],
        )
        # The profile-specific route deviation limit is applied when the
        # scenario artifact is generated; open-space profiles retain 0.5 m.
        self.assertEqual(
            runner._acceptance_threshold_for_profile("long_three_pillars"),
            4.5,
        )
        self.assertEqual(
            runner._acceptance_threshold_for_profile("pillar"),
            1.5,
        )

    def test_cross_track_threshold_uses_declared_benchmark_envelope(self) -> None:
        self.assertEqual(
            runner._acceptance_threshold_for_profile(
                "long_three_pillars",
                {"benchmark": {"max_cross_track_m": 4.25}},
            ),
            4.25,
        )

    def test_cross_track_can_start_after_initial_pass_through(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            rows = [
                {"kind": "sample", "stream": "ground_truth_odometry", "timestamp_ns": index,
                 "payload": {"position": [0.0, 0.0, 0.0]}}
                for index in range(1, 21)
            ]
            rows.append(
                {"kind": "sample", "stream": "ground_truth_odometry", "timestamp_ns": 21,
                 "payload": {"position": [-2.0, 4.5, 3.0]}}
            )
            (session / "samples.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in rows),
                encoding="utf-8",
            )
            waypoints = [(-3.0, 4.5, 3.0), (4.0, 4.5, 3.0)]
            all_p95, _ = report._mission_cross_track_p95(session, waypoints)
            active_p95, active_count = report._mission_cross_track_p95(
                session, waypoints, 21
            )
            self.assertGreater(all_p95 or 0.0, 1.0)
            self.assertAlmostEqual(active_p95 or 0.0, 0.0)
            self.assertEqual(active_count, 1)

    def test_mission_acceptance_starts_after_allowed_initial_pass_through(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            mission_file = session / "mission.yaml"
            mission_file.write_text(
                "mission:\n"
                "  waypoints:\n"
                "    - position: [-3.0, 4.5, 3.0]\n"
                "    - position: [4.0, 4.5, 3.0]\n",
                encoding="utf-8",
            )
            (session / "scenario_config.yaml").write_text(
                yaml.safe_dump({
                    "scenario": {
                        "execution": "mission",
                        "mission_waypoint_count": 2,
                        "allow_initial_pass_through_skip": True,
                        "mission_file": str(mission_file),
                    }
                }),
                encoding="utf-8",
            )
            (session / "scenario.json").write_text(
                json.dumps({
                    "outcome": "COMPLETE",
                    "mission_complete_observed": True,
                    "waypoint_acceptance_events": [{
                        "waypoint_accepted": True,
                        "accepted_waypoint_index": 1,
                    }],
                }) + "\n",
                encoding="utf-8",
            )
            (session / "scenario.jsonl").write_text(
                json.dumps({
                    "kind": "waypoint_accepted",
                    "sim_time_ns": 21,
                    "payload": {
                        "waypoint_accepted": True,
                        "accepted_waypoint_index": 1,
                    },
                }) + "\n",
                encoding="utf-8",
            )
            rows = [
                {"kind": "sample", "stream": "ground_truth_odometry", "timestamp_ns": index,
                 "payload": {"position": [0.0, 0.0, 0.0]}}
                for index in range(1, 21)
            ]
            rows.append(
                {"kind": "sample", "stream": "ground_truth_odometry", "timestamp_ns": 21,
                 "payload": {"position": [-2.0, 4.5, 3.0]}}
            )
            (session / "samples.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in rows),
                encoding="utf-8",
            )

            acceptance = report._mission_acceptance(
                session, {}, {
                    "expected_outcome": "complete",
                    "outcome": "COMPLETE",
                    "mission_complete_observed": True,
                    "mission_waypoint_count": 2,
                    "waypoint_acceptance_events": [{
                        "waypoint_accepted": True,
                        "accepted_waypoint_index": 1,
                    }],
                },
                session,
            )

        self.assertTrue(acceptance["waypoint_acceptance_complete"])
        self.assertAlmostEqual(acceptance["cross_track_error_p95_m"] or 0.0, 0.0)
        self.assertEqual(acceptance["cross_track_sample_count"], 1)

    def test_long_three_pillars_speed_is_an_additive_two_waypoint_speed_benchmark(self) -> None:
        descriptor = runner._map_registry()["long_three_pillars_speed"]
        self.assertEqual(
            descriptor["route_obstacles"],
            [
                "long_three_speed_pillar_01",
                "long_three_speed_pillar_02",
                "long_three_speed_pillar_03",
            ],
        )
        self.assertEqual(descriptor["route_segment_waypoints"], [0, 1])
        self.assertEqual(len(descriptor["collision_truth"]), 30)
        self.assertEqual(
            descriptor["benchmark"]["speed_sweep_mps"],
            [2.0, 3.0, 4.0, 5.0, 6.0, 8.0],
        )
        mission = yaml.safe_load(
            (ROOT / "config/runtime/missions/long_three_pillars_speed.yaml").read_text()
        )["mission"]
        self.assertEqual(len(mission["waypoints"]), 2)
        self.assertEqual(mission["waypoints"][1]["position"], [140.0, 0.0, 3.0])
        self.assertEqual(mission["planning"]["max_velocity_mps"], 5.0)

    def test_long_three_pillars_multiwaypoint_is_a_nine_checkpoint_speed_contract(self) -> None:
        descriptor = runner._map_registry()["long_three_pillars_multiwaypoint"]
        self.assertEqual(descriptor["world"], "long_three_pillars_speed")
        self.assertEqual(descriptor["route_segment_waypoints"], [0, 8])
        self.assertEqual(descriptor["benchmark"]["waypoint_count"], 9)
        mission = yaml.safe_load(
            (ROOT / "config/runtime/missions/long_three_pillars_multiwaypoint.yaml").read_text()
        )["mission"]
        self.assertEqual(len(mission["waypoints"]), 9)
        self.assertEqual(mission["planning"]["max_velocity_mps"], 5.0)
        self.assertEqual(mission["waypoints"][-1]["behavior"], "stop")

    def test_simulator_uses_registry_world_for_multiwaypoint_profile(self) -> None:
        self.assertEqual(
            runner._world_name_for_profile("long_three_pillars_multiwaypoint"),
            "long_three_pillars_speed",
        )
        self.assertEqual(runner._world_name_for_profile("speed"), "open")
        self.assertEqual(runner._world_name_for_profile("smoke"), "px4_lio_smoke")

    def test_navigation_generalization_contract_covers_axes_and_behaviors(self) -> None:
        descriptor = runner._map_registry()["navigation_generalization"]
        benchmark = descriptor["benchmark"]
        self.assertEqual(
            benchmark["required_axes"],
            ["positive_x", "positive_y", "negative_x", "negative_y"],
        )
        self.assertEqual(descriptor["route_segment_waypoints"], [0, 10])
        self.assertEqual(len(descriptor["collision_truth"]), 17)
        self.assertTrue(set(descriptor["route_obstacles"]).issubset(descriptor["collision_truth"]))
        scenarios = {item["id"]: item for item in benchmark["scenario_matrix"]}
        self.assertEqual(
            set(scenarios),
            {
                "collinear_velocity",
                "sudden_high_speed_avoidance",
                "positive_y_axis",
                "local_narrow_gap",
                "negative_x_small_obstacles",
                "lane_change",
                "hairpin_uturn",
                "negative_y_terminal_recovery",
            },
        )
        mission = yaml.safe_load(
            (ROOT / "config/runtime/missions/navigation_generalization.yaml").read_text()
        )["mission"]
        self.assertEqual(len(mission["waypoints"]), 11)
        self.assertTrue(all(
            waypoint["behavior"] == "pass_through"
            for waypoint in mission["waypoints"][:-1]
        ))
        self.assertEqual(mission["waypoints"][-1]["behavior"], "stop")
        legs = [
            (
                right["position"][0] - left["position"][0],
                right["position"][1] - left["position"][1],
            )
            for left, right in zip(mission["waypoints"], mission["waypoints"][1:])
        ]
        self.assertTrue(any(dx > 0.0 and dy == 0.0 for dx, dy in legs))
        self.assertTrue(any(dx < 0.0 and dy == 0.0 for dx, dy in legs))
        self.assertTrue(any(dy > 0.0 and dx == 0.0 for dx, dy in legs))
        self.assertTrue(any(dy < 0.0 and dx == 0.0 for dx, dy in legs))
        self.assertAlmostEqual(
            sum(math.hypot(dx, dy) for dx, dy in legs),
            benchmark["route_length_m"],
        )

    def test_navigation_generalization_scene_resolves_comprehensive_variant(self) -> None:
        self.assertIn("navigation_generalization", runner.CANONICAL_SCENES)
        self.assertIn("comprehensive", runner.TEST_CASES)
        profile, metadata = runner._resolve_scene_profile(
            "navigation_generalization", "comprehensive", "nominal", None
        )
        self.assertEqual(profile, "navigation_generalization")
        self.assertEqual(metadata["scene"], "navigation_generalization")

    def test_canonical_scene_resolver_collapses_variants_without_new_make_profiles(self) -> None:
        self.assertEqual(
            runner._resolve_scene_profile("structured_obstacle", "positive", "nominal", None)[0],
            "occlusion_featured",
        )
        self.assertEqual(
            runner._resolve_scene_profile("structured_obstacle", "degenerate", "nominal", None)[0],
            "occlusion_degenerate",
        )
        self.assertEqual(
            runner._resolve_scene_profile("long_route", "positive", "slow", None)[0],
            "long_open_slow",
        )
        self.assertEqual(
            runner._resolve_scene_profile("planner_negative", "no_path", "nominal", None)[0],
            "no_path",
        )
        legacy, metadata = runner._resolve_scene_profile(None, "positive", "nominal", "corridor")
        self.assertEqual(legacy, "corridor")
        self.assertEqual(metadata["scene"], "legacy")

    def test_registry_seed_is_only_effective_for_stochastic_profiles(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            _, deterministic = runner._resolve_map_descriptor(session, "tunnel_smooth", 47)
            _, stochastic = runner._resolve_map_descriptor(session, "forest_clutter", 47)
            self.assertEqual(deterministic["seed"], 0)
            self.assertEqual(stochastic["seed"], 47)
            self.assertTrue((session.directory / "map_descriptor.json").is_file())
            resolved = session.directory / "resolved_forest_clutter.sdf"
            self.assertEqual(hashlib.sha256(resolved.read_bytes()).hexdigest(), stochastic["world_sha256"])

    def test_legacy_speed_alias_resolves_existing_world_asset(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = runner.Session(Path(temporary) / "session")
            world_name, descriptor = runner._resolve_map_descriptor(session, "speed", 0)
            self.assertEqual(world_name, "open")
            self.assertEqual(descriptor["profile"], "speed")
            self.assertEqual(descriptor["world"], "open")
            self.assertTrue((session.directory / "resolved_map.sdf").is_file())

    def test_profile_variants_share_mission_limits(self) -> None:
        for left, right in (("occlusion_featured", "occlusion_degenerate"),
                            ("tunnel_irregular", "tunnel_smooth")):
            self.assertEqual(runner._map_registry()[left]["family"], runner._map_registry()[right]["family"])
            left_planning = runner._mission_planning(
                ROOT / "config/runtime/missions" / f"{runner._map_registry()[left]['mission']}.yaml")
            right_planning = runner._mission_planning(
                ROOT / "config/runtime/missions" / f"{runner._map_registry()[right]['mission']}.yaml")
            self.assertEqual(left_planning, right_planning)

    def test_no_path_mission_crosses_sealed_wall(self) -> None:
        mission = yaml.safe_load(
            (ROOT / "config/runtime/missions/no_path.yaml").read_text(encoding="utf-8")
        )["mission"]
        wall = next(
            obstacle for obstacle in runner._collision_obstacles("no_path")
            if obstacle["name"] == "sealed_wall"
        )
        self.assertGreater(float(mission["waypoints"][0]["position"][0]), wall["center"][0])
        self.assertLess(0.0, wall["center"][0])

    def test_runtime_forces_legacy_rviz_environment_off(self) -> None:
        self.assertEqual(
            runner.NO_RVIZ_ENV,
            {
                "ENABLE_RVIZ": "0",
                "RVIZ_ENABLE": "0",
                "DISABLE_RVIZ": "1",
                "NAVIGATION_NO_RVIZ": "1",
            },
        )
        self.assertEqual(
            runner.RVIZ_ENV,
            {
                "ENABLE_RVIZ": "1",
                "RVIZ_ENABLE": "1",
                "DISABLE_RVIZ": "0",
                "NAVIGATION_NO_RVIZ": "0",
            },
        )

    def test_rviz_shell_explicitly_enables_visualizer_environment(self) -> None:
        command = runner._ros_shell(["rviz2"], enable_rviz=True)[-1]
        self.assertIn("export ENABLE_RVIZ=1 RVIZ_ENABLE=1 DISABLE_RVIZ=0 NAVIGATION_NO_RVIZ=0", command)

    def test_ros_shell_quotes_inherited_environment_values(self) -> None:
        with mock.patch.dict(os.environ, {"ROS_LOG_DIR": "/tmp/log dir;$unsafe"}, clear=False):
            command = runner._ros_shell(["echo", "ok"])[-1]
        self.assertIn("ROS_LOG_DIR='/tmp/log dir;$unsafe'", command)

    def test_process_registry_rejects_unvalidated_records_and_roles(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = process_group.Session(Path(temporary) / "session")
            (session.registry_path).write_text(
                json.dumps({"schema_version": 1, "processes": [{"pid": 1, "pgid": 0}]}),
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                session.records()
            with self.assertRaises(ValueError):
                session.start("../escape", ["true"], cwd=Path(temporary))

    def test_latest_resolution_rejects_external_symlink_target(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "runtime"
            root.mkdir()
            external = Path(temporary) / "external-session"
            external.mkdir()
            (root / "latest").symlink_to(external)
            with self.assertRaises(FileNotFoundError):
                process_group.resolve_latest(root)

    def test_stop_fails_closed_when_process_identity_cannot_be_read(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = process_group.Session(Path(temporary) / "session")
            session.registry_path.write_text(
                json.dumps({
                    "schema_version": 1,
                    "processes": [{
                        "role": "owned",
                        "pid": 999999,
                        "pgid": 999999,
                        "start_ticks": 123,
                    }],
                }),
                encoding="utf-8",
            )
            with mock.patch.object(process_group, "_group_exists", return_value=True), \
                    mock.patch.object(process_group, "_start_ticks", return_value=None), \
                    mock.patch.object(os, "killpg") as killpg:
                failures = session.stop(grace_s=0.0)
            self.assertTrue(failures)
            killpg.assert_not_called()

    def test_stop_rejects_invalid_grace_period(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = process_group.Session(Path(temporary) / "session")
            for grace in (float("nan"), float("inf"), -1.0):
                with self.assertRaises(ValueError):
                    session.stop(grace_s=grace)

    def test_rviz_command_uses_sim_clock_without_legacy_topic_remap(self) -> None:
        command = runner._rviz_command(use_sim_time=True)
        self.assertIn("--ros-args", command)
        self.assertIn("-p", command)
        self.assertIn("use_sim_time:=true", command)
        self.assertNotIn("/livox/lidar:=/lidar/points", command)

    def test_dataset_replay_uses_recorded_time_as_ros_clock(self) -> None:
        context = {
            "bag": Path("/data/lio"),
            "input": {
                "lidar_topic": "/livox/lidar",
                "imu_topic": "/livox/imu",
            },
        }
        command = runner._dataset_replay_command(context, 1.0)
        self.assertIn("--clock", command)
        self.assertIn("/livox/lidar:=/lidar/points", command)
        self.assertIn("/livox/imu:=/lidar/imu", command)

        source = (ROOT / "tools/runtime/runner.py").read_text(encoding="utf-8")
        dataset_body = source[source.index("def run_dataset("):source.index("def _sim_prerequisites(")]
        self.assertEqual(dataset_body.count('"use_sim_time:=true"'), 1)
        self.assertIn(
            "_navigation_runtime_launch_command(mapping_config, shadow_mission_file)",
            dataset_body,
        )
        self.assertNotIn('"use_sim_time:=false"', dataset_body)

    def test_dataset_shadow_goal_is_horizontal_finite_and_distance_bounded(self) -> None:
        goal = dataset_shadow_planning.relative_goal(
            (1.0, 2.0, 3.0), (3.0, 4.0, 9.0), 5.0
        )
        self.assertEqual(goal, (4.0, 6.0, 3.0))
        with self.assertRaisesRegex(ValueError, "too small"):
            dataset_shadow_planning.relative_goal((0, 0, 0), (0, 0, 1), 5.0)
        with self.assertRaisesRegex(ValueError, "finite"):
            dataset_shadow_planning.relative_goal((0, 0, math.nan), (1, 0, 0), 5.0)

    def test_dataset_shadow_commands_require_matching_goal_identity(self) -> None:
        goal = SimpleNamespace(mission_id="shadow", waypoint_index=2, request_id=7)
        command = SimpleNamespace(
            localization_epoch=3, goal_epoch=9, mission_id="shadow",
            waypoint_index=2, request_id=7,
        )
        self.assertTrue(
            dataset_shadow_planning.command_matches_shadow_goal(command, goal, 3)
        )
        command.request_id = 8
        self.assertFalse(
            dataset_shadow_planning.command_matches_shadow_goal(command, goal, 3)
        )
        command.request_id = 7
        command.localization_epoch = 4
        self.assertFalse(
            dataset_shadow_planning.command_matches_shadow_goal(command, goal, 3)
        )

    def test_monitor_keeps_source_timestamp_high_water(self) -> None:
        stats = monitor.StreamStats("imu", "/imu")
        self.assertTrue(stats.update(100, 1_000))
        self.assertTrue(stats.update(90, 2_000))
        self.assertTrue(stats.update(101, 3_000))
        snapshot = stats.as_dict()
        self.assertEqual(snapshot["timestamp_regression_count"], 1)
        self.assertEqual(snapshot["last_stamp_ns"], 101)
        self.assertEqual(snapshot["maximum_gap_ms"], 1.0e-6)
        stats.update(0, 4_000)
        self.assertEqual(stats.as_dict()["invalid_source_timestamp_count"], 1)

    def test_report_reasons_are_deduplicated_in_first_seen_order(self) -> None:
        self.assertEqual(
            report._dedupe_reasons(["planner failed", "planner failed", "cleanup failed"]),
            ["planner failed", "cleanup failed"],
        )

    def test_sparse_pointcloud_nonfinite_returns_are_diagnostic_not_message_invalid(self) -> None:
        payload = {"is_dense": False, "sampled_nonfinite_points": 5}
        dense_payload = {"is_dense": True, "sampled_nonfinite_points": 5}
        self.assertEqual(
            monitor._pointcloud_nonfinite_message(payload, payload["sampled_nonfinite_points"]),
            False,
        )
        self.assertEqual(
            monitor._pointcloud_nonfinite_message(
                dense_payload, dense_payload["sampled_nonfinite_points"]
            ),
            True,
        )
        stats = monitor.StreamStats("lidar", "/lidar/points")
        stats.update(
            1_000_000_000,
            1_000_000_000,
            nonfinite=0,
            sampled_nonfinite_points=5,
        )
        snapshot = stats.as_dict()
        self.assertEqual(snapshot["nonfinite_message_count"], 0)
        self.assertEqual(snapshot["sampled_nonfinite_point_count"], 5)

    def test_dataset_shadow_planning_is_product_goal_with_fail_closed_report_evidence(self) -> None:
        source = (ROOT / "tools/runtime/runner.py").read_text(encoding="utf-8")
        dataset_body = source[source.index("def run_dataset("):source.index("def _sim_prerequisites(")]
        self.assertIn('"dataset_shadow_planning"', dataset_body)
        self.assertIn('"tools/runtime/dataset_shadow_planning.py"', dataset_body)
        self.assertNotIn(
            'raise RuntimeError(f"dataset shadow planning exited', dataset_body
        )
        helper_source = (ROOT / "tools/runtime/dataset_shadow_planning.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("PropagatedOdometry", helper_source)
        self.assertIn("on_propagated_odometry", helper_source)
        self.assertIn("message.odometry", helper_source)
        self.assertIn('route.waypoint_ids.append("shadow_target")', helper_source)
        self.assertIn(
            "if not command_matches_shadow_goal(\n            message, goal_message, goal_localization_epoch\n        ):",
            helper_source,
        )
        self.assertNotIn('create_subscription(Odometry, "/lio/odometry_propagated"', helper_source)
        self.assertIn("EMER before committing a READY shadow command", helper_source)
        self.assertIn("publish_teardown(latest_odom)", helper_source)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", helper_source)
        runtime = {"dataset_shadow_planning": {"enabled": True, "goal_distance_m": 5.0}}
        planning = {
            "planning_total_us": {"sample_count": 4},
            "rolling_bundle_trace": {"record_count": 4},
        }
        valid = {
            "status": "PASS",
            "goal_published": True,
            "ready_command_count": 20,
            "unique_ready_generations": [1, 2],
            "emergency_command_count": 0,
        }
        self.assertEqual(
            report._dataset_shadow_planning_reasons(runtime, valid, planning), []
        )
        trace_timing = {
            "planning_total_us": {"sample_count": 0},
            "rolling_bundle_trace": {
                "record_count": 2,
                "records": [
                    {"planning_latency_ms": 1.25},
                    {"planning_latency_ms": 0.50},
                ],
            },
        }
        self.assertEqual(
            report._dataset_shadow_planning_reasons(
                runtime, valid, trace_timing
            ),
            [],
        )
        missing_timing = {
            "planning_total_us": {"sample_count": 0},
            "rolling_bundle_trace": {
                "record_count": 1,
                "records": [{"planning_latency_ms": None}],
            },
        }
        self.assertIn(
            "dataset shadow planning has no planner timing samples",
            report._dataset_shadow_planning_reasons(
                runtime, valid, missing_timing
            ),
        )
        missing = dict(valid, ready_command_count=0, unique_ready_generations=[])
        self.assertIn(
            "dataset shadow planning produced no committed READY command",
            report._dataset_shadow_planning_reasons(runtime, missing, planning),
        )
        emergency = dict(valid, emergency_command_count=1)
        self.assertIn(
            "dataset shadow planning emitted an emergency command",
            report._dataset_shadow_planning_reasons(runtime, emergency, planning),
        )
        failed = dict(
            missing,
            status="FAIL",
            failure="planner backend emitted EMER before committing a READY shadow command",
        )
        self.assertIn(
            "dataset shadow planning did not complete: planner backend emitted EMER before "
            "committing a READY shadow command",
            report._dataset_shadow_planning_reasons(runtime, failed, planning),
        )
        self.assertEqual(
            report._dataset_shadow_planning_reasons(
                {"dataset_shadow_planning": {"enabled": False}}, {}, {}
            ),
            [],
        )

    def test_dataset_replay_passes_one_explicit_main_backup_mission_contract(self) -> None:
        source = (ROOT / "tools/runtime/runner.py").read_text(encoding="utf-8")
        dataset_body = source[source.index("def run_dataset("):source.index("def _sim_prerequisites(")]
        self.assertIn("DATASET_SHADOW_MISSION", source)
        self.assertIn("shadow_mission_file = _resolved_mission_file(", dataset_body)
        self.assertIn("mission_file=shadow_mission_file", dataset_body)
        self.assertIn("_navigation_runtime_launch_command(mapping_config, shadow_mission_file)", dataset_body)
        self.assertIn('"main_unknown_policy": "allow_unknown"', dataset_body)
        self.assertIn('"backup_unknown_policy": "require_known_free"', dataset_body)

        mission = yaml.safe_load(
            (ROOT / "config/runtime/missions/recorded_replay.yaml").read_text(
                encoding="utf-8"
            )
        )
        planning = mission["mission"]["planning"]
        self.assertEqual(planning["unknown_policy"], "allow_unknown")
        self.assertEqual(planning["max_velocity_mps"], 12.0)
        self.assertEqual(planning["max_acceleration_mps2"], 12.0)
        self.assertEqual(planning["max_jerk_mps3"], 30.0)

    def test_dataset_raw_observers_use_reliable_product_capacity(self) -> None:
        config = runner.load_config("dataset.yaml")
        self.assertEqual(
            monitor._observer_qos_contract("dataset", config, "imu"),
            ("reliable", 4096),
        )
        self.assertEqual(
            monitor._observer_qos_contract("dataset", config, "lidar"),
            ("reliable", 16),
        )
        self.assertEqual(
            monitor._observer_qos_contract("dataset", config, "corrected_odometry"),
            ("best_effort", 100),
        )
        self.assertEqual(
            monitor._observer_qos_contract("sim", config, "imu"),
            ("best_effort", 100),
        )
        broken = runner.load_config("dataset.yaml")
        broken["fast_lio"]["ros__parameters"]["runtime"]["lidar_queue_capacity"] = 0
        with self.assertRaisesRegex(ValueError, "positive lidar_queue_capacity"):
            monitor._observer_qos_contract("dataset", broken, "lidar")

    def test_dataset_source_counts_are_exact_and_fail_closed(self) -> None:
        streams = {"imu": {"sample_count": 55435}, "lidar": {"sample_count": 2772}}
        runtime = {"dataset_context": {"expected_stream_counts": {
            "imu": 55435, "lidar": 2772,
        }}}
        self.assertEqual(report._dataset_source_count_reasons(streams, runtime), [])
        self.assertTrue(streams["imu"]["source_count_complete"])
        self.assertTrue(streams["lidar"]["source_count_complete"])

        short = {"imu": {"sample_count": 55435}, "lidar": {"sample_count": 2759}}
        self.assertEqual(
            report._dataset_source_count_reasons(short, runtime),
            ["lidar source count mismatch: observed 2759, expected 2772"],
        )
        duplicate = {"imu": {"sample_count": 55436}, "lidar": {"sample_count": 2772}}
        self.assertEqual(
            report._dataset_source_count_reasons(duplicate, runtime),
            ["imu source count mismatch: observed 55436, expected 55435"],
        )

        missing = {"imu": {"sample_count": 55435}, "lidar": {"sample_count": 2772}}
        reasons = report._dataset_source_count_reasons(missing, {})
        self.assertIn("dataset expected source counts are missing", reasons)
        self.assertIn("imu expected source count is invalid or missing", reasons)
        self.assertIn("lidar expected source count is invalid or missing", reasons)

    def test_dataset_report_verdict_fails_for_short_raw_source_count(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            (session / "runtime.json").write_text(json.dumps({
                "failures": [],
                "dataset_context": {"expected_stream_counts": {
                    "imu": 55435, "lidar": 2772,
                }},
            }), encoding="utf-8")
            snapshot = {
                "streams": {
                    "imu": {"received": 55435, "mean_rate_hz": 200.0},
                    "lidar": {"received": 2759, "mean_rate_hz": 10.0},
                    "corrected_odometry": {"received": 1, "mean_rate_hz": 10.0},
                    "propagated_odometry": {"received": 1, "mean_rate_hz": 50.0},
                },
                "diagnostics": {
                    "state": "TRACKING",
                    "navigation_valid": True,
                    "values": {
                        "state": "TRACKING",
                        "current_queue_depth": 0,
                        "imu_drop_count": 0,
                        "lidar_drop_count": 0,
                    },
                },
            }
            config = runner.load_config("dataset.yaml")
            result = report._dataset_report(session, config, snapshot, ROOT)
            self.assertEqual(result["verdict"], "FAIL")
            self.assertIn(
                "lidar source count mismatch: observed 2759, expected 2772",
                result["reasons"],
            )

    def test_dataset_runner_normalizes_topics_and_waits_for_exact_drain(self) -> None:
        context = {"input": {"imu_topic": "/custom/gyro", "lidar_topic": "/custom/cloud"}}
        expected = runner._expected_dataset_stream_counts(
            context, {"/custom/gyro": 7, "/custom/cloud": 3}
        )
        self.assertEqual(expected, {"imu": 7, "lidar": 3})
        counts = {"imu": 7, "lidar": 2, "corrected_odometry": 1,
                  "propagated_odometry": 1}
        with mock.patch.object(runner, "_stream_count", side_effect=lambda _, name: counts[name]):
            self.assertFalse(runner._dataset_outputs_drained(mock.Mock(), expected))
        counts["lidar"] = 3
        with mock.patch.object(runner, "_stream_count", side_effect=lambda _, name: counts[name]):
            self.assertTrue(runner._dataset_outputs_drained(mock.Mock(), expected))

    def test_dataset_playback_summary_exposes_actual_requested_rate_fraction(self) -> None:
        runtime = {
            "replay_started_wall_ns": 10_000_000_000,
            "replay_finished_wall_ns": 20_000_000_000,
            "dataset_context": {"source_duration_ns": 20_000_000_000},
        }
        playback = report._dataset_playback_summary(runtime, 2.0)
        self.assertTrue(playback["available"])
        self.assertEqual(playback["achieved_rate"], 2.0)
        self.assertEqual(playback["requested_rate_fraction"], 1.0)

    def test_dataset_mapping_integrity_rejects_replaced_observations(self) -> None:
        reasons = report._mapping_integrity_reasons({
            "received_observation_count": 100,
            "observation_rejected_before_inbox_count": 0,
            "accepted_observation_count": 100,
            "dropped_cloud_count": 49,
            "stale_input_count": 0,
            "corrected_pair_mismatch_count": 0,
            "invalid_execution_state_count": 0,
            "processing_exception_count": 0,
            "observation_accounting_valid": 1,
            "observation_replaced_pending_count": 49,
            "observation_discarded_pending_count": 0,
            "mapping_published_count": 51,
            "mapping_failed_count": 0,
            "mapping_pending_count": 0,
            "mapping_in_flight_count": 0,
            **_mapping_outcomes(51),
            "world_revision": 51,
        })
        self.assertEqual(reasons, ["mapping replaced an unconsumed cloud: 49"])

    def test_dataset_mapping_integrity_accepts_exact_once_consumption(self) -> None:
        self.assertEqual(report._mapping_integrity_reasons({
            "received_observation_count": 100,
            "observation_rejected_before_inbox_count": 0,
            "accepted_observation_count": 100,
            "dropped_cloud_count": 0,
            "stale_input_count": 0,
            "corrected_pair_mismatch_count": 0,
            "invalid_execution_state_count": 0,
            "processing_exception_count": 0,
            "observation_accounting_valid": 1,
            "observation_replaced_pending_count": 0,
            "observation_discarded_pending_count": 0,
            "mapping_published_count": 100,
            "mapping_failed_count": 0,
            "mapping_pending_count": 0,
            "mapping_in_flight_count": 0,
            **_mapping_outcomes(100),
            "world_revision": 100,
        }), [])

    def test_dataset_mapping_integrity_rejects_missing_or_nonadvancing_outcomes(self) -> None:
        legacy = {
            "mapping_published_count": 1,
            "world_revision": 1,
        }
        self.assertIn("runtime binary may be stale", " ".join(
            report._mapping_integrity_reasons(legacy)
        ))
        bad = {
            "received_observation_count": 2,
            "accepted_observation_count": 2,
            "mapping_published_count": 2,
            "mapping_failed_count": 0,
            "mapping_pending_count": 0,
            "mapping_in_flight_count": 0,
            "observation_discarded_pending_count": 0,
            "observation_replaced_pending_count": 0,
            "world_revision": 1,
            **_mapping_outcomes(1, mapping_outcome_below_ground_count=1),
        }
        reasons = report._mapping_integrity_reasons(bad)
        self.assertIn("mapping rejected odometry below virtual ground: 1", reasons)

        missing_stamp = {
            "mapping_published_count": 1,
            "world_revision": 1,
            **_mapping_outcomes(1),
        }
        missing_stamp.pop("observation_stamp_ns")
        self.assertIn(
            "mapping published-world observation stamp is unavailable",
            report._mapping_integrity_reasons(missing_stamp),
        )

    def test_dataset_mapping_integrity_rejects_unterminalized_observation(self) -> None:
        reasons = report._mapping_integrity_reasons({
            "received_observation_count": 100,
            "observation_rejected_before_inbox_count": 0,
            "accepted_observation_count": 100,
            "dropped_cloud_count": 0,
            "observation_replaced_pending_count": 0,
            "observation_discarded_pending_count": 0,
            "mapping_published_count": 99,
            "mapping_failed_count": 0,
            "mapping_pending_count": 0,
            "mapping_in_flight_count": 0,
            "observation_accounting_valid": 0,
        })
        self.assertIn("mapping observation accounting invariant failed", reasons)
        self.assertIn(
            "mapping accepted-observation conservation equation failed", reasons
        )

    def test_sim_and_dataset_share_mapping_integrity_gate(self) -> None:
        source = (ROOT / "tools/runtime/report.py").read_text(encoding="utf-8")
        dataset_body = source[source.index("def _dataset_report("):
                              source.index("def _sim_report(")]
        sim_body = source[source.index("def _sim_report("):
                          source.index("def build(")]
        self.assertIn("reasons.extend(_mapping_integrity_reasons(navigation_mapping))",
                      dataset_body)
        self.assertIn("reasons.extend(_mapping_integrity_reasons(navigation_mapping))",
                      sim_body)

    def test_external_mode_gui_launch_passes_static_mission_file(self) -> None:
        command = runner._external_mode_launch_command(
            Path("/tmp/external_mode_params.yaml"),
            Path("/tmp/mission.yaml"),
        )
        self.assertIn("use_sim_time:=true", command)
        self.assertIn("mission_file:=/tmp/mission.yaml", command)

    def test_static_sensor_tf_is_derived_from_each_estimator_extrinsic(self) -> None:
        expected = {
            "dataset.yaml": (0.019391, 0.000278, -0.080926),
            "sim.yaml": (0.011, 0.02329, -0.04412),
        }
        for config_name, expected_xyz in expected.items():
            xyz, rpy = runner._lidar_to_imu_launch_arguments(
                runner.load_config(config_name)
            )
            for actual, reference in zip(map(float, xyz.split()), expected_xyz):
                self.assertAlmostEqual(actual, reference, places=12)
            for value in map(float, rpy.split()):
                self.assertAlmostEqual(value, 0.0, places=12)

    def test_product_rviz_config_shows_current_planner_input_and_odometry(self) -> None:
        config = runner.RVIZ_CONFIG.read_text(encoding="utf-8")
        self.assertIn("Fixed Frame: lio_odom", config)
        self.assertIn("Value: /lio/odometry_corrected", config)
        self.assertIn("Value: /lio/registered_points", config)
        self.assertNotIn("/lio/local_map", config)
        self.assertIn("Class: rviz_default_plugins/Odometry", config)

    def test_stop_discovers_all_owned_runtime_sessions(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name in ("sim-old", "sim-new"):
                session = root / name
                session.mkdir()
                (session / "processes.json").write_text("{}", encoding="utf-8")
            (root / "latest").symlink_to("sim-new")
            self.assertEqual(
                [path.name for path in runner._runtime_session_paths(root)],
                ["sim-old", "sim-new"],
            )

    def test_latest_resolution_recovers_from_dangling_symlink(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            older = root / "sim-20260821T010000-1"
            newer = root / "sim-20260821T020000-2"
            for session in (older, newer):
                session.mkdir()
                (session / "processes.json").write_text("{}", encoding="utf-8")
            (root / "latest").symlink_to("missing-session")

            path, recovered = runner._resolve_latest_or_newest(root)
            self.assertEqual(path, newer)
            self.assertTrue(recovered)

    def test_status_without_session_has_no_traceback(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            original_root = runner.ARTIFACT_ROOT
            runner.ARTIFACT_ROOT = Path(temporary) / "runtime"
            error = io.StringIO()
            try:
                with redirect_stderr(error):
                    self.assertEqual(runner.status(), 1)
            finally:
                runner.ARTIFACT_ROOT = original_root
            self.assertIn("No runtime session", error.getvalue())

    def test_stop_finalizes_every_session_missing_a_report(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            root = Path(temporary) / "runtime"
            sessions = [runner.Session.create(root, name) for name in ("sim-old", "sim-new")]
            for session in sessions:
                session.write_state({"workflow": "sim", "headless": True})

            original_root = runner.ARTIFACT_ROOT
            runner.ARTIFACT_ROOT = root

            def fake_build(session_path, *_args, **_kwargs):
                payload = {"verdict": "FAIL"}
                (session_path / "report.json").write_text(
                    json.dumps(payload) + "\n", encoding="utf-8"
                )
                return payload

            try:
                with mock.patch.object(runner.report, "build", side_effect=fake_build) as build:
                    self.assertEqual(runner.stop(), 0)
                    self.assertEqual(build.call_count, 2)
            finally:
                runner.ARTIFACT_ROOT = original_root

            for session in sessions:
                self.assertTrue((session.directory / "report.json").is_file())

    def test_setup_failure_recovery_finalizes_report(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            root = Path(temporary) / "runtime"
            session = runner.Session.create(root, "sim-check")
            session.write_state({"workflow": "sim", "headless": True})
            original_root = runner.ARTIFACT_ROOT
            runner.ARTIFACT_ROOT = root
            try:
                with redirect_stderr(io.StringIO()):
                    runner._recover_unfinalized_session(RuntimeError("setup exploded"))
            finally:
                runner.ARTIFACT_ROOT = original_root

            for name in ("report.json", "REPORT.html"):
                self.assertTrue((session.directory / name).is_file(), name)
            payload = json.loads((session.directory / "report.json").read_text(encoding="utf-8"))
            self.assertIn("runner setup: setup exploded", payload["reasons"])

    def test_gazebo_native_observer_samples_registered_processes_and_psi(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            session = runner.Session(Path(temporary) / "session")
            process = session.start(
                "short_lived",
                [sys.executable, "-c", "import time; time.sleep(1)"],
                cwd=ROOT,
            )
            processes = gazebo_native_observer._process_samples(session.directory)
            self.assertTrue(any(item["role"] == "short_lived" for item in processes))
            first = next(item for item in processes if item["role"] == "short_lived")
            self.assertIn("rss_bytes", first["stat"])
            self.assertIn("sched_runtime_ns", first["stat"])
            psi = gazebo_native_observer._psi_snapshot()
            self.assertTrue(set(psi).issubset({"cpu", "memory", "io"}))
            session.stop()
            process.wait(timeout=2.0)

    def test_gazebo_native_observer_summary_is_diagnostic_only(self) -> None:
        samples = [
            {
                "kind": "world_stats",
                "payload": {
                    "sim_time": {"sec": 1, "nsec": 500},
                    "real_time_factor": 0.95,
                },
            },
            {"kind": "world_clock", "payload": {"sim": {"sec": 2, "nsec": 0}}},
            {
                "kind": "process_sample",
                "processes": [{"role": "px4_gazebo"}, {"role": "mapping"}],
            },
            {"kind": "psi_sample", "psi": {"cpu": "some avg10=0.00"}},
        ]
        summary = gazebo_native_observer._summarize(samples)
        self.assertEqual(summary["gazebo_native"]["world_stats"]["samples"], 1)
        self.assertEqual(summary["gazebo_native"]["world_stats"]["first_sim_time_ns"], 1_000_000_500)
        self.assertEqual(summary["gazebo_native"]["world_clock"]["last_sim_time_ns"], 2_000_000_000)
        self.assertEqual(summary["process_roles"]["px4_gazebo"], 1)
        self.assertEqual(summary["psi_samples"], 1)

    def test_gazebo_native_observer_requires_both_delivered_streams(self) -> None:
        self.assertTrue(gazebo_native_observer._native_streams_observed(1, 1))
        self.assertFalse(gazebo_native_observer._native_streams_observed(1, 0))
        self.assertFalse(gazebo_native_observer._native_streams_observed(0, 1))

    def test_gazebo_native_summary_reports_process_and_psi_counts(self) -> None:
        samples = [
            {"kind": "process_sample", "processes": [{"role": "gz"}, {"role": "gz"}]},
            {"kind": "psi_sample", "psi": {"cpu": "some avg10=0.00"}},
        ]
        summary = gazebo_native_observer._summarize(samples)
        self.assertEqual(summary["process_roles"], {"gz": 2})
        self.assertEqual(summary["psi_samples"], 1)

    def test_gazebo_native_observer_runner_hook_records_artifacts(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            session = runner.Session(Path(temporary) / "session")
            process = SimpleNamespace()
            with mock.patch.object(runner.Session, "start", return_value=process) as start:
                result = runner._start_gazebo_native_observer(session, "test_world", "gz")

            self.assertIs(result, process)
            command = start.call_args.args[1]
            self.assertIn("gazebo_native_observer.py", command[1])
            self.assertIn("--world", command)
            runtime = json.loads((session.directory / "runtime.json").read_text(encoding="utf-8"))
            observer = runtime["gazebo_native_observer"]
            self.assertEqual(observer["world_stats_topic"], "/world/test_world/stats")
            self.assertEqual(observer["world_clock_topic"], "/world/test_world/clock")
            self.assertEqual(observer["verdict_owner"], "diagnostic_only")

    def test_report_includes_gazebo_native_diagnostics_without_reasons(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            session = Path(temporary) / "session"
            session.mkdir()
            (session / "gazebo_native_summary.json").write_text(
                json.dumps({"schema_version": 1, "psi_samples": 2}) + "\n",
                encoding="utf-8",
            )
            runtime = {
                "gazebo_native_observer": {
                    "world_stats_topic": "/world/test/stats",
                    "verdict_owner": "diagnostic_only",
                }
            }
            diagnostics = report._gazebo_native_diagnostics(session, runtime)

            self.assertEqual(diagnostics["psi_samples"], 2)
            self.assertEqual(diagnostics["observer"]["world_stats_topic"], "/world/test/stats")
            self.assertEqual(diagnostics["verdict_owner"], "diagnostic_only")
            self.assertEqual(report._gazebo_native_diagnostics(session, runtime)["verdict_owner"], "diagnostic_only")

    def test_report_analysis_failure_writes_minimal_artifact_set(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            session = Path(temporary) / "session"
            session.mkdir()
            with mock.patch.object(
                report,
                "_sim_report",
                side_effect=RuntimeError("analysis exploded"),
            ):
                result = report.build(
                    session,
                    "sim",
                    ROOT / "config/runtime/sim.yaml",
                    ROOT,
                )

            self.assertEqual(result["verdict"], "FAIL")
            for name in (
                "report.json",
                "REPORT.html",
                "REPORT_BUILD_ERROR.txt",
            ):
                self.assertTrue((session / name).is_file(), name)
            self.assertIn(
                "analysis exploded",
                (session / "REPORT_BUILD_ERROR.txt").read_text(encoding="utf-8"),
            )

    def test_observation_complete_preserves_evaluated_failure_verdict(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            session = Path(temporary) / "session"
            session.mkdir()
            (session / "runtime.json").write_text("{}\n", encoding="utf-8")
            fake_report = {
                "workflow": "sim",
                "verdict": "FAIL",
                "reasons": ["stream freshness violation"],
            }
            import flight_review_report

            with mock.patch.object(report, "_sim_report", return_value=fake_report), \
                    mock.patch.object(report, "_process_failures", return_value=[]), \
                    mock.patch.object(flight_review_report, "render", return_value=session / "REPORT.html"):
                result = report._build_complete_report(
                    session,
                    "sim",
                    ROOT / "config/runtime/sim.yaml",
                    ROOT,
                    observation_complete=True,
                )
            self.assertEqual(result["verdict"], "FAIL")
            self.assertEqual(result["reasons"], ["stream freshness violation"])
            self.assertTrue(result["observation_complete"])
            self.assertEqual(result["observation_status"], "OBSERVATION_COMPLETE")

    def test_simulation_config_is_lio_only_at_startup(self) -> None:
        config = runner.load_config("sim.yaml")["fast_lio"]["ros__parameters"]
        prior = config["initial_prior"]
        self.assertEqual(prior["source"], "zero")
        self.assertEqual(prior["source_frame"], "lio_odom")
        self.assertEqual(prior["source_frame_transform"], "same_frame")
        self.assertTrue(config["output"]["publish_registered_points"])
        local_map = config["mapping"]["local_map"]
        self.assertGreater(local_map["absolute_map_point_guard"], 0)
        propagated = config["propagated_odometry"]
        self.assertNotIn("enabled", propagated)
        self.assertEqual(propagated["imu_history_duration_s"], 1.0)
        self.assertEqual(propagated["maximum_correction_age_s"], 0.50)

    def test_simulation_bridge_gates_are_explicit_profile_parameters(self) -> None:
        config = runner.load_config("sim.yaml")
        external = config["px4_external_odometry_bridge"]["ros__parameters"]
        self.assertEqual(external["timing"]["clock_domain"], "simulation_time")
        self.assertEqual(external["external_odometry"]["maximum_age_s"], 0.5)
        ingress = config["px4_odometry_bridge"]["ros__parameters"]
        self.assertNotIn("simulation_clock", ingress)
        self.assertEqual(ingress["reset"]["stable_samples_after_reset"], 3)
        self.assertEqual(
            config["fast_lio"]["ros__parameters"]["timing"]["clock_domain"],
            "simulation_time",
        )
        launcher = (ROOT / "tools/simulation/run_px4_mid360.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("export PX4_PARAM_SIM_GZ_EN_BARO=1", launcher)
        self.assertIn("export PX4_PARAM_UXRCE_DDS_SYNCT=0", launcher)
        startup = (ROOT / "tools/simulation/px4_mid360_startup.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("param show UXRCE_DDS_SYNCT", startup)
        self.assertIn("param show MPC_YAWRAUTO_MAX", startup)
        self.assertIn("param show MPC_YAWRAUTO_ACC", startup)
        self.assertIn("param show MC_YAWRATE_MAX", startup)

    def test_planner_has_one_unknown_space_policy_owner(self) -> None:
        planner_config = yaml.safe_load(
            (ROOT / "src/runtime/navigation_runtime/config/planner.yaml").read_text(
                encoding="utf-8"
            )
        )
        planner = planner_config["planner"]
        self.assertNotIn("frontend_in_known_free", planner)
        self.assertNotIn("visual_process", planner)
        astar = planner_config["astar"]
        self.assertIn("visual_process", astar)

    def test_simulation_bridge_isolates_pointcloud_from_clock_and_imu(self) -> None:
        bridge_root = ROOT / "src/uav_simulation/bridge"
        control = yaml.safe_load(
            (bridge_root / "px4_mid360_control_bridge.yaml").read_text(
                encoding="utf-8"
            )
        )
        lidar = yaml.safe_load(
            (bridge_root / "px4_mid360_lidar_bridge.yaml").read_text(
                encoding="utf-8"
            )
        )
        control_topics = {item["ros_topic_name"] for item in control}
        lidar_topics = {item["ros_topic_name"] for item in lidar}
        self.assertEqual(
            control_topics,
            {"/clock", "/lidar/imu", "/sim/ground_truth/odometry"},
        )
        self.assertEqual(lidar_topics, {"/lidar/points"})
        self.assertFalse(control_topics & lidar_topics)
        runner_source = (RUNTIME / "runner.py").read_text(encoding="utf-8")
        self.assertIn('session.start("bridge"', runner_source)
        self.assertIn('session.start("bridge_lidar"', runner_source)
        self.assertIn('session.start("visibility_bridge"', runner_source)
        bridge_launch = runner_source[
            runner_source.index('session.start("bridge"'):
            runner_source.index('session.start("px4_ingress"')
        ]
        self.assertNotIn("use_sim_time:=true", bridge_launch)

    def test_simulation_clock_is_a_first_class_freshness_stream(self) -> None:
        runtime = runner.load_config("common.yaml")["runtime"]
        clock = runtime["streams"]["simulation_clock"]
        self.assertEqual(clock["expected_hz"], 100.0)
        self.assertEqual(clock["stale_after_s"], 0.50)
        monitor_source = (RUNTIME / "monitor.py").read_text(encoding="utf-8")
        self.assertIn('"simulation_clock", "/clock", Clock', monitor_source)
        report_source = (RUNTIME / "report.py").read_text(encoding="utf-8")
        self.assertIn(
            'if name == "simulation_clock"', report_source
        )

    def test_clock_arrival_gap_is_detected_without_a_monitor_tick(self) -> None:
        config = runner.load_config("common.yaml")
        samples = [
            {
                "stream": "simulation_clock",
                "arrival_wall_ns": 1_000_000_000,
                "timestamp_ns": 10_000_000_000,
            },
            {
                "stream": "simulation_clock",
                "arrival_wall_ns": 1_700_000_001,
                "timestamp_ns": 10_010_000_000,
            },
        ]
        with mock.patch.object(report, "_first_tracking_wall_ns", return_value=0):
            summary = report._active_arrival_gap_summary(
                "simulation_clock", config, {}, samples
            )
        self.assertEqual(summary["count"], 1)
        self.assertAlmostEqual(summary["maximum_gap_ms"], 700.000001)
        self.assertEqual(
            report._sim_stream_stale_violation(
                "simulation_clock",
                {
                    "active_wall_arrival_gap_count": summary["count"],
                    "source_stale_event_count": 0,
                },
            ),
            1,
        )

    def test_legacy_rate_row_does_not_invent_partial_gap_schema(self) -> None:
        row = report._rate_row(
            {"streams": {"simulation_clock": {"received": 2}}},
            "simulation_clock",
        )
        self.assertNotIn("arrival_gap_event_count", row)

    def test_legacy_sim_report_uses_raw_clock_arrival_rows(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            (session / "runtime.json").write_text(json.dumps({
                "build_provenance": _valid_captured_provenance(),
                "failures": [],
            }), encoding="utf-8")
            rows = [
                {"kind": "sample", "stream": "simulation_clock",
                 "arrival_wall_ns": 1_000_000_000, "timestamp_ns": 10_000_000_000,
                 "payload": {"stamp_ns": 10_000_000_000}, "accepted_by_monitor": True},
                {"kind": "sample", "stream": "simulation_clock",
                 "arrival_wall_ns": 1_700_000_001, "timestamp_ns": 10_020_000_000,
                 "payload": {"stamp_ns": 10_020_000_000}, "accepted_by_monitor": True},
            ]
            (session / "samples.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
            )
            legacy_clock = {
                "received": 2, "mean_rate_hz": 100.0,
                "minimum_window_rate_hz": 100.0, "maximum_gap_ms": 20.0,
                "stale_event_count": 0, "stale_event_times_ns": [],
            }
            result = report._sim_report(
                session, runner.load_config("sim.yaml"),
                {"streams": {"simulation_clock": legacy_clock}, "diagnostics": {}},
                ROOT, None, "external-mode",
            )
            self.assertEqual(
                result["streams"]["simulation_clock"]["active_wall_arrival_gap_count"], 1
            )
            self.assertEqual(result["verdict"], "FAIL")

    def test_clock_arrival_gap_before_tracking_is_not_an_active_failure(self) -> None:
        config = runner.load_config("common.yaml")
        samples = [
            {
                "stream": "simulation_clock",
                "arrival_wall_ns": 1_000_000_000,
                "timestamp_ns": 10_000_000_000,
            },
            {
                "stream": "simulation_clock",
                "arrival_wall_ns": 1_700_000_001,
                "timestamp_ns": 10_010_000_000,
            },
        ]
        with mock.patch.object(
            report, "_first_tracking_wall_ns", return_value=2_000_000_000
        ):
            summary = report._active_arrival_gap_summary(
                "simulation_clock", config, {}, samples
            )
        self.assertEqual(summary["count"], 0)

    def test_simulation_profiles_separate_headless_and_interactive_manual_control(self) -> None:
        self.assertEqual(
            runner._px4_manual_control_mode(True),
            runner.PX4_HEADLESS_COM_RC_IN_MODE,
        )
        self.assertEqual(
            runner._px4_manual_control_mode(False),
            runner.PX4_INTERACTIVE_COM_RC_IN_MODE,
        )

    def test_replay_and_simulation_preserve_propagation_recovery_headroom(self) -> None:
        for config_name in ("sim.yaml", "dataset.yaml"):
            propagated = runner.load_config(config_name)["fast_lio"]["ros__parameters"]["propagated_odometry"]
            self.assertNotIn("enabled", propagated)
            self.assertEqual(propagated["maximum_correction_age_s"], 0.50)
            self.assertGreater(
                propagated["imu_history_duration_s"] - propagated["maximum_correction_age_s"],
                0,
            )

    def test_estimator_status_flags_rate_matches_px4_diagnostic_publication(self) -> None:
        runtime = runner.load_config("common.yaml")["runtime"]
        status_flags = runtime["streams"]["estimator_status_flags"]
        self.assertEqual(status_flags["expected_hz"], 1.0)
        self.assertEqual(status_flags["stale_after_s"], 2.5)

    def test_external_odometry_bridge_accepts_only_lio_not_simulator_truth(self) -> None:
        source = (ROOT / "src/px4/px4_odometry_bridge/src/px4_external_odometry_bridge_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn('constexpr char kLioPropagatedOdometryTopic[] = "/lio/odometry_propagated"', source)
        self.assertIn(
            "create_subscription<navigation_contracts::msg::PropagatedOdometry>",
            source,
        )
        self.assertIn("kLioPropagatedOdometryTopic", source)
        self.assertNotIn("/sim/ground_truth/odometry", source)

    def test_external_mode_scenario_observes_native_planner_pva(self) -> None:
        source = (ROOT / "tools/runtime/external_mode_scenario.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("PropagatedOdometry", source)
        self.assertIn('"/lio/odometry_propagated"', source)
        self.assertNotIn('create_subscription(Odometry, "/lio/odometry_propagated"', source)
        self.assertIn('"/navigation/navigation_command"', source)
        self.assertNotIn('"/navigation/trajectory_bundle"', source)
        self.assertNotIn('"/navigation/trajectory"', source)
        self.assertNotIn("PlannedTrajectory", source)

    def test_external_mode_speed_contract_keeps_upper_and_attainment_gates_independent(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "external_mode_scenario_speed_contract",
            ROOT / "tools/runtime/external_mode_scenario.py",
        )
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        self.assertEqual(
            module._speed_contract_failures(2.0, 2.0, [2.1], [1.9]), []
        )
        below = module._speed_contract_failures(2.0, 2.0, [2.0], [1.899])
        self.assertTrue(any("did not reach" in reason for reason in below))
        missing = module._speed_contract_failures(2.0, 2.0, [2.0], [])
        self.assertTrue(any("did not reach" in reason for reason in missing))
        over = module._speed_contract_failures(2.0, 2.0, [2.101], [2.0])
        self.assertTrue(any("setpoint exceeded" in reason for reason in over))

    def test_external_mode_scenario_waits_for_registration_before_retrying_nav_state(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "external_mode_scenario",
            ROOT / "tools/runtime/external_mode_scenario.py",
        )
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        scenario = object.__new__(module.ExternalModeScenario)
        scenario.config = {
            "activation_timeout_s": 30.0,
            "post_activation_s": 8.0,
            "disarm_timeout_s": 10.0,
            "exit_nav_state": 4,
            "trajectory_publish_period_s": 0.2,
            "command_retry_period_s": 1.0,
            "trajectory_source": "none",
        }
        scenario.wall_start = time.monotonic() - 2.0
        scenario.sim_start_ns = 1_000_000_000
        scenario.sim_now_ns = 1_000_000_000 + 5_000_000_000
        scenario.finished = False
        scenario.failure = ""
        scenario.trajectory_success_count = 1
        scenario.mode_entered = False
        scenario.external_mode_id = None
        scenario.armed_seen = False
        scenario.last_trajectory_ns = 0
        scenario.last_command_ns = {}
        scenario.VehicleCommand = type("VehicleCommand", (), {"VEHICLE_CMD_SET_NAV_STATE": 100001})
        scenario.command_pub = object()
        scenario._retry = lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("retry while external mode id is unknown"))
        scenario.finish = lambda *args, **kwargs: None

        scenario._tick()
        self.assertEqual(scenario.failure, "")

    def test_external_mode_watchdog_separates_post_completion_drift(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "external_mode_scenario_watchdog",
            ROOT / "tools/runtime/external_mode_scenario.py",
        )
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        scenario = object.__new__(module.ExternalModeScenario)
        scenario.post_takeoff_mode_entered = True
        scenario.latest_odom = {"x": 0.7, "y": 0.0, "z": 0.0,
                                "vx": 0.0, "vy": 0.0, "vz": 0.0}
        scenario.latest_ground_truth = {"x": 0.0, "y": 0.0, "z": 0.0,
                                        "vx": 0.0, "vy": 0.0, "vz": 0.0}
        scenario.latest_odom_stamp_ns = 1_000_000_000
        scenario.latest_ground_truth_stamp_ns = 1_000_000_000
        scenario.lio_gt_origin_offset = (0.0, 0.0, 0.0)
        scenario.mission_complete_observed = True
        scenario.sim_now_ns = 2_000_000_000
        scenario.max_lio_position_residual_m = 0.2
        scenario.max_lio_velocity_residual_m_s = 0.0
        scenario.lio_position_residual_samples = [0.2]
        scenario.post_completion_max_lio_position_residual_m = 0.0
        scenario.post_completion_lio_position_residual_samples = []
        scenario.localization_divergence_started_ns = None
        scenario.localization_divergence_failure = False
        scenario.first_divergence_event = None
        scenario.config = {"expected_outcome": "complete"}
        scenario._record = lambda *args, **kwargs: None
        scenario._record_handover_request = lambda *args, **kwargs: self.fail(
            "post-completion drift must not trigger handover"
        )
        scenario._update_localization_watchdog()
        self.assertEqual(scenario.max_lio_position_residual_m, 0.2)
        self.assertAlmostEqual(scenario.post_completion_max_lio_position_residual_m, 0.7)
        self.assertEqual(len(scenario.lio_position_residual_samples), 1)
        self.assertEqual(len(scenario.post_completion_lio_position_residual_samples), 1)

    def test_external_mode_scenario_planner_source_publishes_bounded_goal(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "external_mode_scenario_goal",
            ROOT / "tools/runtime/external_mode_scenario.py",
        )
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        class Message:
            BEHAVIOR_PASS_THROUGH = 0
            BEHAVIOR_STOP = 1

            def __init__(self) -> None:
                self.header = type("Header", (), {"frame_id": "", "stamp": type("Stamp", (), {})()})()
                self.mission_id = ""
                self.waypoint_index = 0
                self.request_id = 0
                self.target = type("Point", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
                self.acceptance_radius_m = 0.0
                self.behavior = 0
                self.has_next_target = False
                self.next_target = type("Point", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
                self.route = type("Route", (), {
                    "mission_id": "",
                    "frame_id": "",
                    "route_revision": 0,
                    "request_id": 0,
                    "active_waypoint_index": 0,
                    "waypoint_positions": [],
                    "waypoint_ids": [],
                    "waypoint_acceptance_radii_m": [],
                    "waypoint_behaviors": [],
                    "measured_progress_valid": False,
                    "measured_segment_index": 0,
                    "measured_progress_arc_m": 0.0,
                    "measured_projection_arc_m": 0.0,
                    "measured_lateral_error_m": 0.0,
                })()

        class Publisher:
            def __init__(self) -> None:
                self.messages = []

            def publish(self, message: object) -> None:
                self.messages.append(message)

        scenario = object.__new__(module.ExternalModeScenario)
        scenario.config = {"planning_frame": "lio_odom", "goal_offset_m": [1.0, -0.5, 0.25]}
        scenario.latest_odom = {"x": 2.0, "y": 3.0, "z": 4.0}
        scenario.sim_now_ns = 2_500_000_000
        scenario.NavigationGoal = Message
        scenario.Point = type("Point", (), {"__init__": lambda self: None})
        scenario.goal_pub = Publisher()
        scenario.goal_publish_count = 0
        scenario.goal_request_id = 0
        scenario.latest_goal = {}
        scenario.last_goal_ns = -10**18
        scenario._record = lambda *args, **kwargs: None
        scenario.finish = lambda *args, **kwargs: (_ for _ in ()).throw(
            AssertionError("valid planner goal must not finish the scenario")
        )

        scenario._publish_planner_goal()

        self.assertEqual(scenario.goal_publish_count, 1)
        self.assertEqual(len(scenario.goal_pub.messages), 1)
        message = scenario.goal_pub.messages[0]
        self.assertEqual(message.header.frame_id, "lio_odom")
        self.assertEqual((message.target.x, message.target.y, message.target.z),
                         (3.0, 2.5, 4.25))
        self.assertEqual(message.request_id, 1)

    def test_external_mode_scenario_rejects_unknown_mission_keys(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "external_mode_scenario_schema",
            ROOT / "tools/runtime/external_mode_scenario.py",
        )
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with self.assertRaises(ValueError):
            module._reject_unknown_keys({"unexpected": True}, {"mission"}, "root")
        with self.assertRaises(ValueError):
            module._reject_unknown_keys(
                {"id": "m", "unexpected": True}, {"id"}, "mission"
            )

    def test_external_mode_scenario_treats_executor_handover_as_exit(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "external_mode_scenario",
            ROOT / "tools/runtime/external_mode_scenario.py",
        )
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        class DummyVehicleStatus:
            NAVIGATION_STATE_EXTERNAL1 = 23
            NAVIGATION_STATE_AUTO_LOITER = 4
            NAVIGATION_STATE_AUTO_RTL = 20
            ARMING_STATE_ARMED = 2
            ARMING_STATE_DISARMED = 1

        scenario = object.__new__(module.ExternalModeScenario)
        scenario.external_mode_id = 23
        scenario.mode_entered = True
        scenario.mode_exit_observed = False
        scenario.exit_requested = True
        scenario.previous_nav_state = 23
        scenario.events = []
        scenario.VehicleStatus = DummyVehicleStatus
        scenario.latest_status = {}
        scenario.failsafe_seen = False
        scenario.armed_seen = False
        scenario.unexpected_rtl = False
        scenario._record = lambda *args, **kwargs: None
        scenario._status(type(
            "Message",
            (),
            {
                "nav_state": 23,
                "arming_state": 2,
                "can_set_nav_states_mask": 0,
                "failsafe": False,
                "pre_flight_checks_pass": True,
                "executor_in_charge": 0,
                "timestamp": 1,
            },
        )())
        self.assertTrue(scenario.mode_exit_observed)

    def test_external_mode_scenario_records_px4_hold_handover_without_commands(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "external_mode_scenario_handover",
            ROOT / "tools/runtime/external_mode_scenario.py",
        )
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        scenario = object.__new__(module.ExternalModeScenario)
        scenario.execution = "mission"
        scenario.sim_now_ns = 12_000_000_000
        scenario.events = []
        scenario._record = lambda *args, **kwargs: None

        scenario._record_handover_request(
            "unexpected_external_mode_exit", {"nav_state": 18})
        scenario._record_handover_request("second_request")

        self.assertEqual(len(scenario.events), 2)
        self.assertEqual(scenario.events[0]["name"], "px4_hold_handover_requested")
        self.assertEqual(scenario.events[0]["detail"], {"nav_state": 18})

    def test_mission_arms_and_takes_off_before_external_mode_activation(self) -> None:
        source = (ROOT / "tools/runtime/external_mode_scenario.py").read_text(
            encoding="utf-8")
        mission_tick = source.index("if mission_mode:")
        arm = source.index("if self.arm_ack_success_sim_ns is None and not self.manual_takeoff:", mission_tick)
        takeoff = source.index("if not self.takeoff_requested:", arm)
        activate = source.index('if not getattr(self, "mode_active", False)', takeoff)
        self.assertLess(arm, takeoff)
        self.assertLess(takeoff, activate)
        self.assertIn("pre_activation_odometry_stable_s", source[arm:activate])
        self.assertIn("prepare_takeoff_hold", source[takeoff:activate])
        self.assertIn("NAVIGATION_STATE_AUTO_LOITER", source[takeoff:activate])

    def test_initial_takeoff_handover_is_not_terminal_mission_pause(self) -> None:
        source = (ROOT / "tools/runtime/external_mode_scenario.py").read_text(
            encoding="utf-8")
        paused = source.index(
            'if self.mode_status_state == int(self.NavigationModeStatus.PAUSED):')
        self.assertIn("if (self.post_takeoff_mode_entered", source[paused:])

    def test_mission_timeout_precedes_stability_window_return(self) -> None:
        source = (ROOT / "tools/runtime/external_mode_scenario.py").read_text(
            encoding="utf-8")
        timeout = source.index(
            'if mode_elapsed_s > float(self.config.get("mission_timeout_s", 90.0))')
        stability_return = source.index("if stable_elapsed_s < stable_s:")
        self.assertLess(timeout, stability_return)

    def test_manual_takeoff_path_sends_neither_arm_nor_takeoff(self) -> None:
        source = (ROOT / "tools/runtime/external_mode_scenario.py").read_text(
            encoding="utf-8")
        mission_tick = source.index("if mission_mode:")
        arm = source.index("if not self.armed_seen:", mission_tick)
        takeoff = source.index("if not self.takeoff_requested:", arm)
        self.assertIn("if self.manual_takeoff:", source[mission_tick:takeoff])
        self.assertIn("if not self.manual_takeoff:", source[takeoff:takeoff + 300])

    def test_runtime_never_depends_on_nonexistent_ev_aid_source_topics(self) -> None:
        for path in (
            RUNTIME / "monitor.py",
            RUNTIME / "report.py",
            ROOT / "config/runtime/common.yaml",
        ):
            source = path.read_text(encoding="utf-8")
            self.assertNotIn("estimator_aid_src_ev", source)
            self.assertNotIn("EstimatorAidSource", source)

    def test_navigation_replanning_is_correlated_to_goal_and_world_revision(self) -> None:
        source = (
            ROOT / "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("new_goal_", source)
        self.assertIn("ReplanOnce", source)

    def test_candidate_exposure_retains_pre_activation_lease(self) -> None:
        source = (
            ROOT / "src/runtime/navigation_runtime/src/navigation_runtime_node.cpp"
        ).read_text(encoding="utf-8")
        export = source.index("bool NavigationRuntimeNode::commitPlannerCandidate")
        sampling = source.index("const auto candidate_sample", export)
        commit = source.index("const navigation_execution::CommitToken token", sampling)
        boundary = source[sampling:commit]
        self.assertIn("candidate_awaiting_activation", boundary)
        self.assertIn("now_ns < candidate_ptr->valid_from_ns", boundary)
        self.assertIn("!candidate_sample && !candidate_awaiting_activation", boundary)
        self.assertIn("!candidate_awaiting_activation &&", boundary)

    def test_lio_diagnostics_expose_map_guard_and_propagation_latency(self) -> None:
        source = (ROOT / "src/estimation/fast_lio_ros/src/ros_output_publisher.cpp").read_text(
            encoding="utf-8"
        )
        for key in (
            'keyValue("absolute_guard_triggered"',
            'keyValue("absolute_guard_recovery_failed"',
            'keyValue("map_maintenance_us"',
            'keyValue("measurement_model_us"',
            'keyValue("maximum_replay_runtime_us"',
            'keyValue("maximum_imu_batch_size"',
        ):
            self.assertIn(key, source)

    def test_first_sample_does_not_create_stale_event(self) -> None:
        stats = StreamStats("external_odometry", "/fmu/in/vehicle_visual_odometry", stale_after_s=0.1)
        start = 1_000_000_000
        stats.update(start, start)
        stats.check_stale(start + 200_000_000)
        self.assertEqual(stats.as_dict()["stale_event_count"], 0)

    def test_stream_stats_records_rates_and_regressions(self) -> None:
        stats = StreamStats("imu", "/lidar/imu", expected_hz=200.0, stale_after_s=0.1)
        start = 1_000_000_000
        stats.update(start, start, frame_id="livox_imu_frame")
        stats.update(start + 5_000_000, start + 5_000_000, frame_id="livox_imu_frame")
        stats.update(start + 4_000_000, start + 6_000_000, frame_id="livox_imu_frame")
        stats.check_stale(start + 200_000_000)
        snapshot = stats.as_dict()
        self.assertEqual(snapshot["received"], 3)
        self.assertEqual(snapshot["timestamp_regression_count"], 1)
        self.assertEqual(snapshot["stale_event_count"], 1)
        self.assertEqual(snapshot["frame_ids"], ["livox_imu_frame"])

    def test_stream_stats_records_direct_gap_without_stale_timer(self) -> None:
        stats = StreamStats("simulation_clock", "/clock", stale_after_s=0.5)
        stats.update(1_000_000_000, 10_000_000_000)
        stats.update(1_020_000_000, 10_500_000_000)  # exact boundary passes
        stats.update(1_040_000_000, 11_000_000_001)
        snapshot = stats.as_dict()
        self.assertEqual(snapshot["arrival_gap_event_count"], 1)
        self.assertEqual(snapshot["stale_event_count"], 0)
        self.assertAlmostEqual(snapshot["maximum_arrival_gap_ms"], 500.000001)
        event = snapshot["arrival_gap_events"][0]
        self.assertEqual(event["previous_source_stamp_ns"], 1_020_000_000)
        self.assertEqual(event["source_stamp_ns"], 1_040_000_000)

    def test_clock_gap_snapshot_is_authoritative_without_raw_samples(self) -> None:
        row = {
            "arrival_gap_event_count": 1,
            "arrival_gap_event_record_count": 1,
            "arrival_gap_event_overflow_count": 0,
            "arrival_gap_event_times_ns": [2_000_000_000],
            "arrival_gap_events": [{
                "event_wall_ns": 2_000_000_000,
                "previous_arrival_wall_ns": 1_500_000_000,
                "arrival_wall_ns": 2_200_000_000,
                "gap_ms": 700.0,
            }],
            "maximum_arrival_gap_ms": 700.0,
            "stale_event_count": 0,
            "stale_event_times_ns": [],
        }
        result = report._active_arrival_gap_summary(
            "simulation_clock",
            {"runtime": {"streams": {"simulation_clock": {"stale_after_s": 0.5}}}},
            {},
            [],
            row,
        )
        self.assertEqual(result["count"], 1)
        self.assertEqual(result["maximum_gap_ms"], 700.0)
        malformed = dict(row, arrival_gap_event_count=2)
        self.assertFalse(report._active_arrival_gap_summary(
            "simulation_clock",
            {"runtime": {"streams": {"simulation_clock": {"stale_after_s": 0.5}}}},
            {}, [], malformed,
        )["evidence_valid"])

    def test_sim_report_blocks_callback_owned_clock_gap_without_raw_clock_rows(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            valid_provenance = _valid_captured_provenance()
            runtime = {
                "build_provenance": valid_provenance,
                "failures": [],
            }
            (session / "runtime.json").write_text(json.dumps(runtime), encoding="utf-8")
            (session / "samples.jsonl").write_text("", encoding="utf-8")
            clock = StreamStats("simulation_clock", "/clock", expected_hz=100.0, stale_after_s=0.5)
            clock.update(1_000_000_000, 10_000_000_000)
            clock.update(1_020_000_000, 10_700_000_000)
            snapshot = {
                "streams": {"simulation_clock": clock.as_dict()},
                "diagnostics": {},
            }
            result = report._sim_report(
                session, runner.load_config("sim.yaml"), snapshot, ROOT, None,
                "external-mode",
            )
            self.assertEqual(result["verdict"], "FAIL")
            self.assertEqual(
                result["streams"]["simulation_clock"]["active_wall_arrival_gap_count"], 1
            )
            self.assertIn(
                "simulation_clock timestamp/freshness/validity violation",
                result["reasons"],
            )

    def test_clock_gap_is_clipped_against_tracking_window(self) -> None:
        def clock_row(before_ns: int, after_ns: int) -> dict[str, object]:
            return {
                "arrival_gap_event_count": 1,
                "arrival_gap_event_record_count": 1,
                "arrival_gap_event_overflow_count": 0,
                "arrival_gap_event_times_ns": [before_ns + 500_000_000],
                "arrival_gap_events": [{
                    "event_wall_ns": before_ns + 500_000_000,
                    "previous_arrival_wall_ns": before_ns,
                    "arrival_wall_ns": after_ns,
                    "gap_ms": (after_ns - before_ns) / 1e6,
                }],
                "maximum_arrival_gap_ms": (after_ns - before_ns) / 1e6,
                "stale_event_count": 0,
                "stale_event_times_ns": [],
            }

        samples = [{
            "kind": "sample",
            "stream": "diagnostics",
            "arrival_wall_ns": 1_400_000_000,
            "timestamp_ns": 1,
            "accepted_by_monitor": True,
            "payload": {"values": {"state": "TRACKING"}},
        }]
        config = {"runtime": {"streams": {"simulation_clock": {"stale_after_s": 0.5}}}}
        # The callback gap starts before TRACKING but remains absent for 600 ms
        # after TRACKING, so it is an active violation.
        blocked = report._active_arrival_gap_summary(
            "simulation_clock", config, {}, samples,
            clock_row(1_000_000_000, 2_000_000_000)
        )
        self.assertEqual(blocked["count"], 1)
        self.assertEqual(blocked["maximum_gap_ms"], 1000.0)
        # The stale interval ended before TRACKING; startup outage is excluded.
        allowed = report._active_arrival_gap_summary(
            "simulation_clock", config, {}, samples,
            clock_row(500_000_000, 1_300_000_000)
        )
        self.assertEqual(allowed["count"], 0)

    def test_monitor_does_not_serialize_normal_clock_samples(self) -> None:
        runtime_monitor = monitor.RuntimeMonitor.__new__(monitor.RuntimeMonitor)
        runtime_monitor.streams = {
            "simulation_clock": StreamStats("simulation_clock", "/clock", stale_after_s=0.5)
        }
        runtime_monitor.latest = {}
        runtime_monitor.diagnostics = {}
        runtime_monitor._sample_stream = io.StringIO()
        spec = monitor.TopicSpec(
            "simulation_clock", "/clock", object,
            lambda _message: {"stamp_ns": 1_000_000_000},
        )
        runtime_monitor._callback(spec)(SimpleNamespace())
        self.assertEqual(runtime_monitor._sample_stream.getvalue(), "")
        self.assertEqual(runtime_monitor.streams["simulation_clock"].received, 1)

    def test_monitor_decodes_typed_propagated_odometry_envelope(self) -> None:
        stamp = SimpleNamespace(sec=12, nanosec=345)
        nested = SimpleNamespace(
            header=SimpleNamespace(stamp=stamp, frame_id="world"),
            child_frame_id="base_link",
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            ),
            twist=SimpleNamespace(
                twist=SimpleNamespace(
                    linear=SimpleNamespace(x=4.0, y=5.0, z=6.0),
                    angular=SimpleNamespace(x=0.1, y=0.2, z=0.3),
                )
            ),
        )
        message = SimpleNamespace(odometry=nested, localization_epoch=7, sequence=9)
        payload = monitor._propagated_odom_payload(message)
        self.assertEqual(payload["stamp_ns"], 12_000_000_345)
        self.assertEqual(payload["localization_epoch"], 7)
        self.assertEqual(payload["sequence"], 9)
        self.assertEqual(payload["position"], [1.0, 2.0, 3.0])

    def test_clock_interval_history_is_disabled_but_max_and_gap_count_remain_exact(self) -> None:
        stats = StreamStats(
            "simulation_clock", "/clock", stale_after_s=0.5,
            interval_history_enabled=False,
        )
        stats.update(1_000_000_000, 10_000_000_000)
        stats.update(1_020_000_000, 10_700_000_000)
        snapshot = stats.as_dict()
        self.assertIsNone(snapshot["p95_interval_ms"])
        self.assertEqual(snapshot["maximum_gap_ms"], 20.0)
        self.assertEqual(snapshot["arrival_gap_event_count"], 1)
        self.assertEqual(snapshot["arrival_gap_event_overflow_count"], 0)

    def test_report_provenance_is_session_captured_not_rerendered_git(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            captured = _valid_captured_provenance()
            (session / "runtime.json").write_text(
                json.dumps({"build_provenance": captured}), encoding="utf-8"
            )
            with mock.patch.object(report, "_git", return_value="changed-after-flight"):
                self.assertEqual(
                    report._session_provenance(session, ROOT)["manifest_sha256"],
                    captured["manifest_sha256"],
                )

    def test_delayed_stale_timer_owns_threshold_crossing_before_observation_end(self) -> None:
        stats = StreamStats("simulation_clock", "/clock", stale_after_s=0.5)
        stats.update(1_000_000_000, 10_000_000_000)
        stats.update(1_020_000_000, 10_020_000_000)
        stats.check_stale(11_000_000_000)
        self.assertEqual(stats.as_dict()["stale_event_times_ns"], [10_520_000_000])
        active = report._active_stale_times(
            stats.as_dict(),
            {"observation_finished_wall_ns": 10_700_000_000},
            [],
        )
        self.assertEqual(active, [10_520_000_000])

    def test_partial_captured_provenance_fails_closed(self) -> None:
        self.assertEqual(
            report._provenance_reasons({"build_provenance": {"status": "VALID"}}),
            ["runtime did not capture a validated authoritative Release build manifest"],
        )

    def test_captured_provenance_rechecks_artifact_digest(self) -> None:
        captured = _valid_captured_provenance()
        self.assertTrue(report._captured_provenance_valid(captured))
        artifact = captured["manifest"]["artifacts"][0]
        artifact["sha256"] = "0" * 64
        self.assertFalse(report._captured_provenance_valid(captured))

    def test_metric_summary_does_not_overflow_rms(self) -> None:
        summary = report._metric_summary([1.0e308, -1.0e308])
        self.assertTrue(math.isfinite(summary["rmse"]))
        self.assertEqual(summary["rmse"], 1.0e308)

    def test_match_rejects_out_of_order_stream(self) -> None:
        sample = lambda stamp: {"timestamp_ns": stamp, "payload": {}}
        self.assertEqual(
            report._match([sample(2), sample(1)], [sample(1), sample(2)], 0), []
        )

    def test_simulation_stream_discards_wall_epoch_without_regression(self) -> None:
        stats = StreamStats(
            "px4_odometry",
            "/fmu/out/vehicle_odometry",
            stale_after_s=0.1,
            timestamp_upper_bound_ns=1_000_000_000_000_000,
        )
        stats.update(1_787_022_423_986_327_000, 1_000_000_000)
        stats.update(4_000_000_000, 1_100_000_000)
        stats.update(4_016_000_000, 1_116_000_000)
        snapshot = stats.as_dict()
        self.assertEqual(snapshot["received"], 2)
        self.assertEqual(snapshot["timestamp_regression_count"], 0)
        self.assertEqual(snapshot["timestamp_epoch_discard_count"], 1)

    def test_report_ignores_monitor_discarded_samples_for_matching(self) -> None:
        def sample(stream: str, stamp_ns: int, accepted: bool = True) -> dict[str, object]:
            return {
                "kind": "sample",
                "stream": stream,
                "timestamp_ns": stamp_ns,
                "accepted_by_monitor": accepted,
                "payload": {"timestamp_sample_us": stamp_ns // 1000},
            }

        samples = [
            sample("external_odometry", 4_000_000_000),
            sample("px4_odometry", 1_787_022_423_986_327_000, False),
            sample("px4_odometry", 4_000_000_000),
        ]
        self.assertEqual(len(report._series(samples, "px4_odometry")), 1)

    def test_residual_report_declares_missing_pre_fusion_data(self) -> None:
        result = report._residuals([], 20.0)
        self.assertEqual(result["pre_fusion"], "NOT_AVAILABLE")
        self.assertEqual(result["fusion_enabled"], "OBSERVED_ONLY")
        self.assertTrue(result["circular_comparison"])

    def test_residual_report_aligns_lio_frd_to_px4_world(self) -> None:
        yaw_quarter_turn = [0.7071067811865476, 0.0, 0.0, 0.7071067811865476]

        def sample(stream: str, stamp_us: int, payload: dict[str, object]) -> dict[str, object]:
            return {"kind": "sample", "stream": stream, "payload": payload, "timestamp_ns": stamp_us * 1000}

        samples = [
            sample(
                "external_odometry",
                1_000,
                {"timestamp_sample_us": 1_000, "position": [1.0, 2.0, 3.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0], "velocity": [1.0, 0.0, 0.0]},
            ),
            sample(
                "external_odometry",
                1_020,
                {"timestamp_sample_us": 1_020, "position": [2.0, 2.0, 3.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0], "velocity": [1.0, 0.0, 0.0]},
            ),
            sample(
                "px4_odometry",
                1_000,
                {"timestamp_sample_us": 1_000, "position": [10.0, 20.0, 30.0], "q_wxyz": yaw_quarter_turn, "velocity": [0.0, 1.0, 0.0], "velocity_frame": 1},
            ),
            sample(
                "px4_odometry",
                1_020,
                {"timestamp_sample_us": 1_020, "position": [10.0, 21.0, 30.0], "q_wxyz": yaw_quarter_turn, "velocity": [0.0, 1.0, 0.0], "velocity_frame": 1},
            ),
        ]
        result = report._residuals(samples, 20.0)
        self.assertEqual(result["source"], "lio/external_odometry_input vs px4/estimator_odometry")
        self.assertEqual(result["matched_sample_count"], 2)
        self.assertAlmostEqual(result["position"]["maximum"], 0.0, places=9)
        self.assertAlmostEqual(result["velocity"]["maximum"], 0.0, places=9)
        self.assertAlmostEqual(result["attitude"]["maximum"], 0.0, places=9)

    def test_residual_report_keeps_independent_stream_epochs_distinct(self) -> None:
        def sample(stream: str, stamp_us: int) -> dict[str, object]:
            return {
                "kind": "sample",
                "stream": stream,
                "payload": {
                    "timestamp_sample_us": stamp_us,
                    "position": [0.0, 0.0, 0.0],
                    "q_wxyz": [1.0, 0.0, 0.0, 0.0],
                },
                "timestamp_ns": stamp_us * 1000,
            }

        result = report._residuals([sample("external_odometry", 1_000), sample("px4_odometry", 2_000)], 0.5)
        self.assertEqual(result["matched_sample_count"], 0)
        self.assertEqual(result["initial_stream_epoch_offset_ms"], -1.0)

    def test_frame_contract_reports_ned_world_mapping_and_attitude(self) -> None:
        samples = [
            {
                "kind": "sample",
                "stream": "propagated_odometry",
                "payload": {
                    "stamp_ns": 1_000_000,
                    "position": [2.0, 3.0, 4.0],
                    "q_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "linear_velocity": [1.0, 2.0, 3.0],
                    "angular_velocity": [4.0, 5.0, 6.0],
                },
                "timestamp_ns": 1_000_000,
            },
            {
                "kind": "sample",
                "stream": "external_odometry",
                "payload": {
                    "timestamp_sample_us": 1_000,
                    "pose_frame": 1,
                    "velocity_frame": 1,
                    "position": [3.0, 2.0, -4.0],
                    # C_NED_FROM_ENU * C_FRD_FROM_FLU at identity attitude is
                    # a +90 deg yaw in the NED basis.
                    "q_wxyz": [0.7071067811865476, 0.0, 0.0, 0.7071067811865475],
                    "velocity": [2.0, 1.0, -3.0],
                    "angular_velocity": [4.0, -5.0, -6.0],
                },
                "timestamp_ns": 1_000_000,
            },
        ]
        result = report._frame_contract_residuals(samples, 1.0)
        self.assertEqual(result["world_transform"], "x_px4_ned=y_lio_enu; y_px4_ned=x_lio_enu; z_px4_ned=-z_lio_enu")
        self.assertEqual(result["matched_sample_count"], 1)
        self.assertAlmostEqual(result["position"]["maximum"], 0.0, places=9)
        self.assertAlmostEqual(result["velocity"]["maximum"], 0.0, places=9)
        self.assertAlmostEqual(result["angular_velocity"]["maximum"], 0.0, places=9)
        self.assertAlmostEqual(result["attitude"]["maximum"], 0.0, places=9)
        self.assertEqual(result["frame_contract_violation_count"], 0)

    def test_planner_reference_residuals_separate_lio_and_px4_alignment(self) -> None:
        records = [
            {
                "commit_observed_this_cycle": True,
                "execution_stamp_ns": 1_000_000,
                "bundle_id": 1,
                "planning_state_position": [0.0, 0.0, 0.0],
                "candidate_start_position": [0.5, 0.2, -0.3],
                "candidate_start_wall_time_s": 0.001,
            },
            {
                "commit_observed_this_cycle": True,
                "execution_stamp_ns": 2_000_000,
                "bundle_id": 2,
                # LIO/execution is one metre behind PX4 while the candidate
                # follows PX4.  The diagnostic must keep these contracts
                # separate rather than blaming hot-replan timing.
                "planning_state_position": [1.0, 0.0, 0.0],
                "candidate_start_position": [2.0, 0.0, 0.0],
                "candidate_start_wall_time_s": 0.0021,
            },
            {
                "commit_observed_this_cycle": True,
                "execution_stamp_ns": 3_000_000,
                "bundle_id": 3,
                "planning_state_position": [100.0, 0.0, 0.0],
                "candidate_start_position": [101.0, 0.0, 0.0],
                "candidate_start_wall_time_s": 0.0031,
            },
        ]

        def px4(stamp_us: int, enu_position: list[float], reset_counter: int = 0) -> dict[str, object]:
            # PX4 payload is NED; inverse of C_NED_FROM_ENU is the same
            # permutation/sign matrix used by the product conversion.
            return {
                "kind": "sample",
                "stream": "px4_odometry",
                "timestamp_ns": stamp_us * 1000,
                "payload": {
                    "timestamp_sample_us": stamp_us,
                    "position": [enu_position[1], enu_position[0], -enu_position[2]],
                    "pose_frame": 1,
                    "reset_counter": reset_counter,
                },
            }

        result = report._planner_reference_residuals(
            records,
            [
                px4(1_000, [0.5, 0.2, -0.3]),
                px4(2_000, [1.0, 0.0, 0.0]),
                px4(2_100, [2.0, 0.0, 0.0]),
                px4(3_000, [100.0, 0.0, 0.0], reset_counter=1),
                px4(3_100, [101.0, 0.0, 0.0], reset_counter=1),
            ],
            1.0,
        )
        self.assertTrue(result["available"])
        self.assertEqual(result["execution_matched_commit_count"], 3)
        self.assertEqual(result["candidate_matched_commit_count"], 3)
        self.assertEqual(result["execution_reset_segment_count"], 1)
        self.assertEqual(result["candidate_reset_segment_count"], 1)
        self.assertEqual(result["invalid_pose_frame_count"], 0)
        self.assertAlmostEqual(result["candidate_vs_px4"]["norm"]["maximum"], 0.0, places=9)
        self.assertGreater(result["execution_vs_px4"]["norm"]["maximum"], 0.5)
        self.assertAlmostEqual(result["candidate_vs_execution"]["norm"]["maximum"], 1.0, places=9)

    def test_planner_reference_residuals_is_fail_closed_as_diagnostic_when_unmatched(self) -> None:
        result = report._planner_reference_residuals(
            [{
                "commit_observed_this_cycle": True,
                "execution_stamp_ns": 1_000_000,
                "planning_state_position": [0.0, 0.0, 0.0],
                "candidate_start_position": [0.0, 0.0, 0.0],
                "candidate_start_wall_time_s": None,
            }],
            [],
            1.0,
        )
        self.assertFalse(result["available"])
        self.assertEqual(result["eligible_commit_count"], 1)
        self.assertEqual(result["execution_matched_commit_count"], 0)
        self.assertEqual(result["candidate_matched_commit_count"], 0)

    def test_planner_reference_residuals_rejects_wrong_px4_pose_frame(self) -> None:
        result = report._planner_reference_residuals(
            [{
                "commit_observed_this_cycle": True,
                "execution_stamp_ns": 1_000_000,
                "planning_state_position": [0.0, 0.0, 0.0],
                "candidate_start_position": [0.0, 0.0, 0.0],
                "candidate_start_wall_time_s": 0.001,
            }],
            [{
                "kind": "sample",
                "stream": "px4_odometry",
                "timestamp_ns": 1_000_000,
                "payload": {
                    "timestamp_sample_us": 1_000,
                    "pose_frame": 2,
                    "position": [0.0, 0.0, 0.0],
                },
            }],
            1.0,
        )
        self.assertFalse(result["available"])
        self.assertEqual(result["invalid_pose_frame_count"], 1)

    def test_planner_reference_residuals_tracks_candidate_timestamp_coverage(self) -> None:
        result = report._planner_reference_residuals(
            [{
                "commit_observed_this_cycle": True,
                "execution_stamp_ns": 1_000_000,
                "planning_state_position": [0.0, 0.0, 0.0],
                "candidate_start_position": [0.0, 0.0, 0.0],
                # Missing candidate time must not silently reuse execution time.
                "candidate_start_wall_time_s": None,
            }],
            [{
                "kind": "sample",
                "stream": "px4_odometry",
                "timestamp_ns": 1_000_000,
                "payload": {
                    "timestamp_sample_us": 1_000,
                    "pose_frame": 1,
                    "position": [0.0, 0.0, 0.0],
                },
            }],
            1.0,
        )
        self.assertTrue(result["available"])
        self.assertEqual(result["execution_matched_commit_count"], 1)
        self.assertEqual(result["candidate_matched_commit_count"], 0)

    def test_report_verdict_set_is_closed(self) -> None:
        self.assertEqual(report.VERDICTS, {"PASS", "FAIL", "BLOCKED", "NOT_RUN", "OBSERVATION_COMPLETE"})

    def test_replay_tail_stale_events_are_ignored(self) -> None:
        row = {"stale_event_count": 3, "stale_event_times_ns": [700_000_000, 950_000_000, 1_100_000_000]}
        runtime = {"replay_finished_wall_ns": 1_000_000_000, "replay_tail_grace_s": 0.1}
        self.assertEqual(report._active_stale_count(row, runtime), 1)

    def test_runtime_stale_accounting_ends_at_observation_boundary(self) -> None:
        row = {
            "stale_event_count": 3,
            "stale_event_times_ns": [700_000_000, 950_000_000, 1_010_000_000],
        }
        runtime = {
            "observation_finished_wall_ns": 1_000_000_000,
            "observation_tail_grace_s": 0.1,
        }
        self.assertEqual(report._active_stale_count(row, runtime), 2)

    def test_startup_stale_event_before_tracking_is_not_runtime_violation(self) -> None:
        row = {"stale_event_count": 2, "stale_event_times_ns": [100, 300]}
        samples = [
            {
                "kind": "sample",
                "stream": "diagnostics",
                "arrival_wall_ns": 200,
                "payload": {"values": {"state": "TRACKING"}},
            }
        ]
        self.assertEqual(report._active_stale_count(row, {}, samples), 1)

    def test_callback_stall_with_continuous_source_timestamps_is_not_source_stale(self) -> None:
        row = {"stale_event_count": 1, "stale_event_times_ns": [250_000_000]}
        samples = [
            {"stream": "imu", "arrival_wall_ns": 100_000_000, "timestamp_ns": 1_000_000_000},
            {"stream": "imu", "arrival_wall_ns": 300_000_000, "timestamp_ns": 1_008_000_000},
        ]
        config = {"runtime": {"streams": {"imu": {"stale_after_s": 0.1}}}}
        result = report._stale_classification("imu", row, config, {}, samples)
        self.assertEqual(result["active_callback_stall_count"], 1)
        self.assertEqual(result["observer_dispatch_stall_count"], 1)
        self.assertEqual(result["source_stale_event_count"], 0)
        self.assertEqual(result["maximum_observer_dispatch_source_gap_ms"], 8.0)

    def test_callback_stall_with_a_source_timestamp_gap_fails_closed(self) -> None:
        row = {"stale_event_count": 1, "stale_event_times_ns": [250_000_000]}
        samples = [
            {"stream": "external_odometry", "arrival_wall_ns": 100_000_000, "timestamp_ns": 1_000_000_000},
            {"stream": "external_odometry", "arrival_wall_ns": 300_000_000, "timestamp_ns": 1_300_000_000},
        ]
        config = {"runtime": {"streams": {"external_odometry": {"stale_after_s": 0.2}}}}
        result = report._stale_classification("external_odometry", row, config, {}, samples)
        self.assertEqual(result["active_callback_stall_count"], 1)
        self.assertEqual(result["observer_dispatch_stall_count"], 0)
        self.assertEqual(result["source_stale_event_count"], 1)

    def test_map_maintenance_summary_keeps_guard_evidence(self) -> None:
        samples = [
            {
                "stream": "diagnostics",
                "payload": {
                    "statuses": [
                        {
                            "name": "fast_lio/estimator",
                            "values": {
                                "absolute_guard_triggered": True,
                                "absolute_guard_recovery_failed": False,
                                "map_maintenance_us": 21606,
                            },
                        }
                    ]
                },
            }
        ]
        result = report._map_maintenance_summary(samples)
        self.assertEqual(result["absolute_guard_trigger_count"], 1)
        self.assertEqual(result["absolute_guard_recovery_failure_count"], 0)
        self.assertEqual(result["maximum_maintenance_us"], 21606.0)

    def test_mapping_and_planning_timing_reports_have_required_percentiles(self) -> None:
        samples = [
            {
                "stream": "diagnostics",
                "payload": {
                    "statuses": [
                        {
                            "name": "navigation_mapping/world_model",
                            "values": {
                                "ros_pointcloud_decode_us": value,
                                "mapping_filter_us": value + 1,
                                "planning_total_us": value + 2,
                            },
                        },
                        {
                            "name": "navigation_runtime/planner",
                            "values": {
                                "exp_frontend_us": value + 3,
                                "exp_opt_us": value + 4,
                                "backup_frontend_us": value + 5,
                                "backup_opt_us": value + 6,
                                "planning_total_us": value + 2,
                            },
                        },
                    ]
                },
            }
            for value in (10, 20, 30)
        ]
        mapping = report._diagnostic_timing_summary(
            samples,
            "navigation_mapping/world_model",
            ("ros_pointcloud_decode_us", "mapping_filter_us"),
        )
        self.assertEqual(mapping["ros_pointcloud_decode_us"]["sample_count"], 3)
        self.assertEqual(mapping["ros_pointcloud_decode_us"]["p50"], 20.0)
        for key in ("mean", "p50", "p95", "p99", "max"):
            self.assertIn(key, mapping["mapping_filter_us"])
        planning = report._planning_timing_summary(samples)
        self.assertEqual(planning["planning_total_us"]["sample_count"], 3)
        self.assertEqual(planning["planning_total_us"]["p50"], 22.0)
        self.assertEqual(planning["exp_opt_us"]["sample_count"], 3)
        self.assertEqual(planning["exp_opt_us"]["p50"], 24.0)
        self.assertEqual(planning["backup_opt_us"]["max"], 36.0)

        stale = {
            "stream": "diagnostics",
            "payload": {"statuses": [{
                "name": "navigation_runtime/planner",
                "values": {
                    "exp_diagnostics_valid": 0,
                    "exp_frontend_us": 900,
                    "exp_opt_us": 901,
                    "backup_frontend_us": 902,
                    "backup_opt_us": 903,
                },
            }]},
        }
        filtered = report._planning_timing_summary(samples + [stale])
        self.assertEqual(filtered["exp_opt_us"]["sample_count"], 3)
        self.assertEqual(filtered["exp_opt_us"]["max"], 34.0)
        self.assertEqual(filtered["backup_opt_us"]["max"], 36.0)

    def test_navigation_mapping_summary_preserves_snapshot_identity_and_cost(self) -> None:
        values = {
            "world_generation": "3",
            "world_revision": "17",
            "world_snapshot_bytes": "4019364",
            "world_snapshot_owned_bytes": "3236160",
            "world_snapshot_shared_metadata_bytes": "783204",
            "world_snapshot_live_count": "1",
            "world_snapshot_peak_live_count": "2",
            "world_snapshot_full_export_count": "4",
            "world_snapshot_patch_export_count": "13",
            "world_snapshot_export_mode": "2",
            "world_snapshot_full_export_reason": "0",
            "world_snapshot_export_base_cells": "2048",
            "world_snapshot_export_inflated_cells": "256",
            "world_snapshot_patch_depth": "3",
            "world_snapshot_export_us": "15018",
        }
        snapshot = {
            "streams": {"mapping_diagnostics": {"received": 1, "mean_rate_hz": 10.0}},
            "latest": {"mapping_diagnostics": {"statuses": [{
                "name": "navigation_runtime/planner",
                "level": 0,
                "message": "MAP_READY",
                "values": values,
            }]}},
        }
        samples = [{"stream": "diagnostics", "payload": {"statuses": [{
            "name": "navigation_runtime/planner", "values": values,
        }]}}]
        mapping = report._navigation_mapping_summary(snapshot, samples)
        self.assertEqual(mapping["world_generation"], 3)
        self.assertEqual(mapping["world_revision"], 17)
        self.assertEqual(mapping["world_snapshot_bytes"], 4019364)
        self.assertEqual(mapping["world_snapshot_owned_bytes"], 3236160)
        self.assertEqual(mapping["world_snapshot_peak_live_count"], 2)
        self.assertEqual(mapping["world_snapshot_full_export_count"], 4)
        self.assertEqual(mapping["world_snapshot_patch_export_count"], 13)
        self.assertEqual(mapping["world_snapshot_export_mode"], 2)
        self.assertEqual(mapping["world_snapshot_patch_depth"], 3)
        self.assertEqual(
            mapping["timing_distributions"]["world_snapshot_export_us"]["p50"], 15018.0)

    def test_navigation_mapping_summary_accepts_deferred_snapshot_exports(self) -> None:
        mapping = {
            "received_observation_count": 3,
            "accepted_observation_count": 3,
            "observation_rejected_before_inbox_count": 0,
            "mapping_started_count": 3,
            "mapping_published_count": 3,
            "mapping_failed_count": 0,
            "mapping_pending_count": 0,
            "mapping_in_flight_count": 0,
            "observation_replaced_pending_count": 0,
            "observation_discarded_pending_count": 0,
            "observation_accounting_valid": 1,
            "observation_accounting_violation_count": 0,
            **_mapping_outcomes(3),
            "world_revision": 3,
            "world_snapshot_published_count": 2,
            "world_snapshot_deferred_count": 1,
        }
        self.assertEqual(report._mapping_integrity_reasons(mapping), [])

    def test_mapping_snapshot_export_timing_ignores_deferred_zero_samples(self) -> None:
        samples = [
            {
                "stream": "mapping_diagnostics",
                "payload": {"statuses": [{
                    "name": "navigation_mapping/world_model",
                    "values": {
                        "world_snapshot_published": published,
                        "world_snapshot_export_us": export_us,
                    },
                }]},
            }
            for published, export_us in ((1, 12000), (0, 0), (1, 14000), (1, 16000))
        ]
        timing = report._diagnostic_timing_summary(
            samples,
            "navigation_mapping/world_model",
            ("world_snapshot_export_us",),
            stream_names=("mapping_diagnostics",),
        )
        self.assertEqual(timing["world_snapshot_export_us"]["sample_count"], 3)
        self.assertEqual(timing["world_snapshot_export_us"]["p50"], 14000.0)

    def test_navigation_mapping_summary_merges_status_owners_in_either_final_order(self) -> None:
        planner = {
            "name": "navigation_runtime/planner",
            "level": 0,
            "message": "MAP_READY",
            "values": {
                "received_observation_count": "2",
                "observation_rejected_before_inbox_count": "0",
                "accepted_observation_count": "2",
                "dropped_cloud_count": "0",
                "observation_replaced_pending_count": "0",
                "observation_discarded_pending_count": "0",
                "mapping_started_count": "2",
                "mapping_published_count": "2",
                "mapping_failed_count": "0",
                "mapping_pending_count": "0",
                "mapping_in_flight_count": "0",
                "observation_accounting_valid": "1",
            },
        }
        world = {
            "name": "navigation_mapping/world_model",
            "level": 0,
            "message": "PUBLISHED",
            "values": {
                "world_revision": "2",
                "mapping_started_count": "2",
                "mapping_published_count": "2",
                "mapping_failed_count": "0",
                "mapping_pending_count": "0",
                "mapping_in_flight_count": "0",
                "observation_accounting_valid": "1",
                "observation_replaced_pending_count": "0",
                "observation_discarded_pending_count": "0",
                **{key: str(value) for key, value in _mapping_outcomes(2).items()},
            },
        }
        for final, earlier in ((world, planner), (planner, world)):
            snapshot = {
                "streams": {"mapping_diagnostics": {"received": 2}},
                "latest": {"mapping_diagnostics": {"statuses": [final]}},
            }
            samples = [
                {"stream": "diagnostics", "payload": {"statuses": [earlier]}},
                {"stream": "diagnostics", "payload": {"statuses": [final]}},
            ]
            mapping = report._navigation_mapping_summary(snapshot, samples)
            self.assertEqual(mapping["received_observation_count"], 2)
            self.assertEqual(mapping["accepted_observation_count"], 2)
            self.assertEqual(mapping["world_revision"], 2)
            self.assertEqual(mapping["mapping_published_count"], 2)
            self.assertEqual(mapping["observation_stamp_ns"], 1_000_000_002)
            self.assertEqual(report._mapping_integrity_reasons(mapping), [])

    def test_navigation_mapping_summary_uses_coherent_world_lifecycle_snapshot(self) -> None:
        planner = {
            "name": "navigation_runtime/planner",
            "values": {
                "received_observation_count": "304",
                "accepted_observation_count": "304",
            },
        }
        world = {
            "name": "navigation_mapping/world_model",
            "level": 0,
            "message": "PUBLISHED",
            "values": {
                "received_observation_count": "305",
                "observation_rejected_before_inbox_count": "0",
                "accepted_observation_count": "305",
                "world_revision": "305",
                "mapping_started_count": "305",
                "mapping_published_count": "305",
                "mapping_failed_count": "0",
                "mapping_pending_count": "0",
                "mapping_in_flight_count": "0",
                "observation_accounting_valid": "1",
                "observation_replaced_pending_count": "0",
                "observation_discarded_pending_count": "0",
                **{key: str(value) for key, value in _mapping_outcomes(305).items()},
            },
        }
        snapshot = {
            "streams": {"mapping_diagnostics": {"received": 2}},
            "latest": {"mapping_diagnostics": {"statuses": [world]}},
        }
        samples = [
            {"stream": "diagnostics", "payload": {"statuses": [planner]}},
            {"stream": "mapping_diagnostics", "payload": {"statuses": [world]}},
        ]
        mapping = report._navigation_mapping_summary(snapshot, samples)
        self.assertEqual(mapping["received_observation_count"], 305)
        self.assertEqual(mapping["accepted_observation_count"], 305)
        self.assertEqual(mapping["mapping_published_count"], 305)
        self.assertEqual(report._mapping_integrity_reasons(mapping), [])

        world["values"]["received_observation_count"] = "304"
        world["values"]["accepted_observation_count"] = "304"
        inconsistent = report._navigation_mapping_summary(snapshot, samples)
        self.assertNotEqual(report._mapping_integrity_reasons(inconsistent), [])

    def test_navigation_mapping_summary_ignores_decision_trace_for_lifecycle(self) -> None:
        planner_lifecycle = {
            "name": "navigation_runtime/planner",
            "values": {
                "received_observation_count": "186",
                "observation_rejected_before_inbox_count": "0",
                "accepted_observation_count": "186",
                "observation_replaced_pending_count": "9",
                "observation_discarded_pending_count": "0",
                "mapping_started_count": "186",
                "mapping_published_count": "186",
                "mapping_failed_count": "0",
                "mapping_pending_count": "0",
                "mapping_in_flight_count": "0",
                "observation_accounting_valid": "1",
            },
        }
        decision_trace = {
            "name": "navigation_runtime/planner",
            "message": "DECISION_TRACE",
            "values": {"trajectory_generation": "1"},
        }
        world_lifecycle = {
            "name": "navigation_mapping/world_model",
            "level": 0,
            "message": "PUBLISHED_UPDATED",
            "values": {
                "received_observation_count": "195",
                "observation_rejected_before_inbox_count": "0",
                "accepted_observation_count": "195",
                "observation_replaced_pending_count": "9",
                "observation_discarded_pending_count": "0",
                "dropped_cloud_count": "9",
                "mapping_started_count": "186",
                "mapping_published_count": "186",
                "mapping_failed_count": "0",
                "mapping_pending_count": "0",
                "mapping_in_flight_count": "0",
                "observation_accounting_valid": "1",
                **{key: str(value) for key, value in _mapping_outcomes(186).items()},
                "world_revision": "186",
            },
        }
        snapshot = {
            "streams": {"mapping_diagnostics": {"received": 3}},
            "latest": {"mapping_diagnostics": {"statuses": [world_lifecycle]}},
        }
        samples = [
            {"stream": "diagnostics", "arrival_wall_ns": 100,
             "payload": {"statuses": [planner_lifecycle]}},
            {"stream": "diagnostics", "arrival_wall_ns": 200,
             "payload": {"statuses": [decision_trace]}},
            {"stream": "mapping_diagnostics", "arrival_wall_ns": 300,
             "payload": {"statuses": [world_lifecycle]}},
        ]
        mapping = report._navigation_mapping_summary(snapshot, samples)
        reasons = report._mapping_integrity_reasons(mapping)
        self.assertIn("mapping replaced an unconsumed cloud: 9", reasons)
        self.assertNotIn(
            "mapping accepted-observation conservation equation failed", reasons
        )
        self.assertNotIn(
            "mapping observation lifecycle evidence incomplete; "
            "terminal conservation not evaluated",
            reasons,
        )

    def test_mapping_input_conservation_allows_rejected_before_inbox(self) -> None:
        reasons = report._mapping_integrity_reasons({
            "received_observation_count": 10,
            "observation_rejected_before_inbox_count": 2,
            "accepted_observation_count": 8,
            "observation_replaced_pending_count": 0,
            "observation_discarded_pending_count": 0,
            "mapping_published_count": 8,
            "mapping_failed_count": 0,
            "mapping_pending_count": 0,
            "mapping_in_flight_count": 0,
            "observation_accounting_valid": 1,
            **_mapping_outcomes(8),
            "world_revision": 8,
        })
        self.assertEqual(reasons, [])

    def test_navigation_mapping_summary_uses_newer_planner_lifecycle_event(self) -> None:
        """A later discard event must not be merged with an older PUBLISHED event."""
        world = {
            "name": "navigation_mapping/world_model",
            "level": 0,
            "message": "PUBLISHED",
            "values": {
                "received_observation_count": "248",
                "accepted_observation_count": "248",
                "world_revision": "174",
                "mapping_started_count": "174",
                "mapping_published_count": "174",
                "mapping_failed_count": "0",
                "mapping_pending_count": "0",
                "mapping_in_flight_count": "0",
                "observation_accounting_valid": "1",
            },
        }
        planner = {
            "name": "navigation_runtime/planner",
            "level": 0,
            "message": "MAP_READY",
            "values": {
                "received_observation_count": "251",
                "accepted_observation_count": "251",
                "dropped_cloud_count": "2",
                "observation_replaced_pending_count": "1",
                "observation_replaced_ready_count": "1",
                "observation_discarded_pending_count": "76",
                "observation_discarded_ready_count": "1",
                "mapping_started_count": "174",
                "mapping_published_count": "174",
                "mapping_failed_count": "0",
                "mapping_pending_count": "0",
                "mapping_in_flight_count": "0",
                "observation_accounting_valid": "1",
            },
        }
        snapshot = {
            "streams": {"mapping_diagnostics": {"received": 2}},
            "latest": {"mapping_diagnostics": {"statuses": [planner]}},
        }
        samples = [
            {"stream": "mapping_diagnostics", "arrival_wall_ns": 100, "payload": {"statuses": [world]}},
            {"stream": "diagnostics", "arrival_wall_ns": 200, "payload": {"statuses": [planner]}},
        ]
        mapping = report._navigation_mapping_summary(snapshot, samples)
        self.assertEqual(mapping["received_observation_count"], 251)
        self.assertEqual(mapping["mapping_published_count"], 174)
        reasons = report._mapping_integrity_reasons(mapping)
        self.assertNotIn("mapping accepted-observation conservation equation failed", reasons)
        self.assertNotIn(
            "mapping replacement compatibility counter mismatch", " ".join(reasons)
        )

    def test_navigation_mapping_summary_missing_final_sample_stays_conservative(self) -> None:
        """Do not synthesize lifecycle conservation from snapshot.latest alone."""
        planner = {
            "name": "navigation_runtime/planner",
            "level": 0,
            "message": "MAP_READY",
            "values": {
                "received_observation_count": "304",
                "accepted_observation_count": "304",
            },
        }
        final_world = {
            "name": "navigation_mapping/world_model",
            "level": 0,
            "message": "PUBLISHED",
            "values": {
                "received_observation_count": "305",
                "accepted_observation_count": "305",
                "world_revision": "305",
                "mapping_started_count": "305",
                "mapping_published_count": "305",
                "mapping_failed_count": "0",
                "mapping_pending_count": "0",
                "mapping_in_flight_count": "0",
                "observation_accounting_valid": "1",
            },
        }
        snapshot = {
            "streams": {"mapping_diagnostics": {"received": 1}},
            "latest": {"mapping_diagnostics": {"statuses": [final_world]}},
        }
        samples = [
            {
                "stream": "diagnostics",
                "arrival_wall_ns": 100,
                "payload": {"statuses": [planner]},
            }
        ]
        mapping = report._navigation_mapping_summary(snapshot, samples)

        self.assertEqual(mapping["received_observation_count"], 304)
        self.assertEqual(mapping["accepted_observation_count"], 304)
        self.assertEqual(mapping["mapping_published_count"], 305)
        reasons = report._mapping_integrity_reasons(mapping)
        self.assertIn(
            "mapping observation lifecycle evidence incomplete; "
            "terminal conservation not evaluated",
            reasons,
        )

    def test_planning_execution_summary_exposes_replan_skips_and_fallbacks(self) -> None:
        snapshot = {
            "latest": {
                "planning_diagnostics": {
                    "statuses": [
                        {
                            "name": "navigation_planning/planner",
                            "values": {
                                "plan_count": "10",
                                "plan_skip_count": "4",
                                "success_count": "9",
                                "failure_count": "1",
                                "safety_fallback_count": "2",
                                "safety_route_selected_count": "1",
                                "safety_stop_selected_count": "1",
                                "nominal_plan_count": "3",
                                "nominal_selected_count": "1",
                                "dual_verification_failure_count": "0",
                                "verification_failure_count": "0",
                                "local_subgoal_selected_count": "0",
                                "local_subgoal_failure_count": "0",
                                "trajectory_revalidation_count": "6",
                                "trajectory_revalidation_failure_count": "1",
                                "trajectory_reuse_count": "5",
                                "full_replan_count": "2",
                            },
                        }
                    ]
                }
            }
        }
        self.assertEqual(
            report._planning_execution_summary(snapshot),
            {
                "plan_count": 10,
                "plan_skip_count": 4,
                "success_count": 9,
                "failure_count": 1,
                "safety_fallback_count": 2,
                "safety_route_selected_count": 1,
                "safety_stop_selected_count": 1,
                "nominal_plan_count": 3,
                "nominal_selected_count": 1,
                "dual_verification_failure_count": 0,
                "verification_failure_count": 0,
                "local_subgoal_selected_count": 0,
                "local_subgoal_failure_count": 0,
                "trajectory_revalidation_count": 6,
                "trajectory_revalidation_failure_count": 1,
                "trajectory_reuse_count": 5,
                "full_replan_count": 2,
            },
        )


if __name__ == "__main__":
    unittest.main()
