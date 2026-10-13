"""Tests for tools.uavnav.sitl_guard (the SITL start guard's process matching). Synthetic processes only."""
import os
import pathlib
import shutil
import tempfile
import unittest

from tools.uavnav import sitl_guard


def write_proc(root, pid, ppid, argv, comm="x"):
    d = pathlib.Path(root) / str(pid)
    d.mkdir()
    (d / "cmdline").write_bytes(b"\0".join(a.encode() for a in argv) + b"\0")
    (d / "stat").write_text(f"{pid} ({comm}) S {ppid} {pid} {pid} 0 -1\n")


class SitlGuardTest(unittest.TestCase):
    def test_words_inside_a_shell_string_do_not_match(self):
        for argv in (["/bin/bash", "-c", "make sitl-probe; colcon build --packages-up-to x; px4 MicroXRCEAgent"],
                     ["bash", "-c", "pgrep -af 'colcon (build|test)'"],
                     ["/usr/bin/vim", "tools/simulation/px4"],
                     ["grep", "-r", "gz sim", "."],
                     ["/usr/bin/python3", "tools/uavnav/sitl_checks.py", "--probe", "x"],
                     ["/usr/bin/python3", "/usr/bin/colcon", "list"],
                     ["/usr/bin/gz", "topic", "-l"]):
            self.assertIsNone(sitl_guard.classify(argv), argv)

    def test_px4_matches_by_binary_name(self):
        for argv in (["/home/u/Dev/Autopilot/build/px4_sitl_default/bin/px4", "-d", "-s", "x.sh"],
                     ["./bin/px4"], ["px4", "-w", "/tmp/w"], ["/opt/renamed/dir/px4"]):
            self.assertEqual(sitl_guard.classify(argv), "sitl", argv)

    def test_other_sitl_processes_match(self):
        for argv in (["/usr/local/bin/MicroXRCEAgent", "udp4", "-p", "8888"],
                     ["MicroXRCEAgent", "udp4", "-p", "9999"],
                     ["/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge", "--ros-args", "-p", "config_file:=x"],
                     ["/ws/install/uav_simulation/lib/uav_simulation/gz_lidar_bridge"],
                     ["gz", "sim", "-r", "-s", "w.sdf"],
                     ["ruby", "/usr/bin/gz", "sim", "-r", "-s", "w.sdf"],
                     ["ruby3.2", "/usr/bin/gz", "sim", "-g"],
                     ["/usr/bin/python3", "tools/uavnav/sitl_probe.py", "--run-dir", "d"],
                     ["/usr/bin/python3", "/ws/tools/uavnav/sitl_hover.py", "--run-dir", "d"],
                     ["/ws/install/uavnav_lio/lib/uavnav_lio/lio_node", "--ros-args", "-r", "__node:=lio"],
                     ["/ws/install/uavnav_px4_bridge/lib/uavnav_px4_bridge/px4_bridge_node", "--ros-args"]):
            self.assertEqual(sitl_guard.classify(argv), "sitl", argv)

    def test_colcon_build_and_test_match(self):
        for argv in (["/usr/bin/python3", "/usr/bin/colcon", "build", "--packages-up-to", "x"],
                     ["python3.12", "/usr/local/bin/colcon", "test"],
                     ["colcon", "build"]):
            self.assertEqual(sitl_guard.classify(argv), "colcon", argv)

    def test_find_excludes_the_ancestor_chain_and_reads_proc(self):
        root = tempfile.mkdtemp(prefix="sitl_guard_proc_")
        self.addCleanup(shutil.rmtree, root, True)
        write_proc(root, 1, 0, ["/sbin/init"])
        # Our ancestry: a controller shell whose argv merely names the tools, then make, then the guard.
        write_proc(root, 100, 1, ["/bin/bash", "-c", "colcon build; px4"])
        write_proc(root, 101, 100, ["colcon", "build"])                 # an ancestor that really matches
        write_proc(root, 102, 101, ["make", "sitl-probe", "PROFILE=gps"])
        write_proc(root, 103, 102, ["/usr/bin/python3", "tools/uavnav/sitl_guard.py"])
        # Unrelated processes elsewhere in the tree.
        write_proc(root, 200, 1, ["/home/u/Autopilot/build/px4_sitl_default/bin/px4", "-d"])
        write_proc(root, 201, 1, ["/bin/bash", "-c", "MicroXRCEAgent udp4 -p 8888"])
        write_proc(root, 202, 1, ["/usr/bin/python3", "/usr/bin/colcon", "test"])
        (pathlib.Path(root) / "self").mkdir()  # non-numeric entries are skipped
        processes = sitl_guard.read_processes(root)
        self.assertEqual(processes[103], (102, ["/usr/bin/python3", "tools/uavnav/sitl_guard.py"]))
        self.assertEqual(sitl_guard.ancestors(processes, 103), {103, 102, 101, 100, 1})
        found = sitl_guard.find(processes, 103)
        self.assertEqual([(c, p) for c, p, _ in found], [("sitl", 200), ("colcon", 202)])

    def test_cli_runs_on_this_host(self):
        # Must not report itself or the test runner; whatever else runs here is the host's business.
        found = sitl_guard.find(sitl_guard.read_processes(), os.getpid())
        self.assertNotIn(os.getpid(), [pid for _, pid, _ in found])


if __name__ == "__main__":
    unittest.main()
