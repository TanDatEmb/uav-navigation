"""tools/uavnav/sitl_hover.sh holds log/sitl/.lock (flock, non-blocking) for the whole run: a second harness started
meanwhile is refused with exit 2 before any precheck, in addition to the process guard (S1b Task 11 review M4)."""
import fcntl
import pathlib
import subprocess
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[3]
HARNESS = ROOT / "tools" / "uavnav" / "sitl_hover.sh"
LOCK = ROOT / "log" / "sitl" / ".lock"


class SitlHarnessLockTest(unittest.TestCase):
    def test_a_second_harness_is_refused_while_the_lock_is_held(self):
        LOCK.parent.mkdir(parents=True, exist_ok=True)
        with open(LOCK, "a", encoding="utf-8") as held:
            try:
                fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                self.skipTest("a SITL harness is running and holds log/sitl/.lock")
            for mode in ("--hover", "--probe"):
                result = subprocess.run(["bash", str(HARNESS), mode, "--profile", "gps", "--dry-run"],
                                        capture_output=True, text=True, timeout=30, check=False)
                self.assertEqual(result.returncode, 2, (mode, result.stderr))
                self.assertIn("REFUSED: another sitl_hover.sh holds log/sitl/.lock", result.stderr)


if __name__ == "__main__":
    unittest.main()
