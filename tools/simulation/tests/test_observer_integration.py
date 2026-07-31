import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path


class ObserverIntegrationTest(unittest.TestCase):
    def test_observer_writes_all_metric_files_and_stops_cleanly(self):
        simulation = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            for name in ("metrics", "snapshots", "logs", "latest", "pids"):
                (session/name).mkdir()
            process = subprocess.Popen([
                sys.executable, str(simulation/"sim_observer.py"),
                "--session", str(session),
                "--config", str(simulation/"config/px4_mid360_observer.yaml"),
                "--sample-hz", "4",
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                time.sleep(2.5)
                process.send_signal(signal.SIGTERM)
                stdout, stderr = process.communicate(timeout=10)
            finally:
                if process.poll() is None:
                    process.kill()
            self.assertEqual(process.returncode, 0, stderr)
            for name in ("streams.csv", "pointcloud.csv", "process.csv",
                         "gazebo.csv", "synchronization.csv", "events.jsonl"):
                self.assertTrue((session/"metrics"/name).exists(), name)
            self.assertTrue((session/"latest/observer_state.json").exists())


if __name__ == "__main__":
    unittest.main()
