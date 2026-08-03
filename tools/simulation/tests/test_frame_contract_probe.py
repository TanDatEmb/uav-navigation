from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "diagnostics"))
import frame_contract_probe as probe


class FrameContractProbeTest(unittest.TestCase):
    def test_summarize_preserves_odometry_frame_and_epoch(self):
        message = SimpleNamespace(
            header=SimpleNamespace(
                frame_id="px4_odom",
                stamp=SimpleNamespace(sec=3, nanosec=4),
            ),
            child_frame_id="base_link",
            pose=SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )),
            twist=SimpleNamespace(twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.1, y=0.2, z=0.3),
            )),
        )
        result = probe.summarize("/px4/estimator_odometry", message)
        self.assertEqual(result["stamp_ns"], 3_000_000_004)
        self.assertEqual(result["frame_id"], "px4_odom")
        self.assertEqual(result["child_frame_id"], "base_link")
        self.assertEqual(result["position"], [1.0, 2.0, 3.0])


if __name__ == "__main__":
    unittest.main()
