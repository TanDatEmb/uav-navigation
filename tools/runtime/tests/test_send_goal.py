import importlib.util
from pathlib import Path
import unittest
from types import SimpleNamespace


RUNTIME = Path(__file__).resolve().parents[1]


def _load_module():
    spec = importlib.util.spec_from_file_location("send_goal", RUNTIME / "send_goal.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SendGoalContractTest(unittest.TestCase):
    def test_manual_goal_contains_a_route_snapshot_matching_its_mirrors(self) -> None:
        module = _load_module()

        class Route:
            def __init__(self) -> None:
                self.waypoint_positions = []
                self.waypoint_ids = []
                self.waypoint_acceptance_radii_m = []
                self.waypoint_behaviors = []

        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="lio_odom"),
            mission_id="manual_goal",
            waypoint_index=0,
            request_id=1,
            acceptance_radius_m=0.5,
            behavior=1,
            BEHAVIOR_STOP=1,
            has_next_target=False,
            target=SimpleNamespace(x=5.0, y=0.0, z=1.0),
            route=Route(),
        )

        module._populate_route_snapshot(message)

        self.assertEqual(message.route.mission_id, message.mission_id)
        self.assertEqual(message.route.frame_id, message.header.frame_id)
        self.assertEqual(message.route.request_id, message.request_id)
        self.assertEqual(message.route.active_waypoint_index, message.waypoint_index)
        self.assertEqual(message.route.waypoint_positions, [message.target])
        self.assertEqual(message.route.waypoint_ids, ["manual_goal"])
        self.assertEqual(message.route.waypoint_acceptance_radii_m, [0.5])
        self.assertEqual(message.route.waypoint_behaviors, [message.BEHAVIOR_STOP])
        self.assertTrue(message.route.measured_progress_valid)

        message.request_id = 2
        module._populate_route_snapshot(message)
        self.assertEqual(message.route.request_id, 2)
        self.assertEqual(len(message.route.waypoint_positions), 1)
        self.assertEqual(len(message.route.waypoint_ids), 1)
        self.assertEqual(len(message.route.waypoint_acceptance_radii_m), 1)
        self.assertEqual(len(message.route.waypoint_behaviors), 1)

    def test_manual_goal_route_rejects_missing_contract_data(self) -> None:
        module = _load_module()
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="lio_odom"), mission_id="manual_goal",
            waypoint_index=0, request_id=1, acceptance_radius_m=0.5,
            behavior=1, BEHAVIOR_STOP=1,
            target=SimpleNamespace(x=5.0, y=0.0, z=1.0), route=None,
        )
        with self.assertRaises(ValueError):
            module._populate_route_snapshot(message)

    def test_manual_goal_route_rejects_nonfinite_coordinates(self) -> None:
        module = _load_module()
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="lio_odom"), mission_id="manual_goal",
            waypoint_index=0, request_id=1, acceptance_radius_m=0.5,
            behavior=1, BEHAVIOR_STOP=1,
            target=SimpleNamespace(x=float("nan"), y=0.0, z=1.0), route=None,
        )
        with self.assertRaises(ValueError):
            module._populate_route_snapshot(message)


if __name__ == "__main__":
    unittest.main()
