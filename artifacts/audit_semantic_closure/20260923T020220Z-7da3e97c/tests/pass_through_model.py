"""Audit-only abstract reducer. Run: python3 -m unittest discover -s tests -p '*_model.py'."""
import itertools
import unittest
from dataclasses import dataclass


@dataclass(frozen=True)
class Crossing:
    route: int
    localization: int
    waypoint: int
    before_stamp: int
    after_stamp: int
    error_m: float


class Model:
    def __init__(self):
        self.route = 1
        self.localization = 1
        self.waypoint = 1
        self.crossing = None
        self.continuation = False
        self.suffix = False
        self.certified_stop = False
        self.world = 1
        self.continuation_world = None
        self.accepted = False
        self.activated = False

    def event(self, name):
        if name == "crossing":
            self.crossing = Crossing(self.route, self.localization, self.waypoint, 10, 11, .1)
        elif name == "continuation":
            self.continuation = True
            self.continuation_world = self.world
        elif name == "suffix":
            self.suffix = True
        elif name == "certified_stop":
            self.certified_stop = True
        elif name == "world":
            self.world += 1
        elif name == "route":
            self.route += 1
            self.crossing = None
            self.continuation = False
        elif name == "localization":
            self.localization += 1
            self.crossing = None
            self.continuation = False
        elif name in ("reverse", "gap", "mission_reset", "expire"):
            self.crossing = None
        elif name == "accept":
            c = self.crossing
            readiness = (self.continuation and self.continuation_world == self.world) or (self.suffix and self.certified_stop)
            if c and c.route == self.route and c.localization == self.localization and c.waypoint == self.waypoint and readiness:
                self.accepted = True
                self.crossing = None
                self.waypoint += 1
        elif name == "activate":
            if self.accepted and self.continuation:
                self.activated = True


class PassThroughModel(unittest.TestCase):
    def test_both_readiness_orders(self):
        for events in itertools.permutations(("crossing", "continuation")):
            m = Model()
            for event in events:
                m.event(event)
            m.event("accept")
            self.assertTrue(m.accepted, events)
            self.assertFalse(m.activated)

    def test_target_callback_counterexample(self):
        crossing_t0, continuation_t0 = True, False
        crossing_t1, continuation_t1 = False, True
        self.assertFalse((crossing_t0 and continuation_t0) or (crossing_t1 and continuation_t1))
        m = Model()
        for event in ("crossing", "continuation", "accept"):
            m.event(event)
        self.assertTrue(m.accepted)

    def test_identity_and_geometry_fences(self):
        for fence in ("route", "localization", "reverse", "gap", "mission_reset"):
            m = Model()
            for event in ("crossing", fence, "continuation", "accept"):
                m.event(event)
            self.assertFalse(m.accepted, fence)

    def test_suffix_does_not_activate_successor(self):
        m = Model()
        for event in ("crossing", "suffix", "certified_stop", "accept", "activate"):
            m.event(event)
        self.assertTrue(m.accepted)
        self.assertFalse(m.activated)

    def test_suffix_without_certified_stop_cannot_accept(self):
        m = Model()
        for event in ("crossing", "suffix", "accept"):
            m.event(event)
        self.assertFalse(m.accepted)

    def test_world_revision_preserves_physical_observation_but_revokes_old_readiness(self):
        m = Model()
        for event in ("crossing", "continuation", "world", "accept"):
            m.event(event)
        self.assertFalse(m.accepted)
        self.assertIsNotNone(m.crossing)
        for event in ("continuation", "accept"):
            m.event(event)
        self.assertTrue(m.accepted)

    def test_retention_expiry(self):
        m = Model()
        for event in ("crossing", "expire", "continuation", "accept"):
            m.event(event)
        self.assertFalse(m.accepted)

    def test_no_crossing_no_acceptance(self):
        m = Model()
        for event in ("continuation", "accept"):
            m.event(event)
        self.assertFalse(m.accepted)


if __name__ == "__main__":
    unittest.main()
