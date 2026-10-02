import unittest

from tools.runtime.collision_geometry import box_signed_distance, cylinder_signed_distance, segment_min_signed_distance


class CollisionGeometryTests(unittest.TestCase):
    def test_yawed_box_does_not_use_its_axis_aligned_bounding_box(self):
        distance = box_signed_distance((1.3, -1.3, 0.0), (0, 0, 0), (2.0, 0.1, 0.1), (0, 0, 0.7853981633974483))
        self.assertGreater(distance, 0.0)

    def test_swept_segment_detects_a_pole_between_samples(self):
        distance = segment_min_signed_distance(
            (-1.0, 0.0, 0.0), (1.0, 0.0, 0.0),
            lambda point: cylinder_signed_distance(point, (0, 0, 0), 0.2, 1.0),
        )
        self.assertAlmostEqual(distance, -0.2, places=7)


if __name__ == "__main__":
    unittest.main()
