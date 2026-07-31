import math
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pointcloud_probe import MissingXyzFields, inspect_pointcloud


def cloud(points, datatype=7, endian=False, dense=False, names=("x", "y", "z")):
    code, size = ("f", 4) if datatype == 7 else ("d", 8)
    prefix = ">" if endian else "<"
    fields = [SimpleNamespace(name=name, offset=index*size, datatype=datatype, count=1)
              for index, name in enumerate(names)]
    data = b"".join(struct.pack(prefix+code*3, *point) for point in points)
    return SimpleNamespace(
        width=len(points), height=1, point_step=size*3, row_step=size*3*len(points),
        fields=fields, data=data, is_bigendian=endian, is_dense=dense,
        header=SimpleNamespace(frame_id="lidar", stamp=SimpleNamespace(sec=1, nanosec=2)))


class PointCloudProbeTest(unittest.TestCase):
    def test_float32_little_endian_finite(self):
        result = inspect_pointcloud(cloud([(1, 2, 3), (0, 0, 0)]))
        self.assertEqual(result.finite_xyz_count, 2)
        self.assertEqual(result.zero_xyz_count, 1)
        self.assertEqual(result.finite_ratio, 1)

    def test_float64_big_endian_and_uniform_bounded_sample(self):
        result = inspect_pointcloud(cloud([(float(i), 1, 1) for i in range(20)], 8, True), 4)
        self.assertEqual(result.sampled_points, 4)
        self.assertEqual(result.finite_xyz_count, 4)

    def test_nan_positive_and_negative_inf(self):
        result = inspect_pointcloud(cloud([
            (math.nan, 0, 0), (math.inf, 0, 0), (-math.inf, 0, 0), (1, 1, 1)]))
        self.assertEqual((result.nan_xyz_count, result.positive_inf_xyz_count,
                          result.negative_inf_xyz_count), (1, 1, 1))
        self.assertEqual(result.finite_ratio, .25)

    def test_dense_contract_violation(self):
        self.assertTrue(inspect_pointcloud(cloud([(math.inf, 0, 0)], dense=True)).density_contract_violation)

    def test_missing_xyz(self):
        with self.assertRaises(MissingXyzFields):
            inspect_pointcloud(cloud([(1, 2, 3)], names=("x", "y", "intensity")))


if __name__ == "__main__":
    unittest.main()
