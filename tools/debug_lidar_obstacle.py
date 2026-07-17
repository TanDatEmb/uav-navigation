#!/usr/bin/env python3
"""Debug tool to correlate Livox point cloud with ObstacleDistance bins.

Run while SITL is active:
    cd ~/Dev/uav-navigation
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 tools/debug_lidar_obstacle.py
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from px4_msgs.msg import ObstacleDistance, VehicleOdometry


class DebugLidarObstacle(Node):
    def __init__(self):
        super().__init__("debug_lidar_obstacle")

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub_cloud = self.create_subscription(
            PointCloud2, "/sim/livox/mid360/points", self.cloud_cb, qos_be
        )
        self.sub_obs = self.create_subscription(
            ObstacleDistance, "/fmu/in/obstacle_distance", self.obs_cb, qos_rel
        )
        self.sub_odom = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.odom_cb, qos_be
        )
        self.cloud = None
        self.obs = None
        self.yaw_deg = None
        self.get_logger().info("Collecting samples, press Ctrl-C to stop...")

    def odom_cb(self, msg: VehicleOdometry):
        q = msg.q
        # q = [w, x, y, z] in px4_msgs VehicleOdometry
        w, x, y, z = q[0], q[1], q[2], q[3]
        self.yaw_deg = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
        self.print_once()

    def obs_cb(self, msg: ObstacleDistance):
        self.obs = msg
        self.print_once()

    def cloud_cb(self, msg: PointCloud2):
        pts = read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if pts.shape[0] > 2000:
            idx = np.random.choice(pts.shape[0], 2000, replace=False)
            self.cloud = pts[idx]
        else:
            self.cloud = pts
        self.print_once()

    def print_once(self):
        if self.cloud is None or self.obs is None or self.yaw_deg is None:
            return
        if getattr(self, "_printed", False):
            return
        self._printed = True

        obs = self.obs
        dists = list(obs.distances)
        print("\n" + "=" * 70)
        print(f"VEHICLE YAW (NED): {self.yaw_deg:.1f} deg  (0=North, 90=East)")
        print("OBSTACLE DISTANCE")
        print(f"  frame={obs.frame} increment={obs.increment:.1f} "
              f"min={obs.min_distance} max={obs.max_distance} "
              f"angle_offset={obs.angle_offset}")
        print("  bins[0]=FWD-BODY, [18]=RIGHT-BODY, [36]=REAR-BODY, [54]=LEFT-BODY")
        for i in range(0, 72, 6):
            chunk = dists[i:i + 6]
            print(f"  bins[{i:2d}-{i + 5:2d}]: {chunk}")

        # Find min-dist points in each sector of the cloud.
        sectors = [
            ("FWD  ", lambda p: abs(p[1]) < 0.5 and p[0] > 0.5),
            ("RIGHT", lambda p: abs(p[0]) < 0.5 and p[1] > 0.5),
            ("REAR ", lambda p: abs(p[1]) < 0.5 and p[0] < -0.5),
            ("LEFT ", lambda p: abs(p[0]) < 0.5 and p[1] < -0.5),
        ]
        print("\nPOINT CLOUD SECTOR MINIMA (raw sensor frame)")
        for name, mask_fn in sectors:
            mask = np.array([mask_fn(p) for p in self.cloud])
            if not np.any(mask):
                print(f"  {name}: no points")
                continue
            sector = self.cloud[mask]
            horiz = np.linalg.norm(sector[:, :2], axis=1)
            idx = np.argmin(horiz)
            x, y, z = sector[idx]
            yaw = math.degrees(math.atan2(y, x))
            print(f"  {name}: point=({x:6.2f}, {y:6.2f}, {z:6.2f}) "
                  f"horiz={horiz[idx]:5.2f} yaw={yaw:6.1f} deg")

        # Overall cloud bounds.
        print("\nCLOUD BOUNDS")
        print(f"  x=[{self.cloud[:,0].min():.2f}, {self.cloud[:,0].max():.2f}] "
              f"y=[{self.cloud[:,1].min():.2f}, {self.cloud[:,1].max():.2f}] "
              f"z=[{self.cloud[:,2].min():.2f}, {self.cloud[:,2].max():.2f}]")

        print("=" * 70)
        # Keep running but stop printing after first snapshot.


def main():
    rclpy.init()
    node = DebugLidarObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
