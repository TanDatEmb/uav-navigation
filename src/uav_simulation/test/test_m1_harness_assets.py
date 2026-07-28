import ast
import xml.etree.ElementTree as element_tree
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_harness_world_has_real_sensor_types_and_bridge_topics():
    world = element_tree.parse(ROOT / "worlds/m1_mid360_harness.sdf")
    assert world.find(".//include/uri").text == "model://mid360_imu_rig"
    rig = element_tree.parse(ROOT / "models/mid360_imu_rig/model.sdf")
    sensor_types = {sensor.attrib["type"] for sensor in rig.findall(".//sensor")}
    assert {"gpu_lidar", "imu"}.issubset(sensor_types)
    bridge = (ROOT / "bridge/m1_sensor_bridge.yaml").read_text()
    assert "sensor_msgs/msg/PointCloud2" in bridge
    assert "sensor_msgs/msg/Imu" in bridge
    assert "gz.msgs.PointCloudPacked" in bridge
    assert "gz.msgs.IMU" in bridge


def test_harness_launch_is_parseable_and_keeps_estimator_opt_in():
    launch = ROOT / "launch/m1_mid360_harness.launch.py"
    ast.parse(launch.read_text(), filename=str(launch))
    source = launch.read_text()
    assert 'DeclareLaunchArgument("start_estimator", default_value="false")' in source
