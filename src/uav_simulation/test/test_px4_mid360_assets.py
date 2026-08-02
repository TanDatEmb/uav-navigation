import xml.etree.ElementTree as element_tree
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_mid360_model_contract():
    model = element_tree.parse(ROOT / "models/lidar_mid360/model.sdf")
    sensors = {sensor.attrib["name"]: sensor for sensor in model.findall(".//sensor")}
    assert sensors["lidar"].attrib["type"] == "gpu_lidar"
    assert sensors["mid360_imu"].attrib["type"] == "imu"
    assert sensors["lidar"].findtext("topic") == "/sim/mid360/scan"
    assert sensors["mid360_imu"].findtext("topic") == "/sim/mid360/imu"
    assert sensors["lidar"].findtext("gz_frame_id") == "livox_frame"
    assert sensors["mid360_imu"].findtext("gz_frame_id") == "livox_imu_frame"
    assert sensors["lidar"].findtext("pose") == "0 0 0 0 0 0"
    assert sensors["mid360_imu"].findtext("pose") == (
        "0.011 0.02329 -0.04412 0 0 0"
    )
    assert model.find(".//frame") is None
    assert sensors["lidar"].find("lidar") is not None


def test_x500_mid360_mount_contract():
    model = element_tree.parse(ROOT / "models/x500_mid360/model.sdf")
    uris = [uri.text for uri in model.findall(".//include/uri")]
    assert "x500" in uris
    assert "model://lidar_mid360" in uris
    joint = model.find(".//joint[@name='mid360_mount_joint']")
    assert joint is not None
    assert joint.attrib["type"] == "fixed"
    assert joint.findtext("parent") == "base_link"
    assert joint.findtext("child") == "livox_frame"


def test_px4_lio_smoke_world_and_bridge_contract():
    world = element_tree.parse(ROOT / "worlds/px4_lio_smoke.sdf")
    assert world.find(".//world").attrib["name"] == "px4_lio_smoke"
    bridge = (ROOT / "bridge/px4_mid360_bridge.yaml").read_text()
    assert "/world/px4_lio_smoke/clock" in bridge
    assert "/sim/mid360/scan/points" in bridge
    assert "/sim/mid360/imu" in bridge
