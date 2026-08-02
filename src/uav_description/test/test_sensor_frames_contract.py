import xml.etree.ElementTree as element_tree
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
XACRO_NS = "{http://www.ros.org/wiki/xacro}"


def test_sensor_xacro_has_canonical_chain_and_required_mount():
    path = ROOT / "urdf/uav_sensor_frames.urdf.xacro"
    tree = element_tree.parse(path)
    args = {arg.attrib["name"]: arg for arg in tree.findall(f"{XACRO_NS}arg")}
    assert "livox_mount_xyz" in args
    assert "livox_mount_rpy" in args
    assert args["livox_mount_xyz"].attrib["default"] == (
        "__required_livox_mount_xyz__"
    )
    assert args["livox_mount_rpy"].attrib["default"] == (
        "__required_livox_mount_rpy__"
    )
    assert args["livox_lidar_to_imu_xyz"].attrib["default"] == (
        "0.011 0.02329 -0.04412"
    )

    joints = {joint.attrib["name"]: joint for joint in tree.findall("joint")}
    assert joints["base_link_to_livox_frame"].find("parent").attrib["link"] == "base_link"
    assert joints["base_link_to_livox_frame"].find("child").attrib["link"] == "livox_frame"
    assert joints["livox_frame_to_livox_imu_frame"].find("parent").attrib["link"] == "livox_frame"
    assert joints["livox_frame_to_livox_imu_frame"].find("child").attrib["link"] == "livox_imu_frame"


def test_standalone_and_canonical_launch_mount_policy():
    standalone = (ROOT / "launch/publish_sensor_frames.launch.py").read_text()
    bringup = (
        ROOT.parent / "navigation_bringup/launch/fast_lio.launch.py"
    ).read_text()
    assert '"livox_mount_xyz"' in standalone
    assert '"livox_mount_rpy"' in standalone
    assert 'default_value="true"' in bringup
    assert "canonical static base_link -> livox_frame ->" in bringup
