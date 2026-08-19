import xml.etree.ElementTree as element_tree
from pathlib import Path

import yaml

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
    physics = world.find(".//physics")
    assert physics is not None
    assert physics.findtext("max_step_size") == "0.004"
    assert physics.findtext("real_time_factor") == "0.8"
    assert physics.findtext("real_time_update_rate") == "200"
    bridge = (ROOT / "bridge/px4_mid360_bridge.yaml").read_text()
    assert "/world/px4_lio_smoke/clock" in bridge
    assert "/sim/mid360/scan/points" in bridge
    assert "/sim/mid360/imu" in bridge
    assert "/sim/ground_truth/odometry" in bridge


def test_launcher_disables_px4_gazebo_truth_odometry_source():
    launcher = (ROOT.parents[1] / "tools/simulation/run_px4_mid360.sh").read_text()
    active_lines = {
        line.strip()
        for line in launcher.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    }
    # PX4's GZ bridge otherwise republishes this model's odometry as internal
    # vehicle_visual_odometry, competing with the ROS LIO external-vision path.
    assert "export PX4_PARAM_SIM_GZ_EN_ODOM=0" in active_lines
    assert "export PX4_PARAM_EKF2_EV_CTRL=15" in launcher
    assert "export PX4_PARAM_EKF2_GPS_CTRL=0" in launcher
    assert 'export PX4_PARAM_COM_RC_IN_MODE="${PX4_PARAM_COM_RC_IN_MODE:-4}"' in launcher
    # EKF2 must not fuse barometer data into the LIO external-vision estimate.
    assert "export PX4_PARAM_EKF2_BARO_CTRL=0" in launcher


def test_safety_map_profiles_have_lio_geometry_and_unique_worlds():
    names = ("open", "pillar", "occlusion", "corridor", "no_path")
    worlds = []
    for name in names:
        root = element_tree.parse(ROOT / f"worlds/{name}.sdf")
        world = root.find(".//world")
        assert world is not None
        worlds.append(world.attrib["name"])
        assert world.find(".//model[@name='ground_plane']") is not None
        assert world.find(".//include[uri='model://x500_mid360']") is not None
        assert world.find(".//light") is not None
    assert worlds == list(names)


def test_long_featured_low_altitude_corridor_has_roadside_features():
    world = element_tree.parse(ROOT / "worlds/long_featured.sdf")
    models = {model.attrib["name"]: model for model in world.findall(".//model")}
    assert all(f"long_featured_tree_{index:02d}" in models for index in range(1, 7))
    assert all(f"long_featured_pillar_{index:02d}" in models for index in range(1, 7))
    assert all(f"long_featured_texture_{index:02d}" in models for index in range(1, 7))
    assert not any("wall" in name for name in models if name.startswith("long_featured_"))
    tree_y = [float(models[f"long_featured_tree_{index:02d}"].findtext("pose").split()[1]) for index in range(1, 7)]
    assert all(4.0 <= abs(value) <= 4.5 for value in tree_y)
    texture_y = [float(models[f"long_featured_texture_{index:02d}"].findtext("pose").split()[1]) for index in range(1, 7)]
    assert all(4.8 <= abs(value) <= 5.6 for value in texture_y)
    mission = yaml.safe_load((ROOT.parent.parent / "config/runtime/missions/long_featured.yaml").read_text())
    altitudes = [waypoint["position"][2] for waypoint in mission["mission"]["waypoints"]]
    assert min(altitudes) >= 2.0 and max(altitudes) <= 3.1
