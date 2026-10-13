"""The Fast DDS profile of the SITL ROS processes (config/ros/fastdds_sitl.xml) and its use by sitl_hover.sh.

A Gazebo MID-360 cloud (645289 B) must fit one shared-memory segment, or best-effort clouds are lost in transport
(S1b Task 11 hover 20261013T151158Z-gps-hover). Static checks only; the delivery itself is measured in SITL."""
import pathlib
import re
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[3]
PROFILE = ROOT / "config" / "ros" / "fastdds_sitl.xml"
HARNESS = ROOT / "tools" / "uavnav" / "sitl_hover.sh"
NS = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
CLOUD_BYTES = 645289
MIN_SEGMENT = 16 * 1024 * 1024


class SitlDdsProfileTest(unittest.TestCase):
    def test_default_participant_carries_a_cloud_in_shared_memory(self):
        root = ET.parse(PROFILE).getroot()
        transports = {d.findtext("f:transport_id", namespaces=NS): d
                      for d in root.findall("f:transport_descriptors/f:transport_descriptor", NS)}
        participants = [p for p in root.findall("f:participant", NS) if p.get("is_default_profile") == "true"]
        self.assertEqual(len(participants), 1)
        rtps = participants[0].find("f:rtps", NS)
        used = [t.text for t in rtps.findall("f:userTransports/f:transport_id", NS)]
        types = {transports[t].findtext("f:type", namespaces=NS) for t in used}
        self.assertEqual(types, {"SHM", "UDPv4"})  # UDPv4 kept for peers that cannot use shared memory
        self.assertEqual(rtps.findtext("f:useBuiltinTransports", namespaces=NS), "false")
        shm = next(transports[t] for t in used if transports[t].findtext("f:type", namespaces=NS) == "SHM")
        self.assertGreaterEqual(int(shm.findtext("f:segment_size", namespaces=NS)), MIN_SEGMENT)
        self.assertGreaterEqual(int(shm.findtext("f:maxMessageSize", namespaces=NS)), CLOUD_BYTES)

    def test_harness_gives_the_profile_to_the_ros_processes_only(self):
        text = HARNESS.read_text(encoding="utf-8")
        self.assertIn('DDS_PROFILE="${WS_DIR}/config/ros/fastdds_sitl.xml"', text)
        self.assertIn('DDS_ENV=(FASTRTPS_DEFAULT_PROFILES_FILE="${DDS_PROFILE}")', text)
        commands = dict(re.findall(r"^\s*CMD_(\w+)=\((.*)$", text, re.MULTILINE))
        for name in ("control_bridge", "lidar_bridge", "estimation"):
            self.assertTrue(commands[name].startswith('env "${DDS_ENV[@]}"'), name)
        for name in ("xrce_agent", "px4_gz"):
            self.assertNotIn("DDS_ENV", commands[name], name)
        drivers = re.findall(r'DRIVER_CMD=\((.*)$', text, re.MULTILINE)
        self.assertEqual(len(drivers), 2)
        self.assertTrue(all(d.startswith('env "${DDS_ENV[@]}"') for d in drivers))


if __name__ == "__main__":
    unittest.main()
