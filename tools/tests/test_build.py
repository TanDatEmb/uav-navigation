from pathlib import Path
import importlib.util
import sys
import unittest

BUILD_MODULE = Path(__file__).resolve().parents[1] / "runtime" / "build.py"
SPEC = importlib.util.spec_from_file_location("runtime_build", BUILD_MODULE)
assert SPEC is not None and SPEC.loader is not None
build = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(build)


class BuildEnvironmentTest(unittest.TestCase):
    def test_ros_environment_exposes_system_ros_python_packages(self) -> None:
        environment = build.ros_environment()
        paths = environment.get("PYTHONPATH", "").split(build.os.pathsep)
        if build.ROS_SYSTEM_PYTHON_PATH.is_dir():
            self.assertIn(str(build.ROS_SYSTEM_PYTHON_PATH), paths)


if __name__ == "__main__":
    unittest.main()
