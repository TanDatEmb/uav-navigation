"""Tests for tools.uavnav.px4_profile (PX4 localization profiles, D20, D28, D33/O23)."""
import contextlib
import io
import os
import pathlib
import tempfile
import unittest

from tools.uavnav import px4_profile

ROOT = pathlib.Path(__file__).resolve().parents[3]
GPS = ROOT / "config" / "px4" / "profiles" / "gps.yaml"
NO_GPS = ROOT / "config" / "px4" / "profiles" / "no_gps.yaml"

GPS_TEXT = """\
localization: gps
px4_params:
  EKF2_EV_CTRL: 7
  EKF2_EV_QMIN: 0
  EKF2_EV_DELAY: 0
  EKF2_DELAY_MAX: 200
  EKF2_EV_NOISE_MD: 0
  EKF2_GPS_CTRL: 7
  EKF2_HGT_REF: 1
"""


class Px4ProfileTest(unittest.TestCase):
    def write(self, text):
        fd, path = tempfile.mkstemp(suffix=".yaml")
        self.addCleanup(lambda: os.path.exists(path) and os.remove(path))
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(text)
        return pathlib.Path(path)

    def assert_rejected(self, text, *needles):
        with self.assertRaises(px4_profile.ProfileError) as ctx:
            px4_profile.load(self.write(text))
        for needle in needles:
            self.assertIn(needle, str(ctx.exception))

    def test_d20_ev_ctrl_per_profile(self):
        gps = px4_profile.load(GPS)
        no_gps = px4_profile.load(NO_GPS)
        self.assertEqual(gps.localization, "gps")
        self.assertEqual(no_gps.localization, "no_gps")
        self.assertEqual(gps.px4_params["EKF2_EV_CTRL"], 7)
        self.assertEqual(no_gps.px4_params["EKF2_EV_CTRL"], 15)
        # The rest of O23 per profile.
        self.assertEqual((gps.px4_params["EKF2_GPS_CTRL"], gps.px4_params["EKF2_HGT_REF"]), (7, 1))
        self.assertEqual((no_gps.px4_params["EKF2_GPS_CTRL"], no_gps.px4_params["EKF2_HGT_REF"]), (0, 3))
        for profile in (gps, no_gps):
            self.assertEqual(profile.px4_params["EKF2_EV_DELAY"], 0)
            self.assertEqual(profile.px4_params["EKF2_DELAY_MAX"], 200)
            self.assertEqual(profile.px4_params["EKF2_EV_NOISE_MD"], 0)
        # A gps file with EV yaw fused (15) is the no_gps value: refused, naming the key.
        self.assert_rejected(GPS_TEXT.replace("EKF2_EV_CTRL: 7", "EKF2_EV_CTRL: 15"), "EKF2_EV_CTRL")

    def test_ev_qmin_is_zero(self):
        self.assertEqual(px4_profile.load(GPS).px4_params["EKF2_EV_QMIN"], 0)
        self.assertEqual(px4_profile.load(NO_GPS).px4_params["EKF2_EV_QMIN"], 0)
        self.assert_rejected(GPS_TEXT.replace("EKF2_EV_QMIN: 0", "EKF2_EV_QMIN: 1"), "EKF2_EV_QMIN")

    def test_unknown_param_rejected(self):
        self.assert_rejected(GPS_TEXT + "  EKF2_FOO: 1\n", "EKF2_FOO")
        # Harness parameters (O23) are not profile parameters.
        self.assert_rejected(GPS_TEXT + "  COM_RC_IN_MODE: 1\n", "COM_RC_IN_MODE")
        # A missing allow-listed key, an unknown top-level key, a bad localization,
        # a duplicate key and a non-integer value are refused too.
        self.assert_rejected(GPS_TEXT.replace("  EKF2_HGT_REF: 1\n", ""), "EKF2_HGT_REF")
        self.assert_rejected(GPS_TEXT + "extra: 1\n", "extra")
        self.assert_rejected(GPS_TEXT.replace("localization: gps", "localization: rtk"), "localization")
        self.assert_rejected(GPS_TEXT + "  EKF2_EV_CTRL: 7\n", "EKF2_EV_CTRL")
        self.assert_rejected(GPS_TEXT.replace("EKF2_DELAY_MAX: 200", "EKF2_DELAY_MAX: true"),
                             "EKF2_DELAY_MAX")
        self.assert_rejected("- not a mapping\n")

    def test_env_output(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rc = px4_profile.main(["--profile", str(GPS), "--format", "env"])
        self.assertEqual(rc, 0)
        lines = out.getvalue().splitlines()
        self.assertEqual(lines, sorted(lines))
        self.assertEqual(lines, [
            "export PX4_PARAM_EKF2_DELAY_MAX=200",
            "export PX4_PARAM_EKF2_EV_CTRL=7",
            "export PX4_PARAM_EKF2_EV_DELAY=0",
            "export PX4_PARAM_EKF2_EV_NOISE_MD=0",
            "export PX4_PARAM_EKF2_EV_QMIN=0",
            "export PX4_PARAM_EKF2_GPS_CTRL=7",
            "export PX4_PARAM_EKF2_HGT_REF=1",
        ])
        bad = self.write(GPS_TEXT + "  EKF2_FOO: 1\n")
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = px4_profile.main(["--profile", str(bad), "--format", "env"])
        self.assertEqual(rc, 2)
        self.assertEqual(out.getvalue(), "")
        self.assertIn("EKF2_FOO", err.getvalue())
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(px4_profile.main(["--profile", str(bad) + ".missing", "--format", "env"]), 2)


if __name__ == "__main__":
    unittest.main()
