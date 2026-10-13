"""PX4 localization profiles (tier c, SYSTEM_DESIGN §0, §4.1, §6.2; D20, D28, D33/O23).

A profile file names its localization ("gps" or "no_gps") and the EKF2 parameters of that
profile. Only the O23 allow-list is accepted, every key must be present, and each value must be
the decided one (D20: EKF2_EV_CTRL 7 / 15; D28: EKF2_EV_QMIN 0; O23: the rest). Anything else
raises ProfileError naming the key, so the SITL harness refuses to start (fail closed).
Harness-only PX4 parameters (UXRCE_DDS_SYNCT, SIM_GZ_EN_ODOM, COM_RC_IN_MODE, ...) are not
profile parameters; tools/simulation/run_px4_mid360.sh sets them.

Library use:

    profile = load(pathlib.Path("config/px4/profiles/gps.yaml"))

CLI (used by tools/simulation/run_px4_mid360.sh):

    python3 tools/uavnav/px4_profile.py --profile PATH --format env
    -> sorted "export PX4_PARAM_<NAME>=<value>" lines; exit 0 ok, 2 invalid
"""
import argparse
import dataclasses
import pathlib
import sys

import yaml

# O23 allow-list with the decided value per localization (D20, D28, D33/O23).
_DECIDED = {
    "gps": {
        "EKF2_EV_CTRL": 7,       # D20: EV pos/vel/hgt, EV yaw off
        "EKF2_EV_QMIN": 0,       # D28: quality is logged only
        "EKF2_EV_DELAY": 0,      # O23: timestamp_sample is already the scan time
        "EKF2_DELAY_MAX": 200,   # O23: horizon above the LIO latency [ms]
        "EKF2_EV_NOISE_MD": 0,   # O23: EV covariance from the message
        "EKF2_GPS_CTRL": 7,      # O23: GNSS fused
        "EKF2_HGT_REF": 1,       # O23: GNSS height reference
    },
    "no_gps": {
        "EKF2_EV_CTRL": 15,      # D20: EV yaw on
        "EKF2_EV_QMIN": 0,
        "EKF2_EV_DELAY": 0,
        "EKF2_DELAY_MAX": 200,
        "EKF2_EV_NOISE_MD": 0,
        "EKF2_GPS_CTRL": 0,      # O23: no GNSS aiding
        "EKF2_HGT_REF": 3,       # O23: EV height reference
    },
}
_TOP_LEVEL = ("localization", "px4_params")


class ProfileError(ValueError):
    """The profile file is missing, malformed or not the decided O23 profile."""


@dataclasses.dataclass(frozen=True)
class Profile:
    localization: str                  # "gps" | "no_gps"
    px4_params: dict[str, int | float]


class _UniqueKeyLoader(yaml.SafeLoader):
    """SafeLoader that refuses a duplicate mapping key instead of keeping the last one."""


def _construct_unique_mapping(loader, node, deep=False):
    seen = set()
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in seen:
            raise ProfileError(f"duplicate key {key!r} (line {key_node.start_mark.line + 1})")
        seen.add(key)
    return loader.construct_mapping(node, deep=deep)


_UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping)


def load(path: pathlib.Path) -> Profile:
    """Load and validate a profile file. Raises ProfileError (naming the key) on anything else."""
    try:
        text = pathlib.Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        raise ProfileError(f"{path}: cannot read: {exc}") from None
    try:
        data = yaml.load(text, Loader=_UniqueKeyLoader)  # noqa: S506 (SafeLoader subclass)
    except ProfileError as exc:
        raise ProfileError(f"{path}: {exc}") from None
    except yaml.YAMLError as exc:
        raise ProfileError(f"{path}: invalid YAML: {exc}") from None
    if not isinstance(data, dict):
        raise ProfileError(f"{path}: expected a mapping with keys {', '.join(_TOP_LEVEL)}")
    for key in data:
        if key not in _TOP_LEVEL:
            raise ProfileError(f"{path}: unknown key {key!r}")
    for key in _TOP_LEVEL:
        if key not in data:
            raise ProfileError(f"{path}: missing key {key!r}")
    localization = data["localization"]
    if localization not in _DECIDED:
        raise ProfileError(f"{path}: key 'localization' must be one of {sorted(_DECIDED)}, got {localization!r}")
    params = data["px4_params"]
    if not isinstance(params, dict):
        raise ProfileError(f"{path}: key 'px4_params' must be a mapping")
    decided = _DECIDED[localization]
    for name in params:
        if name not in decided:
            raise ProfileError(f"{path}: parameter {name!r} is not in the O23 allow-list {sorted(decided)}")
    for name, expected in decided.items():
        if name not in params:
            raise ProfileError(f"{path}: missing parameter {name!r}")
        value = params[name]
        if isinstance(value, bool) or not isinstance(value, int):
            raise ProfileError(f"{path}: parameter {name!r} must be an integer, got {value!r}")
        if value != expected:
            raise ProfileError(f"{path}: parameter {name!r} must be {expected} for localization "
                               f"{localization!r} (D20/D28/D33-O23), got {value}")
    return Profile(localization=localization, px4_params=dict(params))


def env_lines(profile: Profile) -> list[str]:
    """Sorted shell lines that PX4's rcS applies as parameters (PX4_PARAM_<NAME>)."""
    return sorted(f"export PX4_PARAM_{name}={value}" for name, value in profile.px4_params.items())


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Validate a PX4 localization profile and print it.")
    parser.add_argument("--profile", required=True, type=pathlib.Path, help="profile YAML path")
    parser.add_argument("--format", required=True, choices=("env",), help="output format")
    args = parser.parse_args(argv)
    try:
        profile = load(args.profile)
    except ProfileError as exc:
        print(f"px4_profile: {exc}", file=sys.stderr)
        return 2
    print("\n".join(env_lines(profile)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
