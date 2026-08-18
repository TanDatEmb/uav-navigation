# ADR-007: Fixed extrinsics by default

**Status:** accepted. LiDAR-to-IMU extrinsics are retained in the compatible
upstream state as static calibration; online estimation is not a runtime
feature. Rigid mounting is easier to validate and online drift could hide a
wrong calibration. A measured calibration is still required; package defaults
are clearly placeholders.
