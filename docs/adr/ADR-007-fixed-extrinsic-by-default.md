# ADR-007: Fixed extrinsics by default

**Status:** accepted. LiDAR-to-IMU extrinsics are retained in the compatible
model but online estimation defaults off. Rigid mounting is easier to validate
and online drift could hide a wrong calibration. A measured calibration is still
required; package defaults are clearly placeholders.
