# ADR-005: Simulation simultaneous scans

**Status:** accepted. Gazebo scans with one timestamp are modelled as
simultaneous: per-point relative time is zero and deskew is bypassed with an
explicit diagnostic. We will not manufacture timing from point ordering or
curvature, and simulation does not fork a second estimator.
