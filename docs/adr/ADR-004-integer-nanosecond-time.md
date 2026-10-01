# ADR-004: Integer nanosecond time

**Status:** accepted. Absolute times are typed integer nanoseconds with a clock
domain. ROS conversion occurs only in `RosTimeConverter`; local durations alone
may become floating seconds. This prevents loss of precision and makes regressions
and scan bracketing explicit.
