# C0-SW PX4 authority evidence

## Available evidence

The scenario harness already records `VehicleStatus` nav-state transitions, including `AUTO_LOITER`, and records `NavigationModeStatus`, command rejection, and PX4 input traces. These sources can show navigation leaving control and the downstream nav state observed by the harness.

## Missing episode correlation

The current recorder does not emit a producer-owned row for each `scheduleMode(AUTO_LOITER)` request and its asynchronous callback, keyed by activation generation and attempt. Consequently the existing evidence cannot prove request → callback → PX4 status order or loss-account each handover stage. No C0-SW reducer changes were made in this repair checkpoint. Handover C0-SW closure remains blocked until this evidence is added and exercised without granting it authority.

Required reducer closure after implementation: unresolved `0`, conflicts `0`, required references missing `0`, evidence drops `0`.
