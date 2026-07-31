# Upstream provenance

- Repository: <https://github.com/Livox-SDK/livox_ros_driver2>
- Tag: `1.2.6`
- Commit: `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`
- Integrated: 2026-07-28
- License: MIT
- Source: the two upstream message definitions and MIT license at the pinned
  commit; hardware-driver sources and assets are intentionally excluded.

The message definitions are unmodified. Their
upstream SHA-256 values are:

```text
CustomMsg.msg   f42d6709db951b1fa307e929e742c0593cbf0d1b0ff977d2ed63ad8d7cee0a96
CustomPoint.msg b64b31a8edc8c8b3765d82b5d3ccd2d2e1f217b9525ef7007ab918674c619c59
```

This directory is the single workspace source for the official ROS package and
message identity `livox_ros_driver2/msg/CustomMsg`. It is message-only and
contains no hardware transport, configuration, launch files, or SDK linkage.
Real deployments must provide the hardware driver from a separate deployment
workspace while avoiding duplicate package identities in one colcon workspace.
