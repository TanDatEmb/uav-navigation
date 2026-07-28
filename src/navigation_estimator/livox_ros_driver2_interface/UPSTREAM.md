# Upstream provenance

- Repository: <https://github.com/Livox-SDK/livox_ros_driver2>
- Tag: `1.2.6`
- Commit: `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`
- Integrated: 2026-07-28
- License: MIT
- Files used:
  - `msg/CustomMsg.msg`
  - `msg/CustomPoint.msg`

The two message definitions are copied verbatim from the pinned commit. Their
upstream SHA-256 values are:

```text
CustomMsg.msg   f42d6709db951b1fa307e929e742c0593cbf0d1b0ff977d2ed63ad8d7cee0a96
CustomPoint.msg b64b31a8edc8c8b3765d82b5d3ccd2d2e1f217b9525ef7007ab918674c619c59
```

This is deliberately an interface-only package using the official ROS package
and message identity `livox_ros_driver2/msg/CustomMsg`. It does not contain
Livox SDK2, network transport, configuration, launch files, or the
`livox_ros_driver2_node` executable. Install and run the official pinned driver
on the sensor host for hardware operation.

Because ROS message identity contains the package name, this interface-only
package and the full driver package cannot both be built in the same colcon
workspace. Replace this directory with the full pinned upstream package when
Livox SDK2 is available; consumers do not require source changes.
