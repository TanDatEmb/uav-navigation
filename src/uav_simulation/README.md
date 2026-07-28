# UAV simulation assets

`m1_registration_test.sdf` is a small Gazebo Harmonic world with planar and
pillar geometry suitable for static, yaw, translation and vertical registration
checks. It is not a vehicle model and includes no sensor plugin; its purpose is
to avoid presenting unvalidated sensor semantics as a test result.

Start it with `ros2 launch uav_simulation m1_world.launch.py`. A validated sensor
bridge must document its LiDAR header semantics, field layout, IMU units/frame,
and use simulated time consistently. If rays share a scan timestamp, configure
the estimator's explicit `simultaneous_scan` timing mode.
