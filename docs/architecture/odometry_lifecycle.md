# Odometry lifecycle contract

## Public frames

The estimator and PX4 bridge expose independent public frames:

| Symbol | Meaning | Owner |
|---|---|---|
| `E_k` | raw PX4 `VehicleOdometry` epoch | PX4 |
| `P` | continuous `px4_odom` | `px4_odometry_bridge` |
| `M_i` | internal FAST-LIO map generation | FAST-LIO |
| `L` | continuous `lio_odom` | FAST-LIO lifecycle |
| `^L T_P` | comparison-only 4-DOF alignment | supervisor |

`P` is made continuous by applying PX4 reset metadata to position, attitude,
world velocity, body velocity, and covariance before publication. A reset
counter or metadata-generation change is a lifecycle event, not permission to
overwrite a timestamp or silently rebase history.

FAST-LIO increments `lio_generation` on reset. A saved last-good corrected
snapshot is owned by `LioLifecycleCoordinator`; it is invalidated on generation
change and may only be used to request a prior-driven reinitialization. The
in-flight branch does not call the stationary IMU initializer and requires a
full-attitude prior.

External odometry is a gated output. The PX4 authority path remains the
authority; LIO is sent to `vehicle_visual_odometry` only when supervisor gate
and publisher readiness are both true.
