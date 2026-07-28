# M1 upstream integration baseline

## Scope and identity

- Audit date: 2026-07-28 (Asia/Ho_Chi_Minh).
- Repository: `TanDatEmb/uav-navigation`.
- Baseline branch: `main`.
- Baseline commit: `49b2fba1406653cbf29b9328e4309745c5a1d53e`.
- Work branch: `fix/m1-upstream-fastlio2`.
- Host compiler: GCC 13.3.0.
- ROS distribution: ROS 2 Jazzy, `rmw_fastrtps_cpp`.
- Eigen: Ubuntu package `libeigen3-dev` 3.4.0-4build0.1.
- PCL: Ubuntu package `libpcl-dev` 1.14.0+dfsg-1.
- `livox_ros_driver2`: not installed at baseline.

## Baseline build and tests

The existing build tree was reused for the first non-mutating baseline run.
Consequently, the timing below is an incremental build measurement, not a clean
build benchmark.

```text
colcon build --symlink-install
7 packages finished
elapsed: 0.72 s
maximum resident set: 42,976 KiB

colcon test
80 tests, 0 errors, 0 failures, 0 skipped
elapsed: 1.47 s
maximum resident set: 45,648 KiB
```

Packages present:

```text
fast_lio_core
fast_lio_ros
fast_lio_tools
ikfom_vendor
navigation_bringup
uav_description
uav_simulation
```

## Audit findings

### Custom estimator implementation in the production path

- `fast_lio_core/estimation/manifold_state.*` implements project-local
  SO(3), gravity S2 tangent handling, box-plus and box-minus.
- `fast_lio_core/estimation/iterated_kalman_filter.*` forms and solves its own
  normal equations and posterior covariance.
- `fast_lio_core/estimation/process_model.*` and `imu_propagator.*` implement
  project-local prediction and covariance propagation.
- `FastLioPipeline` owns and invokes those custom implementations.
- No `esekfom::esekf` type is compiled, instantiated, owned, or called by the
  runtime estimator.

These files must be replaced in the production runtime by an `IkfomEstimator`
that owns the actual upstream `esekfom::esekf`. Project output views may remain,
but must not duplicate manifold or Kalman machinery.

### Vendor package that only carries an upstream name

- `ikfom_vendor/vendor/` contains only a README.
- `ikfom_vendor` exports an empty CMake interface target.
- `UPSTREAM.md` explicitly records that no upstream file is used.
- A build succeeds when IKFoM source is absent.
- There is no dependency smoke test.

The package must vendor the pinned IKFoM source, export its real include path,
preserve GPLv2 licensing, and fail configuration if the source is missing.

### Fake ikd-Tree production backend

- `IkdTreeRegistrationMap` is a voxel-centroid hash map.
- Its nearest-neighbor implementation constructs a candidate array by
  exhaustively iterating over every voxel and sorting the result.
- Local-map cropping erases voxel entries one-by-one.
- No upstream `KD_TREE`, `Build`, `Nearest_Search`, `Add_Points`,
  `Delete_Point_Boxes`, or `Box_Search` call is present.

The current implementation must leave the runtime path. It may be retained only
under `test/reference/` as a differential oracle. A new pinned
`ikd_tree_vendor` package and a real upstream `KD_TREE` wrapper are required.

### Missing runtime boundaries

- The production node subscribes only to `sensor_msgs/msg/PointCloud2`.
- Selecting `livox_custom` throws because no `livox_ros_driver2::msg::CustomMsg`
  adapter exists.
- Real Mid-360 topics are not the required `/livox/lidar` and `/livox/imu`.
- The timing parser itself rejects unknown values, but the local helper
  `adapterTiming()` still maps every non-`per_point` value to SIM mode. This is
  safe only because validation currently runs first and should be made strict
  locally as well.

### Ambiguous state result semantics

- `ProcessResult` always contains a member named `corrected_state`, including
  rejection and propagation-only paths.
- It has no explicit predicted state, optional corrected state, output-validity
  enum, or last-correction timestamp.
- Public odometry is guarded by `has_corrected_odometry`, but the data model
  still permits a propagated state to be mislabeled as corrected.

### Incomplete simulation and dataset verification

- `uav_simulation` contains one static world and no Mid-360 sensor rig.
- There are no deterministic static/yaw/translation/vertical/square launch
  scenarios, ROS-Gazebo bridge configuration, RViz configuration, or ground
  truth evaluation.
- No official Mid-360 dataset is present in `data/`, and there is no manifest,
  exact legacy message reader, reference comparison, or recorded dataset run.
- The offline CSV adapter does not demonstrate preservation of Livox
  `timebase` and `offset_time`.
- No end-to-end simulator or real-dataset estimator test exists.

### Missing performance and licensing evidence

- There is no ikd-Tree query/insert/delete benchmark and no proof that
  production nearest-neighbor search is sublinear.
- Root licensing does not yet contain a consolidated
  `THIRD_PARTY_NOTICES.md`.
- IKFoM and FAST-LIO are mentioned as GPLv2 references, but their source and
  license texts are not included because they are not actually vendored.
- Commercial/proprietary distribution implications are not called out
  prominently.

## Planned replacement/removal inventory

Production files to replace or remove after upstream-path tests pass:

```text
fast_lio_core/estimation/iterated_kalman_filter.*
fast_lio_core/estimation/manifold_state.* manifold operations
fast_lio_core/estimation/process_model.* custom Kalman propagation path
fast_lio_core/mapping/ikd_tree_registration_map.* fake backend implementation
```

Files to add:

```text
ikfom_vendor/vendor/IKFoM/**
ikd_tree_vendor/vendor/ikd-Tree/**
fast_lio_core/estimation/ikfom_estimator.*
fast_lio_core/estimation/estimator_state_view.*
fast_lio_core/registration/point_to_plane_measurement_model.*
fast_lio_ros/ros_livox_custom_adapter.*
simulation scenarios, dataset tooling, benchmarks and reports
```

No planner, planning world model, safety, PX4 fusion, controller, multi-LiDAR,
or high-rate propagated odometry work is authorized by this remediation.
