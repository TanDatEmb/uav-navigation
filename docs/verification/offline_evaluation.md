# Offline evaluation

`fast_lio_offline_evaluator` must invoke the same `FastLioPipeline` as the ROS
node. Given a rosbag2 or dataset adapter plus explicit estimator configuration,
it produces an `evaluation/` directory containing `summary.json`, `timing.csv`,
`synchronization.csv`, `state.csv`, `trajectory.csv`, `residuals.csv`,
`deskew.csv`, `map.pcd`, and `report.md` (where supported by available input).

Archive the configuration, input identity, expected frame/timing metadata, and
optional ground truth with each report. Re-run identical input/configuration to
check determinism; a separate offline estimator implementation is prohibited.

The CSV adapter is a development-only interface and must not be used to claim a
Mid-360 replay. Production ROS input has a typed Livox `CustomMsg` adapter which
preserves `timebase`, nanosecond `offset_time`, `tag`, and `line`; a raw ROS bag
still has to pass through that boundary. Follow `real_mid360_dataset.md` and
retain the acceptance gate as blocked until such a replay report exists.
# Timing semantics

All stage timers use monotonic wall-clock microseconds for one processed
measurement group. `total_processing_us` is the outer wall-clock interval.
`prediction_us`, `deskew_us`, `preprocessing_us`, `residual_build_us`,
`ikfom_update_us`, `map_insert_crop_us`, and `snapshot_us` describe nested
stages and therefore must not be summed and compared with the outer interval.
`map_maintenance_us` is nested inside `map_insert_crop_us`. Registration timers
may accumulate work across estimator iterations. These definitions apply
equally to ROS diagnostics and offline CSV output.
