# Offline evaluation

`fast_lio_offline_evaluator` must invoke the same `FastLioPipeline` as the ROS
node. Given a rosbag2 or dataset adapter plus explicit estimator configuration,
it produces an `evaluation/` directory containing `summary.json`, `timing.csv`,
`synchronization.csv`, `state.csv`, `trajectory.csv`, `residuals.csv`,
`deskew.csv`, `map.pcd`, and `report.md` (where supported by available input).

Archive the configuration, input identity, expected frame/timing metadata, and
optional ground truth with each report. Re-run identical input/configuration to
check determinism; a separate offline estimator implementation is prohibited.
