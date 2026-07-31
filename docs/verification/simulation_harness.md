# Gazebo Harmonic PX4 Mid-360 session

The active simulation workflow starts and observes the PX4 Mid-360 Gazebo
session. Runtime evidence is written below `.artifacts/simulation`.

Run the automated smoke gate with:

```bash
make sim-px4-mid360-test
```

The observer checks process health, ROS graph readiness, sensor traffic and
estimator outputs. It is an integration smoke test, not real-sensor accuracy
evidence; use the dataset workflow for recorded-data validation.
