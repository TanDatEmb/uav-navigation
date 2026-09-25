# Scope and deferred findings

This branch changes only offline evaluation, scenario evidence recording, tests and documentation. It does not change product source under `src/`, control outputs, safety gates, PX4, Gazebo or sensor models.

- D-001 `DEFERRED_SIMULATION_INFRASTRUCTURE_LIMITATION`: historical simulator/producer stall first component is not proven. The product-stability evidence observes a producer gap and clock gap near 482 ms, but that is insufficient to attribute the initial stall to Gazebo, bridge, host or DDS.
- D-002 `DEFERRED_FLIGHT_DYNAMICS_AND_PX4_TUNING`: two terminal runs stopped with 0.762/0.759 m anchor error against the unchanged 0.750 m gate; independent truth and LIO both showed motion. The gate rejection remains an observed correct safety response, not a mission success.

Software evidence and integrated flight performance must be reported separately. Deferral never turns an absent witness into PASS.
