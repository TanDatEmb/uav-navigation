# Focused performance observation

No planner budget, command period, 100 ms adapter receive lease, 200 ms adapter state boundary, 500 ms freshness gate, or safety threshold changed. The conditional owner checks use bounded pointer/version comparisons under existing locks; planner solve and world sweep remain outside the owner mutex.

The three clean-source nominal bags contain 12 exact adapter admission transitions. Predecessor→successor gap min/median/max was **19.938/20.025/20.090 ms**, versus pinned reference **19.958/20.052/20.169 ms** by the same analyzer. This is the same order and far from the unchanged 100 ms lease. `HANDOFF_MEASUREMENTS.csv` retains individual values and the nearest PX4 setpoint timestamp, which is only an approximation because the setpoint message lacks request identity.

Periodic diagnostic samples of `command_store_publish_us` for the three sessions were: n=534/601/528; p50=40/41/42 us, p95=55/60/59 us, p99=65.7/67/65 us, max=182/77/75 us. `command_transition_lock_wait_us` and `execution_owner_publish_lock_wait_us` reported zero at their periodic samples; this does not establish a zero worst-case wait. Planning scheduling samples still include about 400 ms spikes, also present before this repair. No performance improvement or new deadline is inferred from three SITL runs.
