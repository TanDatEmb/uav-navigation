# World evidence and recovery performance

For the 430 ms recovery SITL, mapping instrumentation reports 662 samples: mapping callback total p50 42.850 ms, p95 59.841 ms, p99 68.988 ms, max 82.553 ms; mapping total update p50 30.896 ms, p95 45.875 ms, p99 54.895 ms, max 68.492 ms. Active immutable revalidation p50 0.665 ms, p95 2.006 ms, p99 2.861 ms, max 4.395 ms. These are run diagnostics, not WCET.

For the long 700 ms control run, mapping callback total p50 38.384 ms, p95 56.530 ms, p99 70.539 ms, max 73.854 ms; mapping total update p50 28.178 ms, p95 40.231 ms, p99 49.954 ms, max 62.746 ms.

World evidence records: 1029 submitted/written, zero drops, zero serialization/write errors; final writer pending count 0. Recorder queue high-water occupancy and isolated per-event producer CPU cost are not exported, so they are `NOT_MEASURED` rather than inferred. The evidence path performs bounded event construction/enqueue; no JSON or disk formatting was added to the safety callback.
