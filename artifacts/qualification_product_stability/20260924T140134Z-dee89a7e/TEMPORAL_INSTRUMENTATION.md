# Temporal instrumentation

Existing state-transport trace records worker ready, publish call, adapter callback entry, mutex request/acquire, and accepted receive for exact localization/sequence. The explicit native diagnostic observer already records Gazebo stats/clock/LiDAR gap events and low-rate process/PSI samples. This branch adds a bounded 50 ms observer-loop scheduling witness so simultaneous native gaps are not attributed to Gazebo when the observer itself stopped. It also adds offline first-layer analysis and a cohort switch for the native observer. No flight callback reads these witnesses.

The observer additionally subscribes to native IMU metadata without copying its sensor payload, allowing native IMU arrival to be distinguished from bridged ROS IMU arrival when its coverage is complete. Classification is withheld when native gap events overflow or the observer lacks coverage over the candidate interval.

Native gap durations use the host monotonic clock. Each bounded event also carries wall-clock endpoints to join independent ROS and process traces; a wall-clock step alone cannot manufacture a native gap. The 1 Hz process and PSI witnesses carry both clock readings.

The runner explicitly sets the native observer's gap recording budget to 150 ms for this experiment; the observer's former 500 ms default would have missed the investigation range. This 150 ms value is diagnostic recording only. The adapter's 200 ms state boundary and 100 ms command lease remain unchanged.

The offline classifier requires both native stats and clock to show a receive gap and a source-progress deficit exceeding the diagnostic 150 ms trigger, while the observer loop stays scheduled. This rule classifies the natural 269 ms accepted-state tail and the controlled 350 ms Gazebo pause. The earlier percentage-of-gap heuristic had left both `UNRESOLVED` despite 239–360 ms native progress deficits; this is a diagnostic correction, with no product admission effect.

Historical seq 2721→2722: accepted gap 481.677 ms, producer gap 481.825 ms, ROS `/clock` recorder gap 481.869 ms with +4 ms source progress, IMU arrival gap 481.517 ms with +4 ms source progress. Publish→callback and adapter lock were short. The historical session has no native Gazebo stats/clock or host-loop witness, so exact first component remains `UNRESOLVED`, consistent with the predecessor verdict. The new analyzer does not backfill a Gazebo/bridge/host cause into that event.
