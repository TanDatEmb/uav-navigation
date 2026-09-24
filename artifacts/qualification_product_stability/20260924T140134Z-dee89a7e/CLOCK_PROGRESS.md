# Clock progress

Historical diagnostic pilot session `external-mode-check-20260924T093208-186236` contains one accepted-state gap above 150 ms: seq 2721→2722, 481.677 ms. Independent ROS `/clock` bag recorder arrival gap was 481.869 ms while source time advanced 4.000 ms. IMU observer gap was 481.517 ms with source +4.000 ms. This establishes a multi-stream upstream slowdown, not its first component. The pilot did not capture Gazebo-native clock/stats, so a Gazebo, bridge, executor, or host-specific cause is unproved.

For new runs, `--gazebo-native-diagnostic` separately observes `/world/<world>/stats` and `/world/<world>/clock` through Gazebo Transport. Its observer-loop witness distinguishes a native publication gap from delayed observation in that process. The ROS recorder remains a separate downstream clock witness. Both clocks retain source and receive times; no equality of clock domains is assumed.

