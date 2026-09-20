# H3 — publication/lock timeline

Source proves this final-exposure order:

1. publishCommand takes localization_transition_mutex_.
2. It takes input_mutex_.
3. It takes command_execution_lease_failure_latch_.transitionMutex().
4. It calls CommittedBundleStore::publishIfCurrent, which takes the store mutex and runs its finalizer before releasing it.
5. The finalizer repeats execution-state, world, command/bundle lease and identity checks, stamps authorization, then invokes rclcpp Publisher::publish before returning.
6. Only after the scopes end is last_command_store_publish_us_ stored.

onPropagatedOdometry takes localization_transition_mutex_ in the same documented order. The two paths are structurally mutually exclusive; a blocked publish/finalizer holds the shared localization lock and delays that odometry path. A two-thread executor and reentrant command/state groups permit competition but do not establish a total order for independent callbacks.

The store callback is intended to be a bounded finalizer. last_publish_us_ measures the ROS API call only. last_command_store_publish_us_ spans the larger interval from before lock acquisition through the store path, so it combines lock wait and hold. No target workload distribution, separate lock wait/hold, competing callback latency, RMW name, payload/subscriber load or matched runtime binary was available.

- Structural blocking: CONFIRMED_WITH_SCOPE by source lock scope; existing store test confirms store-lock waiting and freshness recheck.
- Material bottleneck in target workload: UNRESOLVED; no representative no-fault measurements. No lock movement or relaxed freshness experiment was made.

