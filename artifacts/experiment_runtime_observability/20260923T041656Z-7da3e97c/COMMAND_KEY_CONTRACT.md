# Command correlation identity

The normalized key is `(localization_epoch, goal_epoch, mission_hash, waypoint_index, request_id, bundle_generation, sample_id)`. This extends the audit's proposed four-tuple with mission and request. A producer process incarnation partitions repeated keys after a runtime restart, but the transported `NavigationCommand` has no incarnation field. Therefore an exact key that appears twice in a bag is **ambiguous** and the parser refuses a one-to-one pair. A consumer event does not infer the producer incarnation from the receiving process.

`sample_id` is producer-monotone within its lifetime in the pinned source; epoch, goal, request and bundle fields can change or reset. The parser tests duplicate keys across one run rather than assuming global uniqueness. A hash collision is theoretically possible; full mission IDs remain in the captured product command topic and should be checked when a collision is suspected. Instrumentation does not add a product command identity.

The source and receiver event key must match exactly. A missing publish or receive, a duplicate key, or a producer sequence gap is `INSUFFICIENT_TRACE` for timing inference. `COMMAND_PUBLISH.outcome=1` means the publish call returned; it does not prove middleware delivery. `COMMAND_RECEIVE.outcome=1` means the adapter's existing commit path admitted that command.
