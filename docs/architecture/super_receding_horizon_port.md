# SUPER-style receding horizon contract

The runtime uses the following contract for a long WP0 to WP1 leg. The
mission waypoint remains the only terminal goal; a horizon endpoint is an
intermediate state on the reference route.

1. At each replan tick, evaluate the active trajectory at
   `now + replan_forward_dt_s`. This is the splice state, not the measured
   pose with an invented zero acceleration.
2. Check the active trajectory against the inflated KnownFree map from that
   state forward. If it is safe, preserve a prefix up to
   `receding_distance_m` (or the first newly occupied/Unknown sample).
3. Run the route/A* search from the prefix endpoint state and extend the guide
   to `planning_horizon.maximum_distance_m`. The endpoint is not terminal and
   carries a non-zero continuation tangent. Even when WP1 is already inside
   the map, selector Direct is disabled until measured odometry is inside the
   WP1 acceptance radius; map visibility alone cannot create a terminal stop.
4. Publish `common_prefix + nominal_suffix/safety_suffix` in the atomic bundle.
   `branch_time` is after the prefix; PX4 executes the prefix and then the
   suffix without restarting at the current pose.
5. If no suffix is available, keep only the verified KnownFree prefix or issue
   a braking stop. A temporary `NoUsableSubGoal` is never terminal and must not
   hand over to POSCTL before WP1.

The safety horizon is bounded by observed KnownFree evidence. A 30 m sensor
can justify at most a 30 m KnownFree execution suffix (minus stopping margin);
40 m is a reference/search horizon unless the sliding map already contains
verified evidence beyond 30 m. Map allocation boundaries are not treated as
the sensing horizon; selector/A*/corridor verification remains the authority
for whether a candidate is executable. As the map slides, the next replan
extends the suffix.

The port deliberately does not copy SUPER's ROS/FIRI/GCOPTER dependencies.
It ports the safety-critical state machine: future splice, preserved prefix,
spatial receding distance, corridor validation, and branchable trajectory
handover.
