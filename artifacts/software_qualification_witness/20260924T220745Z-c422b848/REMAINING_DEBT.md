# Remaining debt and next gate

1. Attribute the two upstream state-receive stalls in verification runs and add a measured gate witness for safety-stop outcome assessment; neither is covered by a nominal COMPLETE run.
2. Determine whether missing same-sample setpoint traces reflect expected adapter coalescing or diagnostic observer loss; exact Core-to-adapter command identity is already present, but PX4 firmware consumption is not proven.
3. Measure diagnostic publish overhead under representative load. Do not change 100/200/500 ms temporal gates to compensate for an unproven tail.
4. C0-IFP tracking, terminal physical performance, Gazebo validity and PX4 tuning remain deferred. Historical raw gaps remain `NOT_EVALUABLE`.

The next architecture work may start from an approved evidence-capable HEAD only after this scoped C0-SW contract verdict is accepted. World Temporal Contract is a separate branch; PX4 Hold redesign, planner tuning and physical qualification are outside this branch.
