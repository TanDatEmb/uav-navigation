# Isolated world-source stale runtime evidence

Status: **NOT RUN**.

No fault in this worktree stops only `/lio/mapping_observation` while keeping odometry, estimator health, Core timers, DDS, simulator clock and PX4 adapter live. No raw session, metadata, scenario trace, adapter receipt timeline, execution exposure transition, lease result or Hold request is available.

The previous world temporal audit reached the same limit. A source review shows the existing source-stamp expiry path can suspend exact execution, but that is not runtime evidence that the independent producer stream was isolated or that the complete boundary response occurred.

Do not classify as a pass or as a resolved source-staleness defect.
