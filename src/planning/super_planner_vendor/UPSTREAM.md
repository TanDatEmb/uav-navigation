# SUPER planner provenance

The planner core in this package is ported from `hku-mars/SUPER` at commit
`2ad3419c127a617c6d7df6925e81a14175a9c096`. ROS 1/ROS 2 visualization,
simulator, FSM and research logging wrappers are intentionally excluded from
the realtime target. The product adapter keeps the upstream planner data
structures and optimization path while using the workspace-owned ROG-Map
implementation and ROS 2 input/output boundaries.

The imported source retains its upstream copyright and license headers. Any
future modification must preserve the corresponding provenance and license
notices.
