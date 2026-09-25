# World temporal assessment

## AS-IS assessment contract

Current runtime's source-time assessment uses `classifyTimestampFreshness(now_ros_ns, identity.observation_stamp_ns, data_freshness_window_ns_)`, producing `VALID`, `STALE`, `FUTURE` or `INVALID`. No published world maps to `INVALID`. The identity's localization/generation/revision relation is checked independently by exact identity/transaction operations. This is safer than a single `world_valid` bit, but the concerns are spread across call sites and lack one named world-specific assessment value.

For future evidence, report the source-time freshness result separately from identity relation and from the active/pending transaction disposition. Do not label source-fresh as receive-fresh: receive/publication time is absent from WorldSnapshotIdentity.

## Implemented non-persistent value contract

`navigation_runtime::WorldTemporalAssessment` combines snapshot availability, immutable identity, current ROS time, and configured maximum source age. Its typed reason distinguishes no snapshot, invalid time contract, missing source stamp, stale source, future source, and current source. It carries the identity as evidence but deliberately does not decide identity relation, receive freshness, execution disposition, or mutation. It is an ephemeral value with no owner state or mutex. Runtime source-time checks now use this value while identity/owner checks remain in their canonical stores.

This is a naming/centralization improvement around the existing classifier, not a new temporal safety policy. The 500 ms threshold and decisions are unchanged. The contract remains partial because receive/integration/publication progress has no independent world timestamp and no isolated runtime source-stale test or C0-SW transaction events exist.

## Existing source-time meaning

The 500 ms window is applied to the world observation source stamp at candidate admission and active command authorization; a stale/future/invalid world causes candidate rejection or exact command freshness suspension. The same numeric parameter is separately used for execution state freshness and candidate exposure lease length. No threshold change is proposed.
