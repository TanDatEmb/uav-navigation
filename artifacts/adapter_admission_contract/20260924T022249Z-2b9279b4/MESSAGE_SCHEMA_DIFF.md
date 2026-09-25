# Message schema diff

Pinned baseline `2b9279b4`: `NavigationCommand` has 74 nonconstant fields. Current branch has 27 control fields, with an explicit allowlist in `tools/check_navigation_command_contract.py`.

| Destination | Fields | Authority |
| --- | ---: | --- |
| `NavigationCommand` | 27 | Core command proposal, independently admitted by PX4 adapter |
| `NavigationExecutionDiagnostics` | 47 moved fields plus command/world correlation key | Observer only |
| `NavigationCommandRejection` | stage/reason/disposition and compact callback identity/timing | Adapter observer only |
| `NavigationCommandAdmission` | unchanged | Exact successful adapter admission only |

The observer schema repeats the existing command key `(mode_activation_id, localization_epoch, goal_epoch, mission_id, waypoint_index, request_id, bundle_generation, sample_id)` for an exact offline join. It does not acquire authority from repetition. `world_generation`, `world_revision`, `world_observation_stamp` are also copied for prior lifecycle provenance. `NavigationExecutionDiagnostics` is published best effort after the command and outside the execution transaction; the adapter does not subscribe.

Core's mission-command issuance no longer reads a wire diagnostic field. The final `publishIfCurrent` callback passes its local authorization decision directly to `rememberMissionCommandIssued(command, true)`. The diagnostic copy of authorization is emitted only for evidence. Emergency authorization reason and candidate commit result remain on `NavigationCommand` because the adapter's velocity-only experiment consumes them for a safety decision. Certified continuation fields remain because both the adapter contract and Core's exact admission receipt use them. See the field audit for every field.

**Compatibility:** The ROSIDL definition of `NavigationCommand` changes. Old bag files require their matching old overlay to deserialize that topic. No in-repository external live consumer requiring the old wire schema was found; unknown external consumers remain a deployment compatibility check. No package or topic silently aliases the old schema.
