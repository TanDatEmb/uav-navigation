# WP-A2 open questions

## OQ-01 — Required architecture references are not in baseline

- **Question:** Should WP-A2 be re-run after `ARCHITECTURE_REVIEW.md`, ADR-013..016,
  and `risk_register_20260928.md` are committed to `main`?
- **Evidence:** These paths are absent at `main@7e0b850`; matching documents
  existed only as untracked files in the separate WP-D0 working clone used for
  contextual reading. The ICD source anchors and baseline SHA remain pinned to
  `7e0b850`.
- **Options:** (a) accept this ICD as source-pinned and treat the current
  architecture/risk documents as contextual only; (b) rebase/re-run after the
  documents land, without changing the current baseline claims.
- **Status:** open; no behavior or source change made.

## OQ-02 — PX4 offered QoS and firmware endpoint ownership

- **Question:** What exact offered QoS does the target PX4/DDS deployment
  negotiate for each `/fmu/in/*` and `/fmu/out/*` versioned topic?
- **Evidence:** Product code and pinned `px4_ros2_interface_lib` identify topic
  names, message versions, and local ROS subscription/publication QoS, but PX4
  firmware transport is outside this clone. `icd_topics.yaml` therefore uses
  `NOT_STATICALLY_DECLARED` for the external endpoint.
- **Next evidence:** capture `ros2 topic info --verbose` for the exact PX4
  build/agent and store firmware commit plus DDS profile.

## OQ-03 — Full upstream PX4 schemas versus product-touch fields

- **Question:** Does the destination ICD require every field of every pinned
  `px4_msgs` message, including fields never read by product or observer code?
- **Evidence:** `icd_msgs.yaml` now lists every serialized field of the 22 PX4
  message types touched by product code, the pinned interface library, or the
  SITL observers. Untouched fields are retained with `unused: true`; constants
  remain in the pinned `.msg` files.
- **Options:** accept this full touched-type schema; or generate a separate
  appendix for every non-touched `px4_msgs` type, which would not change
  product ownership facts.

## OQ-04 — Mission interface lifecycle contract

- **Question:** Should `/navigation/mission_progress` and
  `/navigation/command_admission` be guaranteed to exist before a mission is
  loaded, or should subscribers be conditional too?
- **Evidence:** runtime creation is guarded by `mission_progress_` at
  `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:1753-1763`,
  while the PX4 mode subscriber is unconditional at
  `src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp:291-296`.
- **Status:** documented as an AS-IS anomaly; no refactor decision made in this
  read-only work package.
