# N2 terminal recovery classification

Classification: **EXPECTED_FAIL_CLOSED_FROM_DEFERRED_PX4_TRACKING**.

Session: `external-mode-check-20260925T092630-369436`; accepted waypoints 0–3 and then PAUSED_SAFETY_STOP. There was no World suspension in the event chain. The active trajectory remained identified through bundle generation 20; the terminal decision's exact state was fresh, current, anchored, and known-free, but its command bridge was unusable.

The first causal failure is in a hot replan near the terminal segment. `navigation_runtime_node` logged backup replanning failure with anchor error `0.258 m` against the tracking limit `0.250 m`, projected anchor error `0.245 m`, relative anchor speed `0.020 m/s`, and state age `0.012 s`. The current retained-decision witness records `owner_snapshot_current=1`, `callback_request_current=1`, `monitor_window_current=1`, `final_freshness_reason=0`, `final_anchor_valid=1`, `final_body_known_free=1`, `final_bridge_usable=0`, then `after_command_available=0` and `after_failure_latched=1`. The runtime emits SAFETY_STOP; the adapter subsequently requests PX4 Hold after External Mode exits.

The exact witness makes this outcome assessable as C0-SW PASS: the software fail-closed decision is evidenced. Runtime mission outcome remains BLOCKED/PAUSED_SAFETY_STOP, and C0-IFP remains NOT_EVALUABLE due its own tracking/motion policy evidence. This is not a mission completion or physical tracking pass.

The trigger is a tracking/terminal-performance boundary only 8 mm beyond the existing limit. No threshold is changed. This is allowed as a known safe pre-beta stability debt for architecture integration; repeated similar failures remain a beta blocker. It is not treated as a newly introduced authority, World, or qualification-scope defect.
