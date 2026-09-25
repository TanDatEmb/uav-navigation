# Lifecycle unresolved classes

`LIFECYCLE_UNRESOLVED_CLASSES.csv` lists every unresolved transaction in the ten-run predecessor cohort, with raw session and reasons reproducible by `reevaluate_raw.py`. The baseline reducer yields **140 unresolved transactions** over the ten runs; this is a structural cluster, not 140 independent bugs.

The mutually exclusive report classification (priority order in the script) is **87 missing bundle identities, 30 missing export-owner identities, 23 missing consumer or supersession witnesses**. The original reducer labeled many request-only rows as `BUNDLE_IDENTITY_CONFLICT` when there were zero generations. That was a classification bug; zero is now `BUNDLE_IDENTITY_MISSING`, while two different generations remain a conflict. This does not suppress their incomplete status.

| Class | Source-backed interpretation | Safe disposition |
|---|---|---|
| Missing producer/export identity | Authorization or PX4 input has an active bundle generation but no unique matching export witness; sample `causal_planning_cycle_id=0` is not bundle ownership. | `MISSING_EVIDENCE`; never join by nearest timestamp. |
| Missing bundle identity | A request-only planning cycle lacks any exported, active or authorized generation. Some candidate attempts may legitimately produce none, but the missing outcome/supersession witness prevents closure. | Preserve exact rows; require producer-owned terminal disposition. |
| Missing consumer/supersession witness | Request/export has no later active/authorization/PX4 input. It may have been legitimately superseded, but the current evidence does not always prove that. | `MISSING_EVIDENCE` unless exact supersession exists. |
| Missing identity | Goal transport request or non-command adapter trace has no planner cycle/active bundle. | Keep in separate protocol domain. |
| Duplicate/conflicting payload | Reducer checks identical event IDs for contradictory values. | `CONFLICTING_EVIDENCE`, never last-write-wins. |

The existing raw cohort cannot be retroactively supplied with a producer cycle or an execution transition event that it did not record. `LIFECYCLE_ATTRIBUTION_INCOMPLETE` remains valid. New capture must emit exact authority transition identity at the producer; recorder order alone is insufficient.
