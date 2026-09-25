# Stale-result disposition

| Result/evidence | Desired current | Exact active current | Causal evidence current | Action |
|---|---:|---:|---:|---|
| Failed replacement, certified incumbent | yes | yes | yes | Full retained-command validation; keep only if existing world/anchor/suffix/lease gates pass |
| `OptimizationFailed`, certified incumbent | yes | yes | yes | Same retained-command validation |
| Failed replacement after execution cutover | either | no | no | Discard result; newer execution intact |
| Failed replacement after desired revision change | no | either | no | Discard result |
| Success from old solve | no | either | no | Existing `PlanningKey`/commit-token rejection; no commit, stage or revoke |
| Stopped-recovery timeout, exact stopped hold | yes | yes | yes | Apply unchanged bounded timeout policy; conditional fail-close if due |
| Stopped-recovery timeout after hold/pending replacement | either | no | no | Discard; keep newer active and pending |
| Stale state L1, valid L2 installed | n/a | yes | no | Discard L1 failure; evaluate L2 on next command callback |
| Stale state L1, no replacement | n/a | yes | yes | Existing fail-close and downstream Hold fence |
| Old world recertification failure after newer world/execution | n/a | no | no | Existing world-version owner transaction rejects old result |

“Desired current” authorizes planning work only. It never by itself grants authority to revoke active execution. A current active pointer without current causal state/world/solve evidence is likewise insufficient.
