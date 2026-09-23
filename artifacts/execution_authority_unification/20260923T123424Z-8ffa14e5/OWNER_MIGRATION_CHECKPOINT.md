# Owner deletion checkpoints

| Former mutable owner / consumer | Mapping at this cut |
|---|---|
| Episode `beginGoal`, reset, clear | `ExecutionAuthority::beginGoal`, `reset`, `clearGoal` under one mutex |
| Episode command commit and staged activation | owner commit/activation atomically swap bundle, full goal and lifecycle |
| Episode fail closed / suspend / retain / sampled BACKUP / stopped hold / restart / recovery | typed owner transition methods; no second object or sync callback |
| Episode telemetry | `episodeSnapshot()` is a derived value from the same owner record |
| Episode tests | test-local fixture calls the product owner; world-revocation tests inspect owner directly |
| RuntimeNode executing goal and command epoch | active record goal plus active bundle epoch; absent before first commit |
| Publication | captures one owner snapshot and authorizes the exact sampled bundle via `publishIfCurrent` |
| Completion and terminal handling | reads owner active goal/bundle; completion witness remains independent measured/mission evidence |
| Retained recovery and renewal | reads owner snapshot/derived identity; desired and active may differ |
| World recertification | full active/staged goals remain paired with recertified bundle copies; exact revocation updates lifecycle in same transaction |
| Diagnostics | old wire fields derive from owner; no field is retained solely for telemetry |

Review of the above is based on explicit call-site migration and focused tests, not compilation alone. `ExecutionEpisode` class and RuntimeNode's `executing_goal_`/`command_goal_epoch_` are absent from product. The file name `committed_bundle_store.hpp` and test-only legacy type alias remain source compatibility names; neither adds state or a lock.
