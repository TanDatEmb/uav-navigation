# Failed migration timeline (exact `db5880a0` code)

Traced seed-0 `long_featured` session `external-mode-check-20260923T104546-270858` used the Release binary built from the failed migration plus behavior-neutral H4–H8/stack diagnostics. It had the same map/seed/`tracking=off` mode as the fresh baseline; the first three repair completions used `relaxed` and are supplementary. Accepted prefix `[0,1,2]`; mission did not complete.

At the first failed handoff, simulation time in seconds:

| Event | Time | Identity/evidence |
|---|---:|---|
| H16 last admitted predecessor | 41.752 | request 3, bundle generation 8, sample 1429; exact `/navigation/command_admission` receipt |
| H2 acceptance / H4 desired transition | 41.768 | waypoint 2 accepted; desired request 4, epoch 5; predecessor request 3, execution epoch 4, generation 8 available |
| H5 successor desired goal installed | 41.768 | `hot_retained=1`, predecessor generation 8 still active |
| First divergence | 41.768 | `publishCommand()` called `failClosedLocked()` and cleared execution epoch to 0; Release stack captured `publishCommand()+0x37b2` |
| H6 planner submit | not reached before divergence | Planning cannot bridge a command owner revoked at H5 |
| H11/H12 first successor sample/publication | absent | No request-4 command in `pva_command` stream |
| H17 finite command expiry | 41.852 | last admitted sample carried the unchanged 100 ms lease; expiry is calculated from its source stamp, not a new event |
| H18 Hold request / AUTO_LOITER observation | 41.892 | adapter logged stale PVA command and requested Hold; scenario observed nav state 4 |

Adapter accepted 1,429 predecessor samples before this transition and did not first reject a valid predecessor for request non-regression. The first causal category is **COMMAND_PUBLICATION_GAP** caused by Core execution revocation; later stale command/Hold are consequences. The earlier migration `long_featured` 0/3 differential (`094018-180594`, `095317-204843`, `095556-208505`) is retained separately; this exact-stack run proves the mechanism on the requested `db5880a0` starting point.

H0 measured crossing, H1 continuation certificate creation, H9 candidate admission, H10 cutover, H13 adapter callback entry and H15 receipt emission are not independently timestamped in this trace. H9–H15 for the **successor** are absent after the first divergence. No timestamp is fabricated for those stages.
