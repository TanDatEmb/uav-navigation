# Focused SITL results

`long_featured` map seed 0, unchanged product thresholds, exact PX4 checkout `deaff86ee335dd697677bcfc2415a23878e1b895` and binary SHA256 `e440bd77fbacdc422eaafdb168fec01554298d545f11e2b004a640b04e324ff9`.

| Session suffix | Source / tracking | Accepted prefix | COMPLETE | Interpretation |
|---|---|---|---|---|
| `095057-201248`, `095812-211924` | pinned baseline / off | `[0,1,2,3,4]` each | 2/2 | Retained prior matched baseline reference |
| `103615-258561` | pinned baseline / off | `[0,1,2,3,4]` | Yes | Fresh baseline reference |
| `094018-180594`, `095317-204843`, `095556-208505` | prior migration / off | `[0,1]`, `[0]`, `[0,1]` | 0/3 | Retained differential regression; source predates final failed checkpoint |
| `104546-270858` | exact failed `db5880a0` / off | `[0,1,2]` | No | H4/H5 retained predecessor, `publishCommand()` revoked it at 41.768 s; Hold at 41.892 s |
| `112040-309344`, `112251-312566`, `112456-315691` | repair / off | `[0,1,2,3,4]` each | **3/3** | Matched parity; all manifests VALID; 12 PASS handoffs, no nominal identity/continuity reject or unintended lease expiry |
| `105937-284324`, `110442-292258`, `110706-295798` | repair / relaxed | `[0,1,2,3,4]` each | 3/3 | Supplementary liveness; **not tracking-matched** to baseline |
| `110922-299270` | repair / relaxed pillar | `[0]` | No | Stopped recovery then safety stop; not mission parity evidence |
| `111139-302607` | repair / off pillar | `[0,1,2]` | No | STOP 0/1 and PASS 2 accepted; later world revision 510 invalidated generation 9 with no certified brake, so Hold was fail-closed |
| `112700-318895` | repair / off open + 650.180 ms Core pause | `[0]` | No | Adapter detected stale PVA and requested Hold; mapping unconsumed-cloud gate ended the run before native AUTO_LOITER was observed. Lease result only; Hold confirmation `NOT_EVALUABLE`. |
| `112903-322160` | repair / off open + 650.146 ms Core pause | `[0]` | No | Adapter detected stale PVA, requested Hold; native AUTO_LOITER observed. Mapping unconsumed-cloud gate also fired, so report remained `BLOCKED`. Focused lease-fence observation, not isolated world-source-staleness evidence. |

Two attempted repeats `110156-288722` and `110157-288967` had no simulation samples because documentation edits invalidated the Release source fingerprint before launch. They are **preflight failures**, excluded from the nominal-run denominator; a new validated Release manifest preceded the three `relaxed` repetitions. Their `report.json` retains the invalid-manifest reason.

The matched normal mission completed **3/3** with no safety threshold change. PX4 adapter tracking-setpoint handoff gaps were 4–28 ms over 12 transitions (min/median/max `4/16/28` ms), below the unchanged 100 ms receive lease. The off pillar failure is a separate latest-world certificate invalidation at revision 510; it preserved the safety stop and observed PX4 Hold, but does not establish pillar completion. The pause fault directly exercises command heartbeat loss, not an isolated stale-world source timestamp. All successful mission observations remain diagnostic: runner overall `FAIL` is retained because versioned evaluation was not qualification eligible. No SITL run is flight acceptance.
