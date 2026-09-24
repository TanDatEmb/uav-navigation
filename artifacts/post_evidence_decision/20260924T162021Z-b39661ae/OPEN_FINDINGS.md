# Open findings after PATH D selection

| ID | Classification | Finding | Owner / next evidence |
|---|---|---|---|
| PD-001 | `POLICY_GAP` | No approved C0 source-paired coverage/pairing limits. | Policy authority decides P-COV-01 after clock/frame/gap evidence. |
| PD-002 | `POLICY_GAP` | No approved independent-truth tracking p95/maximum position/velocity limits. | Policy authority decides P-TRK-02 after estimator/truth and hazard allocation. |
| PD-003 | `POLICY_GAP` | No approved measured-motion metric, role/phase scope or acceptance limits. | Policy authority decides P-MOT-03 after dynamics/filter evidence. |
| PD-004 | `EVIDENCE_CONTRACT_DEFECT` | 140 historical lifecycle transactions lack exact producer/consumer closure. | Separate focused evidence repair: 87 bundle identity, 30 export-owner, 23 consumer/supersession; new producer-owned capture required. |
| PD-005 | `EVIDENCE_CONTRACT_DEFECT` | 5,957/32,745 raw command references cannot join a valid exact qualification transaction. | Same focused evidence repair; this is not proof of invalid flight commands. |
| PD-006 | `EVIDENCE_CONTRACT_DEFECT` | Historical A3 changed world revision at repeated source tick remains conflicting evidence. | Exact recertification/authorization witness; World behavior not inferred. |
| PD-007 | `SIMULATION_LIMITATION` | Historical approximately 482 ms producer and clock gap lacks first stalled layer. | Deferred simulation infrastructure evidence; no Gazebo/DDS blame assigned. |
| PD-008 | `FLIGHT_PERFORMANCE_DEFERRED` | Two terminal anchor errors 0.762/0.759 m exceeded unchanged 0.750 m gate. | Separate PX4/vehicle stopping characterization; not reclassified as mission success. |

There is no `UNKNOWN` blocker classification. No listed evidence gap is silently treated as a product-behavior defect, policy approval or PASS.
