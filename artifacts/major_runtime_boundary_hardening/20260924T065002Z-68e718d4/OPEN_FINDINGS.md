# Open findings

| ID | Class | Finding | Risk | Required evidence | Target |
|---|---|---|---|---|---|
| A-001 | BLOCKING_FINDING | Runner requested `tracking=off` while Core/adapter enabled the zero-coefficient bypass under `use_sim_time=true`. | False configuration provenance and non-evaluable nominal evidence. | Explicit policy, live witnesses, three tracking-off SITLs. | Phase A |
| A-002 | RECORD_ONLY | Historical tracking-zero default is a documented diagnostic bypass; true tracking-off may expose mission or health failures. | Liveness/physical response unknown until SITL. | Exact rejection/command/mission traces; no threshold tuning. | Phase A |
| B-001 | RECORD_ONLY | Isolated world-source stale evidence remains open. | Unknown temporal boundary outcome. | Source-only fault SITL. | Phase B |
| C-001 | RECORD_ONLY | PX4 Hold callback/status ordering and emergency runtime path remain separate debts. | Authority handover uncertainty. | Pinned dependency trace, deterministic protocol tests, focused SITL. | Phase C |
