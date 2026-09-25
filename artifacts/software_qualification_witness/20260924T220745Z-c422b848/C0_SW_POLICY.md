# C0-SW policy, version C0_SW_V1

**Provenance:** user-approved software-first C0-SW decision in the 2026-09-25 task. This is a scoped software qualification witness contract, not an approved integrated flight-performance threshold policy. Scope metadata is first-class: `qualification_scope=C0_SW`, `qualification_policy_version=C0_SW_V1`, and the exact policy provenance string must be present. The runner rejects a request for this scope unless tracking experiment is explicitly off.

**Scope:** C0 requested cruise speed 1–5 m/s; product mission, desired intent, execution authority, planner lifecycle, world/reference identity used by software, adapter admission and rejection, state/command temporal safety, recovery ownership, fault handling, configuration truth, and evidence integrity. The existing 100 ms command lease, 200 ms adapter state boundary, 500 ms world freshness and 0.750 m terminal anchor gate remain in force. No new pass threshold for tracking or measured motion is created.

**Eligibility:** exact producer-owned lifecycle and reference identities, no required unresolved/conflicting transaction, no missing/conflicting required reference, complete loss-accounted recorder, truthful tracking-off configuration at Core and adapter, no suppression or planner fault injection, and an attributable software outcome. Eligibility is scoped to C0-SW; an eligible software FAIL remains a FAIL. An integrated runner FAIL is retained independently. `FAULT_HANDLING` needs its own focused suite and is not converted to PASS by a nominal run.

**Deferred:** C0-IFP absolute tracking, PX4 oscillation, terminal physical stopping quality, LIO truth accuracy, frame/time truth budget, Gazebo/custom sensor fidelity and real-vehicle tuning. See `SCOPE_AND_DEFERRED_DOMAINS.md`.

The policy is implemented in `tools/runtime/evaluation.py::evaluate_software_qualification`, reported by `tools/runtime/report.py`, and selected by `tools/runtime/runner.py --qualification-scope C0_SW`. There is no evaluator fallback to this policy for legacy sessions.
