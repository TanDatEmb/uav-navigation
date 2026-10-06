---
name: design-check
description: Use before implementing or reviewing any work package on rebuild/v2, and whenever a change might touch an interface, ownership, state, thread, timing, budget or safety behaviour — checks the change against the design map and records deviations
allowed-tools: Bash Read Glob Grep Edit
---

# Design Check

[SYSTEM_DESIGN.md](../../../docs/architecture/SYSTEM_DESIGN.md) is the map.
[DECISIONS.md](../../../docs/architecture/DECISIONS.md) records why each part
of it was chosen. [TRACEABILITY.md](../../../docs/TRACEABILITY.md) tracks
progress. The working rules are in AGENTS.md.

## Before implementing

1. Name the spec section (§) that the work package implements, and read it in
   full, together with the D\* decisions it cites.
2. List what the change will add or alter:
   - interfaces and messages;
   - states and transitions;
   - owners of decisions;
   - threads and locks;
   - timing and budgets;
   - safety behaviour.

   Any item the spec does not already describe is a **deviation**.
3. Check the structural rules (AGENTS.md §2) and the non-negotiable safety
   behaviour (AGENTS.md §3, spec §0) against the plan.

## When there is a deviation

- Stop. Do not implement around it.
- Add an O\* entry to DECISIONS.md. It must contain: what the spec says, what
  the implementation needs, why, and the options.
- Add a row under "Lệch thiết kế đang mở" in TRACEABILITY.md.
- Ask the owner. Once the owner decides, update the spec and DECISIONS.md
  (mark the entry AGREED), then continue.
- If three or more deviations are open, or any deviation touches spec §2,
  stop and revise the overall design before continuing.

## Three-level review of the finished change

1. Ownership and the end-to-end flow against the spec sections.
2. Source details:
   - units and clock domains;
   - typed reasons;
   - an event at each decision point;
   - config tier (a/b/c);
   - tests for state machines and safety decisions.
3. Adversarial pass:
   - hidden bypasses;
   - flag-encoded state;
   - fail-open paths;
   - locks held while publishing;
   - latency tails;
   - regressions in another layer.

## Finish

- Update the TRACEABILITY.md row: set the status, and give evidence (test
  name, commit or event log).
- Run:

  ```bash
  python3 tools/check_documentation.py . docs
  git diff --check
  ```

  Report the output verbatim.
