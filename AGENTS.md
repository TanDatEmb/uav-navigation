# Repository working contract (branch `rebuild/v2`)

This branch rebuilds the navigation stack against a new architecture. The
`main` branch is a **reference only**. Do not run it, fix it or keep it
working. Read old code with `git show main:<path>`, and reuse ideas or
algorithms deliberately.

Local workspace rules in `CLAUDE.local.md` take precedence over this file:

- work only in this tree;
- no worktrees or clones;
- one active work package at a time;
- `git add <path>` only.

## 1. The design is the map

- The sources of truth are the design spec
  [docs/architecture/SYSTEM_DESIGN.md](docs/architecture/SYSTEM_DESIGN.md) and
  the decision log [docs/architecture/DECISIONS.md](docs/architecture/DECISIONS.md).
  Progress is tracked in [docs/TRACEABILITY.md](docs/TRACEABILITY.md).
  The decision log holds three kinds of entry:
  - **AGREED decisions (D\*)**, which are binding;
  - **OPEN questions (O\*)**, which are not yet decided;
  - **verified facts (F\*)**, each with evidence.
- Every work package names the design section it implements, and its commits
  reference that section.
- Implement what the design says. Small local choices need no approval, for
  example private helper names or the internal layout of a function.
- **Stop on design deviation.** Stop, record the deviation in the decision
  log, and wait for the owner's decision before continuing when the work would
  need any of the following:
  - a new or changed interface;
  - a change in ownership of a decision;
  - a new state or transition;
  - a new thread or lock;
  - a change in timing or budget;
  - a change in safety behaviour;
  - anything the design does not cover.

  Update the design first, then continue.
- If several deviations pile up, or the design looks wrong as a whole, stop
  implementation and revise the overall design. Patching around it is not
  acceptable.
- Never build on an OPEN question as if it were decided.

## 2. Structural rules (decision D12)

These rules apply to all new code. Review rejects code that breaks them.

1. **Explicit state machines with a single writer.** Each component has one
   state `enum`, one transition table, and one function
   `transition(event) -> (state, effects)`. Do not represent state as loose
   boolean flags, and never require "set these flags in this order".
2. **Typed results that fail closed.** A decision returns
   `Result<T, Reason>`, where `Reason` is an exhaustive enum. Do not encode
   several meanings in a number or a bool. Missing or ambiguous input means
   rejection, never pass.
3. **One structured event per decision.** Every transition, rejection, cancel
   and handover emits an event at the decision point. The event carries:
   component, event, state before and after, reason, and the values that
   drove the decision. Use one logger; no raw `std::cout` or `printf`. Logs
   are the main debugging tool for the whole system, so a failure without a
   reason is a defect.
4. **One typed config schema.** Each value has a unit and a single source, and
   is validated once at startup. There are no hidden defaults that differ from
   YAML, and no literals standing in for tunables.
5. **One time snapshot per cycle.** Clock domains are explicit: sensor, ROS
   and steady. Absolute time is signed integer nanoseconds. Measure budgets
   without lock waits.
6. **No publishing and no callbacks while holding a lock.** Thread boundaries
   are explicit. Every worker failure has a defined recovery or fail-closed
   path; a worker never dies silently.
7. **Layer boundaries are enforced by types.** Each layer owns its decisions.
   A layer must not mutate another layer's state, and must not hard-code
   another layer's facts, such as the airframe or the policy.
8. **Test and experiment code stays out of the product binary.** Put it
   behind a compile-time flag or keep it in test targets.

## 3. Safety behaviour that stays non-negotiable

These rules are recorded in the design, not in the old ledger:

- Fail closed on missing or stale time, frame, identity, epoch, world or
  certificate data.
- In the default profile, BACKUP is known-free. The agile profile is
  owner-approved, risk-accepted and explicitly logged (decision D2).
- Never hand over silently. Every handover to PX4 and every abort carries a
  reason code in the event log (decision D11).
- No hidden bypass. A temporary shortcut has to meet three conditions:
  - it is visible in config;
  - it is logged;
  - it is listed in the decision log.

The old safety ledger on `main` and
`tools/validate_runtime_safety_ledger.py` describe `main`. They are not
maintained on this branch.

## 4. Testing for the beta (decision D14)

- Unit tests are required for every state machine and every safety decision
  function.
- SITL smoke for each vertical slice, plus the beta gate:
  - zero collisions on the fixed scenes;
  - missions complete at 1–5 m/s;
  - no silent handover.
- Distribution and qualification measurements (the full O4 table) come after
  the beta. Do not spend effort qualifying or tuning on the old baseline.

## 5. Resource limits on this machine

The machine has 15 GB RAM. Run static analyzers and builds with at most
`-j3`, under `nice`. Run at most 3 subagents in parallel. Subagents write
results progressively, so that interrupted work is not lost.

## 6. Deleting and moving files

These rules come from what happened during S0.

- Before deleting or moving a tracked path, run a **reference sweep** across all file types, not only `package.xml`: CMake, YAML, launch files, tests, scripts and docs. Grep for both the path and the package name. Resolve every hit, or list it with a reason, before the change.
- Move a package in the slice that rebuilds it. Its first rebuild after the move uses `colcon build --cmake-clean-cache`, because the shared `build/` caches hold the old source path.
- The owner allows agents to delete tracked files themselves (D30). Delete with explicit `git rm` paths after the reference sweep. If the permission system blocks a deletion, do not retry or work around it: write a script that runs one `git rm` command per path, dry-run it (`-n`), and hand it to the owner.
