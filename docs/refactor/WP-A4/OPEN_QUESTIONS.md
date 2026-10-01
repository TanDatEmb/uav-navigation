# WP-A4 — Open questions / baseline discrepancies

## OQ-01 — Architecture-review inputs absent at the required baseline

- **Question:** Should WP-A4 be re-run after the architecture-review package is
  landed on `main`, or is the baseline source-only audit below the intended
  oracle?
- **Evidence:** `7e0b850` contains `AGENTS.md`, `docs/safety/runtime_safety_current.md`
  and `docs/safety/runtime_safety_index.md` (removed at the 2026-10-01 baseline), but does not contain
  `docs/refactor/ARCHITECTURE_REVIEW.md`, `docs/refactor/adr/ADR-013..016`, or
  `docs/refactor/risk_register_20260928.md`. The required context was read
  read-only from `origin/refactor/WP-D0`; it is not copied into this baseline.
- **Impact:** Architecture/risk claims remain context-only and are not treated
  as baseline source evidence. Source and checker evidence is still pinned to
  `7e0b850`.
- **Options:** (A) accept this source-pinned audit and review the missing
  documents separately; (B) rebase/re-run WP-A4 after those documents land.

## OQ-02 — `planner_fsm.hpp` count in the prompt does not match baseline

- **Question:** What is the intended denominator for the historical “38/38
  functions” statement?
- **Evidence:** The baseline file contains 40 free decision helpers plus five
  public `PendingGoalHandoffOwner` methods in the decision inventory. The
  private `goalMessageNewer`/`goalSnapshot` storage helpers are excluded from
  the decision denominator. `predicates.csv` and the checker map 45 entries;
  the prompt's stale 38 is not used.
- **Impact:** The report states the actual denominator and does not silently
  omit functions to manufacture a 38/38 result.
- **Options:** (A) approve the baseline's actual helper set; (B) define a
  narrower denominator for a follow-up oracle revision.

## OQ-03 — Runtime reachability of lifecycle combinations

- **Question:** Are the statically reachable enum combinations below also
  reachable under a production ROS/PX4 event trace?
- **Evidence:** `lifecycle_enums.md` distinguishes combinations constructed by
  direct setters from combinations observed in tests. No live PX4 trace was
  run for this read-only WP.
- **Impact:** Unobserved combinations remain `CONDITIONAL`/`NOT_MEASURED`; they
  are not called impossible solely from component tests.

## OQ-04 — Remote branch history reconciliation (RESOLVED locally)

- **Question:** How should the existing PR #5 branch be reconciled with the
  already-created local WP-A4 commits while preserving the no-force-push rule?
- **Evidence:** The initial divergence was `HEAD=a9f0a00` versus
  `origin/refactor/WP-A4=102964e0`, both from `7e0b850`. A non-destructive
  same-branch merge was completed as `867337c6`; the R1 artifacts were kept on
  conflicts and no `src/` file changed.
- **Impact:** The next push to `origin/refactor/WP-A4` is fast-forwardable;
  force-push, reset, rewrite, cherry-pick, and merge from `codex/*`/`feat/*`
  were not used. This OQ is closed locally; remote push/PR state is verified in
  the final handoff.

## OQ-05 — Source return denominator differs from the earlier review note

- **Question:** Should nested lambda returns lexically inside a scoped function
  count toward the function's source oracle?
- **Evidence:** The R1 checker counts every `return` token in each target
  function's balanced source span, including executable nested lambdas. This
  yields `runCycle=75` and `194` returns across all 19 target functions; the
  earlier review note said 74 for `runCycle`.
- **Impact:** The checker is fail-closed on source coverage and does not drop a
  nested return to manufacture the older denominator. Runtime reachability is
  still `NOT_MEASURED`.
- **Options:** (A) retain lexical-span coverage as the safer oracle; (B) define
  a C++ AST ownership rule in a follow-up if reducer extraction excludes nested
  lambda bodies.
