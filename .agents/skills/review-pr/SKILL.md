---
name: review-pr
description: Substance-focused review of a uav-navigation pull request — correctness, flight-safety contract, architecture fit. Produces a debrief and a draft review comment.
argument-hint: "<PR number or URL>"
allowed-tools: Bash Read Glob Grep Agent
---

# Pull Request Review

Brief the user and draft a review comment they may post. Substance only: no
commit-message, merge-strategy, or formatting review. Scale effort with lines
changed and how flight-critical the code is; fan out subagents per subsystem
for large diffs.

## Gather (parallel)

- `gh pr view <PR> --json number,title,body,author,baseRefName,files,reviews`
- `gh pr diff <PR>` (on HTTP 406 use `gh api repos/{owner}/{repo}/pulls/<PR>/files --paginate`)
- `gh pr checks <PR>` (exit 8 = pending)
- existing inline and conversation comments via `gh api`

Read linked issues. Full post-change file: `git fetch origin pull/<PR>/head`,
`git show FETCH_HEAD:<path>`. **Always read
`docs/safety/runtime_safety_current.md` before judging any diff under `src/`**
and cite its invariants by number in findings; read `docs/architecture/SYSTEM_DESIGN.md` and `docs/ROADMAP.md`.

## Review

Never judge a hunk without its enclosing function and callers.

- **Merit:** real problem or a patch over one? Right layer? Simpler option?
- **Math/physics:** re-derive; check units, frames, signs, singularities,
  discretization, NaN/float precision, time representation, against the
  conventions in `docs/architecture/SYSTEM_DESIGN.md` and
  its frame/time sections and current source contracts.
- **Safety contract:** missing/ambiguous time, frame, identity, epoch, or
  freshness converted to a default? `UNKNOWN`/`OUT_OF_MAP` made traversable?
  Candidate committed without atomic goal/localization/lease/latest-world/
  corridor/swept checks? Newer world not invalidating old candidate? Finite,
  continuous P/V/A to External Mode? Fail-closed paths intact?
- **Hidden bypasses:** relaxed gate, diagnostic-only flag leaking to product,
  fallback-only path, test-specific branch, without a ledger record.
- **Ownership:** logic lives in the layer that owns the data, per the
  ownership chain in the safety contract's purpose section and
  `docs/architecture/SYSTEM_DESIGN.md`; check current source ownership and unresolved debt.
- **Runtime:** latency tails, allocations/locks in real-time paths, budgets.
- **Evidence:** thresholds tuned from one SITL run? Tests that prove the
  contract, not just the happy path? "QUALIFIED" claimed without evidence?
- **Compatibility:** msg/param/config contract changes (the canonical design configuration section); every new parameter is a burden.
- Behavior changes mixed with refactors/observability in one PR: flag.

## Deliver (post nothing)

1. **Debrief:** findings first, ordered blocker / concern / nit, with
   `file:line`; framing only as needed (is the problem real, risk, CI state).
2. **Draft comment** in a fenced block, first line
   `**<assistant> review on behalf of @<login>**` (login: `gh api user --jq .login`).
   Terse, actionable, each with a concrete code path or derivation. If you
   cannot demonstrate a flaw, stay silent. No praise or filler; do not repeat
   others' feedback.

Post only when asked: `gh pr comment <PR> --body "$(cat <<'EOF' ... EOF)"`.
