<!-- Keep these two lines; CI (check_change.py --body-file) reads them. Rules: docs/CHANGE_CONTROL.md -->
Arch-Level: 0
Docs-Impact: none

## Task / requirement
<!-- Plan task or TRACEABILITY row; spec section (§) and decision (D*) it implements -->

## What changed and why

## Architecture impact
- [ ] Level 0: no contract, invariant, or state-model change (default)
- [ ] Level 1: interface / observable behaviour changed; spec and TRACEABILITY updated in this PR (or `pending:` with reason)
- [ ] Level 2 / 3: Architecture Review `AR<n>` linked (Level 3: also `Decision: D<n>` AGREED)
- Invariants touched (see `check_change.py` output): <!-- none | INV ids -->
- State-model impact: flags / states / transitions added: <!-- none | list; each new state needs Level >= 2 -->
- Workarounds added: <!-- none | description and root cause -->

## Evidence
<!-- Tests run, commit SHA, SITL `run:<id>` if any. A passing unit test is COMPONENT_VERIFIED, not INTEGRATED or QUALIFIED. -->

## Known limitations / deviations
<!-- DV* introduced or closed; known limitations -->

## Reviewer checklist (what CI cannot judge)
- [ ] The declared level matches the change
- [ ] No logic duplicated from another subsystem; no responsibility leaking across a layer boundary
- [ ] No new special-case path without a place in the model
