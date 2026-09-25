# Main integration diff

Fresh comparison target is `origin/main` SHA `287b7b84cf4311e31d52ca04796223cc4efc1bc5`; it is an ancestor of the candidate, so no rebase/merge was required. At this comparison, candidate pre-evidence-doc commit is `42007f55505416cd63a591d5d168381956bcd5b3`. The full architecture integration delta from current main is 519 files, 52,189 insertions and 27,836 deletions, including 96 `src/` files from the previously approved architecture stack. The current campaign delta from its base is limited to evaluator/report tooling, tests, gate scripts, and evidence artifacts; `src/` is byte-identical to campaign base.

The campaign delta adds independent C0-SW/C0-IFP result objects and golden coverage, makes exact witnessed fail-closed stops assessable under C0-SW without weakening evidence requirements, includes omitted first-party test packages, rebases static scope guards to the frozen incoming architecture, and adds one authoritative pre-main command. It does not change planner, World, MissionProgress, DesiredPlanningIntent, ExecutionAuthority, PX4 adapter source, control contract, or safety thresholds.

One canonical integration PR targets `main`; audit/experiment branches are not merged separately.
