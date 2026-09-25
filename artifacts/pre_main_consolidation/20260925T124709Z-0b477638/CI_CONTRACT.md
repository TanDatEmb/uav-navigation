# Canonical CI contract

`.github/workflows/ci.yml` adds stable GitHub checks:

- `uav-nav / static-contract`: PR patch whitespace, permanent authority/contract guards, and the runtime safety ledger.
- `uav-nav / python`: the runtime/evaluator unittest suite, including C0-SW witness/scope, report semantics, evidence writer, World transaction reducer, and explicit scope-guard regression tests.

The workflow deliberately excludes branch-delta guards requiring a caller-provided historical baseline. Patch whitespace checks cover source, tooling, configuration, and documentation changes outside the archived `artifacts/` tree; the inherited evidence archive includes CRLF CSVs that Git reports as trailing whitespace, so it is not used as a source-format gate.

Hosted CI status: not yet observed before the workflow push. The stable job names become GitHub status checks when the workflow runs on the updated PR. Local validation before push passed all nine permanent guards and the 460-test Python suite (one expected skip).

## Product build limitation

No self-hosted GitHub Actions runner is registered for this repository. The authoritative Release/PRODUCT_REQUIRED build depends on the controlled ROS 2 Jazzy and workspace dependency environment; a GitHub-hosted approximation would not establish equivalent provenance. Therefore full ROS product build/test automation is classified `CI_BUILD_ENVIRONMENT_REQUIRED`, not PASS. The authoritative local gate remains required for this PR. Before beta, register and maintain a reproducible ROS 2 Jazzy product runner, then add its stable required check.

The pinned live-FMU integration test remains environment-gated and is not required by ordinary CI.
