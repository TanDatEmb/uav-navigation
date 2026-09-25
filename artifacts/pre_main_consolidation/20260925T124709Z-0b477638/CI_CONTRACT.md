# Canonical CI contract

`.github/workflows/ci.yml` adds stable GitHub checks:

- `uav-nav / static-contract`: PR patch whitespace, permanent authority/contract guards, and the runtime safety ledger.
- `uav-nav / python`: the runtime/evaluator unittest suite, including C0-SW witness/scope, report semantics, evidence writer, World transaction reducer, and explicit scope-guard regression tests.

The workflow deliberately excludes branch-delta guards requiring a caller-provided historical baseline. Patch whitespace checks cover source, tooling, configuration, and documentation changes outside the archived `artifacts/` tree; the inherited evidence archive includes CRLF CSVs that Git reports as trailing whitespace, so it is not used as a source-format gate.

Hosted CI attempt for PR head `872c708b0e188466e6a7450f1e8ec13f6727cf40`: workflow run `36160009439` created both stable checks, but neither job started. GitHub annotated both with `The job was not started because your account is locked due to a billing issue.` Therefore hosted static and Python checks are infrastructure-blocked, not test-passing. The equivalent local permanent guards and Python suite passed in the authoritative gate; retry hosted checks after account billing is restored.

## Product build limitation

No self-hosted GitHub Actions runner is registered for this repository. The authoritative Release/PRODUCT_REQUIRED build depends on the controlled ROS 2 Jazzy and workspace dependency environment; a GitHub-hosted approximation would not establish equivalent provenance. Therefore full ROS product build/test automation is classified `CI_BUILD_ENVIRONMENT_REQUIRED`, not PASS. The authoritative local gate remains required for this PR. Before beta, register and maintain a reproducible ROS 2 Jazzy product runner, then add its stable required check.

The pinned live-FMU integration test remains environment-gated and is not required by ordinary CI.
