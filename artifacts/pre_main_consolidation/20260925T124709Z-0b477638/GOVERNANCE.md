# PR #2 review and integration governance

The repository currently has one human collaborator (`TanDatEmb`). No independent human reviewer is available in the observed collaborator list. Do not claim an independent human approval.

For this intentionally single-maintainer repository, use a temporary documented integration rule:

1. Keep the change in a PR; do not push directly to `main`.
2. Require an independent automated Codex review of the exact latest PR head.
3. Require zero unresolved P0/P1 findings.
4. Require the available GitHub CI checks green and retain the authoritative local ROS 2 Jazzy Release/test evidence while hosted product-build CI is unavailable.
5. Require the repository owner to make an explicit merge decision after reviewing the diff and evidence.

GitHub currently reports `main` as unprotected. This campaign does not change branch-protection settings. The documented process is therefore a governance rule, not a claim that GitHub technically enforces required reviews or checks. Before beta, configure branch protection when repository permissions and policy permit.
