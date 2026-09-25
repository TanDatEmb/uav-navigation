# Scope guard migration

The two qualification scope scripts are pre-merge branch-delta evidence tools. They now accept `--base <commit>` explicitly, verify object availability and ancestry, and fail with typed messages rather than relying on a development-history constant.

Current full-history checkout historical check:

```text
check_qualification_evidence_scope.py --base 0b477638d21ce60cdb42ed85fb7c2d568bf500ed: PASS
check_software_qualification_scope.py --base 0b477638d21ce60cdb42ed85fb7c2d568bf500ed: PASS
```

These commands are recorded as pre-merge evidence only. Neither check is invoked by the canonical post-merge `pre_main_gate.py`. The permanent semantic guards continue to run there.

Automated regression coverage verifies a valid base, protected-source changes, a missing Git object, malformed and non-ancestor revisions, omitted `--base`, and gate-registry operation in a one-commit temporary checkout without the historical object.
