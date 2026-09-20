# Runtime safety ledger compatibility pointer

The authoritative routine working set is now
[`docs/safety/runtime_safety_current.md`](../safety/runtime_safety_current.md).

Use the decision index for targeted history:
[`docs/safety/runtime_safety_index.md`](../safety/runtime_safety_index.md).
The complete pre-migration ledger is preserved byte-for-byte at
[`docs/safety/archive/runtime_safety_legacy_full.md`](../safety/archive/runtime_safety_legacy_full.md).

This compatibility path is intentionally not a second ledger. Do not add new
decisions, gates, or bypasses here. The repository working contract and all
safety-relevant changes must use `runtime_safety_current.md`; retrieve the
archive only for provenance, regression lineage, rejected/reverted designs,
temporary-bypass history, or an audit that requires the original evidence.
