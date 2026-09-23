# Evidence index and proof levels

- **Pinned source:** product `7da3e97cb399c2e39d62cfe60213a45e8a92300e`; guarded instrumentation `d926013eff4d46af9756427dd53a8028710f2765`. See `PROVENANCE.md`, `SOURCE_FACTS.md` for exact source anchors.
- **Existing diagnostic runtime:** `artifacts/experiment_runtime_observability/20260923T041656Z-7da3e97c/EXPERIMENT_VERDICT.md`, O1–O4 CSVs and raw normalized per-run files in separate instrumentation worktree. Raw files are ignored there, not copied into this Git branch. `INPUT_SHA256.csv` records the exact 42 normalized audit/integrity/raw-protocol files consumed or inspected; existing experiment SHA index provides capture provenance.
- **Model proof:** `tools/shadow_reducer/model.py`, `tests/shadow_reducer/test_model.py`, `ADVERSARIAL_MODEL_RESULTS.md`. Synthetic event order, no flight qualification.
- **Differential replay:** `tools/shadow_reducer/replay.py`, `REPLAY_RESULTS.json`, `DIFFERENTIAL_RESULTS.md`. Observer-order and trace-integrity limitations are explicit.
- **State inventory:** pinned 239-field prior CSV plus `FIELD_CLUSTER_MAP.csv`; `tools/shadow_reducer/conservation.py` asserts exact key coverage and emits `STATE_CONSERVATION_MATRIX.csv`. Dispositions are target migration decisions, not product edits.
- **Static non-authority:** `tools/shadow_reducer/static_check.py`, `STATIC_NON_AUTHORITY_CHECK.md`; lexical scoped check, not formal proof.
- **No SITL/flight qualification performed in this branch.** Runtime evidence was reused, not regenerated. No performance threshold or safety gate was changed.

Reproduce:

```bash
python3 -m unittest discover -s tests/shadow_reducer -q
python3 -m tools.shadow_reducer.static_check
python3 -m tools.shadow_reducer.conservation artifacts/shadow_reducer/20260923T074145Z-7da3e97c/STATE_CONSERVATION_MATRIX.csv
python3 -m tools.shadow_reducer.cli --normalized-root /home/letandat/Dev/uav-navigation-experiment-runtime-observability-20260923/.artifacts/experiment_runtime_observability/20260923T041656Z-7da3e97c --output artifacts/shadow_reducer/20260923T074145Z-7da3e97c/REPLAY_RESULTS.json
```
