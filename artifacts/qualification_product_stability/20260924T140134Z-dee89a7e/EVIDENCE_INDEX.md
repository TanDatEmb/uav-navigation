# Evidence index

## Source and raw predecessor evidence

Product behavior in the ten-run predecessor cohort: `96ed8d089e576c505abdc15500746e9b5be10e50`. Audit branch base: `dee89a7ee7fff0199e2ca02b3b1166b9c8285fdb`. Exact PX4 checkout/binary are in `BASE_PROVENANCE.md`. The three raw sessions below are retained outside Git; paths, byte sizes, and SHA256 values identify the analyzed bytes.

| Session | File | Bytes | SHA256 |
|---|---|---:|---|
| `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260924T094122-193534` | `samples.jsonl` | 73714103 | `9bb7ffcdfac6f9076f00ac21aa4f83355b5adee85ee08150e8f96cefaba3261a` |
| same | `metadata.json` | 252871 | `65648f0d5a6aec40d8fffd7221722cfee01f807a7e3af504c576a56ba6f727e5` |
| same | `report.json` | 30239309 | `4d63700364c3be55c6591b763c7708c58c96dd646d318924230698741000836f` |
| `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260924T094341-193534` | `samples.jsonl` | 68618818 | `82bf056463fd7b71da247a58133e84a78621a8586cc4b9a3b5c6aeddee5052ca` |
| same | `metadata.json` | 252871 | `0af5a6509dacb51080f0899b1950437c81db1cf89f61f22f1ff030c4afdf4726` |
| same | `report.json` | 28192495 | `33ba474c69153a5a6eab4e784fc8fde5dac4fb8d346d8cdfaf702a6a27d19fe7` |
| `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260924T093208-186236` | `samples.jsonl` | 60817395 | `859ed90c325b5b244f1f3f0815a564977451a899be0e118f831d711f270d9869` |
| same | `metadata.json` | 252870 | `71411b3932a97a36cf1ed3ad458cebfe068748795d7f03be32e37edbfd1ced3b` |
| same | `report.json` | 19430245 | `10a2c59d6695f8dd883c61cd9330b68726fc2e244c508b8c164e2b31ff587a60` |

`TERMINAL_RESULTS.csv` indexes all ten predecessor runs. `terminal_run_1.json`/`terminal_run_2.json` are reproducible reductions from `tools/runtime/analyze_terminal_recovery.py`; `historical_temporal_layers.json` is produced by `tools/runtime/analyze_temporal_layers.py`. `TIMING_DISTRIBUTIONS.csv` retains transport summary per run. They are diagnostic reductions, not product or evaluator verdict owners.

## New branch evidence

Pending focused diagnostic SITL validation and final build/test logs. Their exact paths and hashes will be added after runs finish.
