# ikd-Tree benchmark procedure

The benchmark builds the pinned upstream `ikd_Tree.cpp`, then directly invokes
`Build`, `Nearest_Search`, `Add_Points`, and `Delete_Point_Boxes`; no voxel-map
reference implementation is involved. Run:

```bash
bash tools/benchmarks/run_ikd_tree_benchmark.sh .artifacts/benchmarks/ikd_tree.json
```

It writes a raw measurement and an analysis report. The report requires at least
three increasing point counts and applies an empirical query-scaling gate. A
pass is timing evidence for this host/build/command, not a proof of asymptotic
complexity. Record CPU model, governor, RAM pressure, compiler, PCL version,
source revision, command and SHA-256 hashes beside any M1 result.

On this workspace, a smoke run with `--sizes 1000,2000,4000 --queries 200`
passed the configured empirical gate. Raw SHA-256:
`5af2ee0260ba2fbc081bdecd331bc3d07c676ef36911d31e9ad1b7d241d99100`;
analysis SHA-256:
`46a73cd9c828dacbb3c8d2125d8cf9f3077688b30095c40107ec665faf4642a2`.
These `/tmp` artifacts are not committed and are not Raspberry Pi 5 performance
evidence.
