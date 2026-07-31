# ikd-Tree benchmark

This standalone CMake harness compiles the pinned upstream `ikd_Tree.cpp` and
calls `Build`, `Nearest_Search`, `Add_Points`, and `Delete_Point_Boxes` directly.
It does not benchmark the removed/reference voxel implementation.

```bash
tools/benchmarks/run_ikd_tree_benchmark.sh .artifacts/benchmarks/ikd_tree.json
```

The companion report gates measured query-time scaling across at least three
sizes, but calls its result empirical evidence rather than a mathematical proof.
Keep the raw measurement JSON, report JSON, host/CPU governor details, compiler,
PCL version, and exact command with any acceptance report. No benchmark result
has been fabricated or committed in this repository.
