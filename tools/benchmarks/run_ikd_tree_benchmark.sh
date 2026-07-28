#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <output-json> [ikd_tree_benchmark arguments...]" >&2
  exit 64
fi

output=$1
shift
build_dir=build/tools/ikd_tree_benchmark
cmake -S tools/benchmarks -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" --parallel
mkdir -p "$(dirname "$output")"
"$build_dir/ikd_tree_benchmark" "$@" > "$output"
python3 tools/benchmarks/analyze_ikd_tree_benchmark.py "$output" \
  --output "${output%.json}.report.json"
