#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <dataset-directory> [ros2 bag play arguments...]" >&2
  exit 64
fi

dataset=$1
shift
python3 tools/datasets/verify_mid360_dataset.py "$dataset"
exec ros2 bag play "$dataset" --clock "$@"
