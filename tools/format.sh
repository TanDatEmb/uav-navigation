#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WORKSPACE_DIR}"

echo "[format] Running clang-format on C++ sources (skipping px4_msgs submodule)..."
find . \
    -path './src/px4_msgs' -prune -o \
    -path './build' -prune -o \
    -path './install' -prune -o \
    -path './log' -prune -o \
    \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) \
    -print0 | xargs -0 -r clang-format -i -style=file

echo "[format] Done."
