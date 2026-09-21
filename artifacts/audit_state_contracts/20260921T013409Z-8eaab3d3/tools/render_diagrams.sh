#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
command -v dot >/dev/null 2>&1 || { echo 'Graphviz dot is required' >&2; exit 2; }
mkdir -p "$HERE/diagrams/svg"
for src in "$HERE"/diagrams/src/*.dot; do
  base="$(basename "$src" .dot)"
  dot -Tsvg "$src" -o "$HERE/diagrams/svg/$base.svg"
done
