#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
src="$root/diagrams/src"
dst="$root/diagrams/svg"
command -v dot >/dev/null
mkdir -p "$dst"
for file in "$src"/*.dot; do
  name="$(basename "$file" .dot)"
  dot -Tsvg "$file" -o "$dst/$name.svg"
done
printf 'Rendered %s DOT files with %s\n' "$(find "$src" -maxdepth 1 -name '*.dot' | wc -l)" "$(dot -V 2>&1)"
