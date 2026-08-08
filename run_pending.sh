#!/usr/bin/env bash
# Runs the queued experiments back to back, announcing which is which. Each
# harness invocation also prints the same [i/n] tag on its own progress lines.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS=(run_exp3_plugin_default.sh run_exp4_polish_points.sh)

for index in "${!SCRIPTS[@]}"; do
  echo "[$((index + 1))/${#SCRIPTS[@]}] ${SCRIPTS[$index]}"
  "$HERE/${SCRIPTS[$index]}"
done
echo "all queued experiments done"
