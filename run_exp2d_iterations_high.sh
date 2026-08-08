#!/usr/bin/env bash
# Experiment 2d: the last fairness check on the iteration cap.
#
# 30 -> 60 -> 120 improved all three methods and never flattened, and NDT gained
# fastest at every step (+5.7, +3.3) while GICP gained least (+0.9, +0.8). If the
# ranking is going to change with more iterations it will show at 240; if the
# gaps hold, the cap is no longer what separates the methods.
#
# Same frames as exp2, 2b and 2c (start=5, stride=73).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HERE/out" "$HERE/logs"

"$HERE/build/polish_compare" \
  start=5 stride=73 threads=16 progress=10 \
  out="$HERE/out/exp2d_iterations_high.csv" \
  tag="[iterations-240]" \
  config=icp,dist=4.0,iter=240 \
  config=gicp,dist=4.0,iter=240 \
  config=ndt,res=3.0,iter=240 \
  2>&1 | tee "$HERE/logs/exp2d_iterations_high.log"
