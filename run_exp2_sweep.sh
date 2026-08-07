#!/usr/bin/env bash
# Experiment 2: parameter sweep, so the final comparison is not one tuned method
# against two defaults. Each method gets its main knob swept over the same
# frames, and every configuration sees the same RANSAC candidate set.
#
# Frames are offset from the final run's grid (start=5, stride=73 vs start=0,
# stride=10) so tuning and the final measurement mostly do not share frames.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HERE/out" "$HERE/logs"

"$HERE/build/polish_compare" \
  start=5 stride=73 threads=16 progress=10 \
  out="$HERE/out/exp2_sweep.csv" \
  tag="[sweep]" \
  config=none \
  config=icp,dist=1.0,iter=30 \
  config=icp,dist=2.0,iter=30 \
  config=icp,dist=4.0,iter=30 \
  config=gicp,dist=1.0,iter=30 \
  config=gicp,dist=2.0,iter=30 \
  config=gicp,dist=4.0,iter=30 \
  config=ndt,res=1.0,iter=30 \
  config=ndt,res=2.0,iter=30 \
  config=ndt,res=3.0,iter=30 \
  2>&1 | tee "$HERE/logs/exp2_sweep.log"
