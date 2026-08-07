#!/usr/bin/env bash
# Experiment 2b: extend the sweep past the edge of exp2.
#
# In exp2 both ICP and NDT were still improving at the largest value swept
# (icp dist=4, ndt res=3), while GICP had flattened. Declaring a winner there
# would compare GICP at its optimum against two methods pinned at a boundary.
# This run pushes each method's main knob one or two steps further and checks
# the iteration count on each method's current best.
#
# Same frames as exp2 (start=5, stride=73) so the two sweeps are comparable.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HERE/out" "$HERE/logs"

"$HERE/build/polish_compare" \
  start=5 stride=73 threads=16 progress=10 \
  out="$HERE/out/exp2b_sweep_edge.csv" \
  tag="[sweep-edge]" \
  config=icp,dist=6.0,iter=30 \
  config=icp,dist=4.0,iter=60 \
  config=gicp,dist=6.0,iter=30 \
  config=gicp,dist=4.0,iter=60 \
  config=ndt,res=4.0,iter=30 \
  config=ndt,res=3.0,iter=60 \
  2>&1 | tee "$HERE/logs/exp2b_sweep_edge.log"
