#!/usr/bin/env bash
# Experiment 2c: is the iteration count still the binding constraint?
#
# exp2b showed the iteration cap, not the distance/resolution knob, is what each
# method was running into: going 30 -> 60 gained ICP 2.5%p, GICP 0.9%p and NDT
# 5.7%p, while pushing dist/res one step further gained under 1%p for all three.
# So all three are compared at 120 iterations here; if that matches 60, the
# methods are converged and the ranking is not an artefact of the cap.
#
# Same frames as exp2 and exp2b (start=5, stride=73).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HERE/out" "$HERE/logs"

"$HERE/build/polish_compare" \
  start=5 stride=73 threads=16 progress=10 \
  out="$HERE/out/exp2c_iterations.csv" \
  tag="[iterations]" \
  config=icp,dist=4.0,iter=120 \
  config=gicp,dist=4.0,iter=120 \
  config=ndt,res=3.0,iter=120 \
  2>&1 | tee "$HERE/logs/exp2c_iterations.log"
