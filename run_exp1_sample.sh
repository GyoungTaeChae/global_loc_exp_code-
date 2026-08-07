#!/usr/bin/env bash
# Experiment 1: trajectory-wide sample of kaist002 against a kaist001 map, with
# the three polish methods at their library default-ish settings plus the
# no-polish baseline. Purpose is twofold: a first ranking, and the ground-truth
# probe columns that say how far the two sessions' ground truth disagree.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HERE/out" "$HERE/logs"

"$HERE/build/polish_compare" \
  stride=60 threads=16 progress=10 \
  out="$HERE/out/exp1_sample.csv" \
  tag="[1/1]" \
  config=none \
  config=icp,dist=2.0,iter=20 \
  config=gicp,dist=2.0,iter=20 \
  config=ndt,res=2.0,iter=30 \
  2>&1 | tee "$HERE/logs/exp1_sample.log"
