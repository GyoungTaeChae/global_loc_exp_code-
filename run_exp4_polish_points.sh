#!/usr/bin/env bash
# Experiment 4: how many scan points does polish actually need?
#
# Polish currently gets the whole downsampled scan (query_voxel=0.5, median
# ~11 k points on MulRan) and spends 3-12 s on ten candidates. If accuracy
# survives a thinner scan, the method becomes deployable; if it collapses, the
# cost is irreducible. Every method gets the same point counts so the ranking
# can be checked at each one.
#
# Scoring always uses the whole scan, so best_inlier and the errors stay
# comparable across point counts; only the polish input is thinned, by a fixed
# stride that preserves the sweep's angular spread.
#
# Half the exp2 frames (start=5, stride=146) to keep the twelve configurations
# inside a couple of hours; n=61 resolves a collapse, not a 1%p difference.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HERE/out" "$HERE/logs"

"$HERE/build/polish_compare" \
  start=5 stride=146 threads=16 progress=5 \
  out="$HERE/out/exp4_polish_points.csv" \
  tag="[2/2 polish-points]" \
  config=icp,dist=4.0,iter=120 \
  config=icp,dist=4.0,iter=120,points=5000 \
  config=icp,dist=4.0,iter=120,points=2000 \
  config=icp,dist=4.0,iter=120,points=1000 \
  config=gicp,dist=4.0,iter=120 \
  config=gicp,dist=4.0,iter=120,points=5000 \
  config=gicp,dist=4.0,iter=120,points=2000 \
  config=gicp,dist=4.0,iter=120,points=1000 \
  config=ndt,res=3.0,iter=120 \
  config=ndt,res=3.0,iter=120,points=5000 \
  config=ndt,res=3.0,iter=120,points=2000 \
  config=ndt,res=3.0,iter=120,points=1000 \
  2>&1 | tee "$HERE/logs/exp4_polish_points.log"
