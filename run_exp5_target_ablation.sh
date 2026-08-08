#!/usr/bin/env bash
# Experiment 5: the plugin-default gap is 5 %p — which half of it is which?
#
# gicp_plugin differs from the tuned GICP in two independent ways: its
# registration settings (dist 1.0, 32 iterations, epsilon 0.01) and its polish
# target (the whole map instead of the tile the candidates came from). Only the
# settings are cheap to change on a robot — they are setter calls on the
# reg_loc_ the plugin already owns — so it matters which one costs the accuracy.
#
# A 2x2: {plugin settings, tuned settings} x {full map target, tile target}.
#
# Same frames as exp2/2b/2c/2d/exp3 (start=5, stride=73).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HERE/out" "$HERE/logs"

"$HERE/build/polish_compare" \
  start=5 stride=73 threads=16 progress=10 \
  out="$HERE/out/exp5_target_ablation.csv" \
  tag="[1/1 target-ablation]" \
  config=gicp_plugin \
  config=gicp,dist=1.0,iter=32,k=16,eps=0.01,reps=0.01,label=gicp_pluginparams_tile \
  config=gicp,dist=4.0,iter=120,k=16,eps=0.01,reps=0.01,fullmap=1,label=gicp_tuned_fullmap \
  config=gicp,dist=4.0,iter=120,k=16,eps=0.01,reps=0.01,label=gicp_tuned_tile \
  2>&1 | tee "$HERE/logs/exp5_target_ablation.log"
