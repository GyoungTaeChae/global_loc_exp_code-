#!/usr/bin/env bash
# Experiment 3: is a polish tuned separately from tracking worth its code?
#
# The plugin already binds reg_loc_ to the whole map in loadGlobalMap() and
# already calls calculateSourceCovariances() on the scan in runLocPass() right
# before global localization runs. So a polish that reuses reg_loc_ costs one
# align() per candidate and nothing else: no PolishEngine, no tile target, no
# covariance work. The gicp_plugin config is exactly that, with every value read
# off the plugin at e9d662db:
#
#   max_correspondence_distance 1.0   params.yaml relocalization.gicp
#   max_iterations              32    gicp_max_iter_
#   correspondence_randomness   16    gicp_k_correspondences_
#   transformation/rotation eps 0.01  gicp_transformation_ep_, gicp_rotation_ep_
#   target                      whole map session, bound once
#
# Compared against the best tuned settings found so far. The pass-rate gap is
# the price of not reusing reg_loc_.
#
# Same frames as exp2/2b/2c/2d (start=5, stride=73).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HERE/out" "$HERE/logs"

"$HERE/build/polish_compare" \
  start=5 stride=73 threads=16 progress=10 \
  out="$HERE/out/exp3_plugin_default.csv" \
  tag="[1/2 plugin-default]" \
  config=none \
  config=gicp_plugin \
  config=gicp,dist=4.0,iter=60 \
  config=ndt,res=3.0,iter=120 \
  2>&1 | tee "$HERE/logs/exp3_plugin_default.log"
