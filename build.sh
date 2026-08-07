#!/usr/bin/env bash
# Standalone build of the polish-comparison harness. Nothing is written into the
# raisin workspace; only its headers and thirdparty sources are read.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLUGIN=/home/cgt24/raisin_master/src/raisin_plugin/raisin_lidar_slam_plugin

INCLUDES=(
  -I "$HERE/src"
  -I "$PLUGIN/include"
  -I "$PLUGIN/include/lidar_slam"   # nano_gicp's sources include "types.hpp" plainly
  -I "$PLUGIN/thirdparty/nano_gicp/include"
  -I "$PLUGIN/thirdparty/ndt_omp"
  -I /usr/include/pcl-1.14
  -I /usr/include/eigen3
)

# PCL_NO_PRECOMPILE is mandatory: without it the templates get instantiated
# against libpcl's own alignment settings and PCL objects corrupt the heap.
FLAGS=(-std=c++17 -O3 -DPCL_NO_PRECOMPILE -fopenmp -pthread -Wall
       -Wno-unused-variable -Wno-unused-but-set-variable -Wno-deprecated-declarations)

LIBS=(
  -lpcl_common -lpcl_io -lpcl_kdtree -lpcl_search -lpcl_features -lpcl_filters
  -lpcl_registration -lpcl_surface -llz4 -fopenmp
)

mkdir -p "$HERE/build"

compile() {
  local source="$1" object="$2"
  if [[ -f "$object" && "$object" -nt "$source" ]]; then
    echo "  up to date: $(basename "$object")"
    return
  fi
  echo "  cc $(basename "$source")"
  g++ "${FLAGS[@]}" "${INCLUDES[@]}" -c "$source" -o "$object"
}

echo "[1/3] thirdparty"
compile "$PLUGIN/thirdparty/nano_gicp/src/nano_gicp/nanoflann.cc" "$HERE/build/nanoflann.o"
compile "$PLUGIN/thirdparty/nano_gicp/src/nano_gicp/nano_gicp.cc" "$HERE/build/nano_gicp.o"
compile "$PLUGIN/thirdparty/nano_gicp/src/nano_gicp/lsq_registration.cc" \
  "$HERE/build/lsq_registration.o"

echo "[2/3] pcl instantiations"
compile "$HERE/src/pcl_instantiations.cpp" "$HERE/build/pcl_instantiations.o"

echo "[3/3] pipeline + harness"
compile "$HERE/src/global_localization.cpp" "$HERE/build/global_localization.o"
g++ "${FLAGS[@]}" "${INCLUDES[@]}" -c "$HERE/src/polish_compare.cpp" \
  -o "$HERE/build/polish_compare.o"
g++ "$HERE/build/polish_compare.o" "$HERE/build/global_localization.o" \
  "$HERE/build/pcl_instantiations.o" "$HERE/build/nano_gicp.o" \
  "$HERE/build/lsq_registration.o" "$HERE/build/nanoflann.o" \
  -o "$HERE/build/polish_compare" "${LIBS[@]}"

echo "built: $HERE/build/polish_compare"
