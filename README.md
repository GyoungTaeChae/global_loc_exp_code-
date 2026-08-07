# Global localization polish experiment

Which refinement should sit behind FPFH + RANSAC global localization in
`raisin_lidar_slam_plugin`: point-to-point ICP, GICP, or NDT?

The pipeline is rewritten here from the plugin as of `e9d662db` (the parent of
`cde63e45`, the commit after which the plugin's global localization stopped
being trustworthy). `PIPELINE.md` documents every stage, its inputs and its
parameters, and lists where this implementation differs from that commit.
`RESULTS.md` holds the measurements.

## Data

Cross-session MulRan, both sequences already posed in UTM-52N so their ground
truth is directly comparable:

| role | sequence | frames |
|---|---|---|
| map | `/home/cgt24/work/dataset/rosBagFiles/mulran/kaist001` | 8226 (8046 posed) |
| query | `/home/cgt24/work/dataset/rosBagFiles/mulran/kaist002` | 8941 (8850 posed) |

The dataset ships no calibration file. The vehicle -> Ouster extrinsic is the
MulRan / Complex Urban platform convention (yaw 180 deg, pitch -1.5 deg,
translation 1.7042, -0.021, 1.8047), applied identically to both sequences —
see `mulran::baseToLidarExtrinsic`.

### The map the localizer searches

Rebuilding a map for every query frame would be unaffordable, so query frames
are grouped: each frame's ground-truth position is snapped to a `tile_grid`
lattice, and the tile is the accumulation of every map-session frame whose
ground-truth position lies within `tile_radius` of that lattice point, voxelized
to `tile_voxel`. Frames that snap to the same lattice point share one tile, and
the tile's FPFH descriptors, occupancy voxels and polish target structures are
built once for all of them.

With the defaults (`tile_radius=80`, `tile_grid=24`, `tile_voxel=0.5`) a tile is
~500 map frames and ~215 k points, and the query sits up to ~17 m off the tile
centre. So this measures no-hint localization inside an 80 m disc, not inside a
city-scale map: absolute pass rates are optimistic relative to a full map, and
the query scan reaches slightly past the tile edge, which caps the achievable
inlier fraction. Both effects apply identically to every polish method.

## Build

Compiles standalone against the plugin's headers and `thirdparty/`. It never
writes into the raisin workspace and never invokes colcon.

    ./build.sh            # -> build/polish_compare

Needs system PCL 1.14, Eigen 3, OpenMP. `PCL_NO_PRECOMPILE` is required
because `lidar_slam::Point` is a custom point type; the templates are
instantiated once in `src/pcl_instantiations.cpp`.

## Run

Every argument is `key=value`. One invocation runs the FPFH + RANSAC stages once
per frame and applies **all** `config=` polish settings to that same candidate
set, so configurations within a run differ only in the polish step.

    ./build/polish_compare \
      stride=10 threads=16 progress=10 \
      out=out/final.csv \
      config=none \
      config=icp,dist=2.0,iter=20 \
      config=gicp,dist=2.0,iter=20 \
      config=ndt,res=2.0,iter=30

| argument | default | meaning |
|---|---|---|
| `map`, `query` | kaist001, kaist002 | sequence directories |
| `out` | `out/polish_compare.csv` | per-frame per-config csv |
| `tag` | — | prefix printed on progress lines, e.g. `[3/9]` |
| `start`, `count`, `stride` | 0, all, 10 | query frame selection |
| `threads` | 16 | OMP threads |
| `progress` | 10 | print a progress block every N frames |
| `tile_radius`, `tile_voxel`, `tile_grid`, `tile_min_frames` | 80, 0.5, 24, 20 | map tile |
| `top_k`, `min_inlier`, `budget`, `corr_dist` | 10, 0.25, 10000, 1.0 | candidate stage |
| `query_voxel`, `normal_radius`, `fpfh_radius` | 0.5, 2.0, 4.0 | feature stage |
| `config=<method>[,key=value...]` | `none` | repeatable; `none`/`icp`/`gicp`/`ndt`, options `dist iter eps res step k label` |
| `gt_probe` | 1 | also align once from the ground-truth pose, to measure cross-session ground-truth disagreement |

Progress goes to stdout, flushed per block:

    [1/1]  40/147  elapsed 610s  eta 1631s  tiles 39 (0 thin)
      none                 pass 55.0%  >=10m 22.5%  p50 1.31m  p90 12.40m  polish 7ms
      icp_d2_i20           pass 80.0%  >=10m 12.5%  p50 0.41m  p90 3.10m  polish 2401ms

Long runs belong in the background:

    nohup ./run_exp1_sample.sh > /dev/null 2>&1 &
    tail -f logs/exp1_sample.log

### Scripts

| script | what it runs |
|---|---|
| `run_exp1_sample.sh` | trajectory-wide sample (stride 60), the four baseline configs, plus the ground-truth probe |

## Metrics

Per frame the estimate is compared with the query frame's interpolated ground
truth pose:

- **pass**: translation error < 2 m and rotation error < 5 deg
- **gross mismatch**: translation error >= 10 m (a frame with no candidate at all
  counts here)
- **p50 / p90**: translation error percentiles over frames that produced a
  candidate
- **polish_ms**: wall time of the polish + rescore loop over all K candidates,
  per frame

`gt_inlier` (occupancy inlier fraction at the ground-truth pose) and the
`gt_probe_*` columns bound how much the 2 m threshold can mean: they measure how
far the two sessions' ground truth disagree.

## Repository layout

    src/global_localization.{hpp,cpp}  the pipeline, stages 1-6
    src/mulran_sequence.hpp            MulRan reader (scans, ground truth, extrinsic)
    src/map_tile.hpp                   tile accumulation and lattice snapping
    src/pcl_instantiations.cpp         explicit templates for the custom point type
    src/polish_compare.cpp             harness main
    build.sh                           standalone build
    run_exp*.sh                        one script per experiment
    logs/                              stdout of each run
    out/                               csv results
    PIPELINE.md                        stage-by-stage contract and parameters
    RESULTS.md                         measurements and conclusions

Datasets and build outputs are gitignored; sources, scripts, logs and csv
results are committed.
