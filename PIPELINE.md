# FPFH + RANSAC global localization: stages, inputs, parameters

The reference implementation is `src/global_localization.{hpp,cpp}` in this
repository. It was written from scratch against the plugin as of
`e9d662db` (`raisin_lidar_slam_plugin`, the parent of `cde63e45`); the
differences from that commit are listed at the end.

Frames: a scan lives in the lidar sensor frame, the map in an origin-shifted
UTM-52N frame, and every transform below maps scan -> map. All lengths are
metres.

## Stage 1 — scan downsample

    in   raw sensor-frame scan (MulRan Ouster, ~65 k points after no-return removal)
    out  voxelized scan, ~8-10 k points
    par  query_voxel_size = 0.5

`Localizer::downsample`. The same voxel size is applied to the map inside
`setMap`, so features on both sides are computed at one resolution. The
downsampled scan is what every later stage sees, polish included.

## Stage 2 — FPFH descriptors

    in   voxelized cloud
    out  33-bin FPFH per point, 1:1 with the cloud
    par  normal_estimation_radius = 2.0, fpfh_search_radius = 4.0, num_threads

`Localizer::extractFpfh`, PCL's OMP normal estimation followed by
`FPFHEstimationOMP`. Run once per scan and once per map. On a 215 k-point map
tile this is the dominant per-tile cost (~4 s of a ~4.5 s tile setup on 16
threads); on a 10 k-point scan it is ~0.5 s.

## Stage 3 — feature matching

    in   scan features, map feature kd-tree
    out  for each scan point, the k feature-nearest map point indices
    par  correspondence_randomness = 2 (this is k), num_threads

`pcl::KdTreeFLANN<FPFHSignature33>::nearestKSearch`, parallel over scan points.
RANSAC then draws from these lists instead of querying the tree per iteration.

## Stage 4 — RANSAC with polygon prerejection and SVD

    in   voxelized scan, downsampled map, the k-nearest lists, occupancy voxels
    out  every hypothesis whose inlier fraction cleared min_inlier_fraction
    par  ransac_max_iterations = 1000000, ransac_matching_budget = 10000,
         similarity_threshold = 0.5, min_inlier_fraction = 0.25, num_threads

Per iteration: draw 3 distinct scan points, pick one of the k feature matches
for each, reject the triple when
`CorrespondenceRejectorPoly::thresholdPolygon` finds the two triangles'
edge-length ratios disagree by more than `similarity_threshold`, otherwise
estimate the rigid transform by SVD and score it (stage 6). `max_iterations`
is a ceiling that is never reached in practice; the budget on *scored*
hypotheses is what stops the loop.

Each thread gets `ransac_matching_budget / num_threads` scored hypotheses and
its own `mt19937` seeded from the thread index and the two cloud sizes, so the
loop is reproducible: two runs on the same inputs produce byte-identical
candidates (verified, `out/det1.csv` vs `out/det2.csv` differ only in the
timing columns).

Observed on kaist001/kaist002 tiles: ~1 k-5 k of the 10 k scored hypotheses
clear 0.25 inlier fraction. The occupancy test is dilated (see stage 6), so
clearing 0.25 is weak evidence on its own.

## Stage 5 — top-K candidates

    in   all passing hypotheses
    out  at most top_k of them, best first
    par  top_k = 10, suppress_translation = 2.0, suppress_rotation_degrees = 10.0

Sorted by inlier fraction (ties broken by lower mean squared residual), then
walked best-first: a hypothesis within `suppress_translation` **and**
`suppress_rotation_degrees` of one already kept is dropped. Without the
suppression the ten slots fill with ten copies of the same pose, because RANSAC
returns many near-identical hits; with it, all 10 slots are typically distinct.

## Stage 6 — polish, then occupancy-voxel rescore

    in   the K candidates, the voxelized scan, the map
    out  the single best pose
    par  per polish method, see below

Each candidate is handed to the polish method as an initial guess, and the
refined pose is rescored by the occupancy test. The best rescore wins.

The occupancy test (`Localizer::score`, also used inside RANSAC) is a hash set
of voxels at `max_correspondence_distance = 1.0` resolution. A voxel is
occupied when a map point lies within 1.0 m of its centre, which is built by
marking the qualifying members of each map point's 27-neighbourhood — so the
set is dilated by about one voxel beyond the map surface. A transformed scan
point that lands in an occupied voxel is an inlier, and its residual is the
squared distance to that voxel's centre. The score is
`inlier_fraction = inliers / scan points`, with mean squared residual as the
tie-break.

The polish result is always taken, whatever the method's convergence flag says:
the three libraries define convergence differently, and the rescore already
rejects a polish that made the pose worse. The flag is recorded but not acted
on.

### Polish methods under test

| method | class | parameters |
|---|---|---|
| `none` | — | baseline: keep the RANSAC pose |
| `icp` | `pcl::IterativeClosestPoint<PointType, PointType>` | `dist` = max correspondence distance, `iter`, `eps` = transformation epsilon |
| `gicp` | `nano_gicp::NanoGICP<PointType, PointType>` | `dist`, `iter`, `eps`, `k` = covariance neighbourhood (20) |
| `ndt` | `pclomp::NormalDistributionsTransform<PointType, PointType>` | `res` = voxel resolution, `iter`, `eps`, `step` = Newton step (0.1) |

All three run on the same `PointType` clouds — the same downsampled scan
against the same map tile — so only the registration differs. Target-side setup
(ICP's kd-tree, GICP's target covariances, NDT's voxel-covariance grid) happens
once per map tile in `PolishEngine::setTarget`, and source-side setup once per
scan, so the per-candidate cost is the alignment alone.

Threading is not equal across the three and cannot be made so: PCL's ICP is
single threaded, while nano_gicp and pclomp NDT take `num_threads`. Candidates
are polished serially for all three. Wall-clock per frame is therefore not a
like-for-like comparison; it is reported as measured, with this caveat.

## Differences from the plugin at e9d662db

1. **Stages 5 and 6 are new.** `FpfhRansacLocalizer::localize` keeps a single
   running best hypothesis and has no polish step at all.
2. **RANSAC stopping rule.** The plugin shares one `std::atomic<int>` budget
   counter across threads, so where the loop stops depends on thread
   interleaving and the candidate set is not reproducible. Here each thread has
   a fixed share.
3. **Per-thread RANSAC helpers.** The plugin shares one
   `TransformationEstimationSVD` and one `CorrespondenceRejectorPoly` across all
   OMP threads. In PCL 1.14 `estimateRigidTransformation` is `const` and
   `thresholdPolygon`, although not marked `const`, only reads members and works
   on locals, so the sharing is benign — but each thread gets its own here so
   the question does not arise.
4. **No hint-region crop.** The plugin has `setGlobalMapRegion`, which crops
   features to `region_radius` around a GUI tap and verifies out to
   `region_verify_margin`. This experiment feeds `setMap` a map tile built by
   the harness instead, so the localizer searches the whole cloud it is given.

Everything else — the FPFH parameters, the k-nearest scheme, the polygon
prerejection, the per-thread seed formula, the occupancy voxel construction and
the inlier score — is the same as e9d662db.
