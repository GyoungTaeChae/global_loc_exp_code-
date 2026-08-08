# Results

Which refinement belongs behind FPFH + RANSAC global localization in
`raisin_lidar_slam_plugin`: point-to-point ICP, GICP, or NDT?

Every number here is measured by `build/polish_compare` on cross-session MulRan
(map kaist001, query kaist002) and reproduced from the csv files in `out/`.
Nothing carried over from earlier sessions; the pipeline was rewritten from the
plugin at `e9d662db` (see `PIPELINE.md`).

## Conditions

Shared by every run below unless a table says otherwise:

| | |
|---|---|
| map session | kaist001, 8226 frames (8046 posed) |
| query session | kaist002, 8941 frames (8850 posed) |
| map the localizer searches | 80 m disc tile, `tile_voxel=0.5`, tile centres on a 24 m lattice |
| tile size | median 420 map frames, 215 k points (min 36 k, max 364 k) |
| scan stacking | **none** — one scan per query frame, no accumulation over time |
| polish input | the same downsampled scan, `query_voxel=0.5`, **median 11.2 k points** (min 5.7 k, max 15.4 k) |
| candidate stage | `top_k=10`, `min_inlier=0.25`, `budget=10000`, `corr_dist=1.0` |
| candidate sharing | one RANSAC run per frame; all configurations in a run polish the **same** candidate set |
| threads | 16 |
| pass | translation < 2 m and rotation < 5 deg |
| gross mismatch | translation >= 10 m; a frame with no candidate counts here |

The 80 m disc means these are no-hint pass rates *inside a bounded search area*,
not inside a city-scale map. Absolute rates would be lower on a full map. The
bound applies identically to every method, so the comparison is unaffected.

Note the polish input is about 11 k points, not the ~30 k the plugin would hand
it: the plugin feeds relocalization a scan at `slam_input_voxel_size=0.25`,
while this pipeline uses the global-localization stage's own
`query_voxel_size=0.5`. Timings here are therefore optimistic relative to the
plugin's current scan by roughly the point-count ratio.

### Timing is not symmetric across the three methods

`pcl::IterativeClosestPoint` has no OpenMP path and always runs on **one
thread**. `nano_gicp::NanoGICP` and `pclomp::NormalDistributionsTransform` both
honour `setNumThreads` and ran on **16**. Candidates are polished serially for
all three, so a `polish_ms` column compares 1-thread ICP against 16-thread GICP
and NDT and is unfair to ICP.

The csv records this per row in the `omp_threads` column, and the harness prints
it in the config banner and the summary table. It is not correctable by
configuration — PCL ships no threaded ICP. If ICP had ended up competitive on
accuracy the honest next step would have been to parallelize it across the ten
candidates and re-measure; it did not (it is last at every iteration count
tested), so that was not run.

`polish_ms` excludes target-side setup (kd-tree, GICP target covariances, NDT
voxel grid). On a robot that is paid once when the map is loaded, not per
localization request, so it is reported separately as `target_prep_ms`.

## Is the ground truth good enough to grade against?

Both sessions are posed in UTM-52N, but two MulRan passes do not agree exactly.
Two columns bound the disagreement: the occupancy inlier fraction at the ground
truth pose, and the residual of a GICP alignment *started* at the ground truth
pose.

| run | n | inlier at truth (p50 / p10) | GICP-from-truth offset (p50 / p90 / max) | rotation (p50 / p90) |
|---|---|---|---|---|
| exp1 | 146 | 0.958 / 0.855 | 0.64 m / 1.64 m / 2.48 m | 0.47 / 1.20 deg |
| exp2 | 122 | 0.958 / 0.861 | 0.67 m / 1.58 m / 2.63 m | 0.49 / 1.12 deg |

So part of the 2 m pass threshold is spent on ground-truth disagreement, and the
absolute pass rates below include that error. It is identical for every method,
so the ranking is unaffected — but "88 % pass" should not be read as "88 % of
poses are within 2 m of the true pose".

## exp1 — first trajectory-wide sample

`out/exp1_sample.csv`, `logs/exp1_sample.log`. n=146, stride 60, 35 min.
Library-default-ish settings.

| polish | pass% | >=10m% | p50 | p90 | p99 | polish ms/frame | threads |
|---|---|---|---|---|---|---|---|
| none | 21.9 | 13.7 | 2.82 | 19.55 | 54.43 | 8 | — |
| icp d=2 i=20 | 71.9 | 8.2 | 1.19 | 7.63 | 53.60 | 3298 | 1 |
| gicp d=2 i=20 | 84.9 | 7.5 | 0.86 | 5.86 | 38.01 | 3339 | 16 |
| ndt r=2 i=30 | 82.2 | 8.2 | 0.98 | 5.53 | 56.96 | 2148 | 16 |

Polish is not optional: 21.9 % -> 72-85 %.

## exp2 / exp2b / exp2c / exp2d — parameter sweeps

All four run on the same frames (`start=5, stride=73`), offset from the grid a
final stride-10 run would use so tuning and evaluation mostly do not share
frames. exp2 and exp2b are n=122; exp2c is n=122; exp2d was stopped by hand at
n=82 of 122.

Because they share frames, the four can be pooled on exp2d's 82-frame subset for
a like-for-like read. That subset is what the next table uses, so its numbers
run a few points above the full-122 numbers quoted elsewhere.

**Main knob (n=82 subset, 30 iterations):**

| polish | pass% | >=10m% | p50 | p90 | polish ms |
|---|---|---|---|---|---|
| none | 25.6 | 14.6 | 2.44 | 23.70 | 7 |
| icp d=1 | 67.1 | 7.3 | 1.24 | 6.15 | 4310 |
| icp d=2 | 79.3 | 6.1 | 0.97 | 5.50 | 4497 |
| icp d=4 | 82.9 | 6.1 | 0.97 | 3.87 | 4510 |
| icp d=6 | 84.1 | 7.3 | 0.97 | 3.70 | 4516 |
| gicp d=1 | 86.6 | 4.9 | 0.66 | 2.90 | 4956 |
| gicp d=2 | 90.2 | 3.7 | 0.64 | 1.91 | 4924 |
| gicp d=4 | 91.5 | 3.7 | 0.74 | 1.56 | 4823 |
| gicp d=6 | 91.5 | 4.9 | 0.69 | 1.56 | 4632 |
| ndt r=1 | 81.7 | 6.1 | 0.80 | 4.80 | 2007 |
| ndt r=2 | 84.1 | 4.9 | 0.75 | 4.13 | 2153 |
| ndt r=3 | 84.1 | 4.9 | 0.78 | 4.36 | 2170 |
| ndt r=4 | 85.4 | 4.9 | 0.80 | 4.37 | 2118 |

Past `d=4` / `r=3` every method gains under 1 %p. That knob is done.

**Iteration cap (n=82 subset, at each method's best main knob):**

| polish | i=30 | i=60 | i=120 | i=240 | polish ms at i=120 |
|---|---|---|---|---|---|
| icp d=4 | 82.9 | 87.8 | 89.0 | 90.2 | 10700 (1 thread) |
| gicp d=4 | 91.5 | 91.5 | 92.7 | **92.7** | 11477 (16 threads) |
| ndt r=3 | 84.1 | 90.2 | 93.9 | **93.9** | **3702** (16 threads) |

This is the sweep that mattered. At 30 iterations GICP looks like a clear winner
over NDT (91.5 vs 84.1) — but NDT was pinned against the iteration cap, not
losing on merit. Raising the cap gained NDT 9.8 %p and GICP 1.2 %p, and both are
identical at 120 and 240, so they are converged. **Concluding from exp2 alone
would have put the wrong method in the plugin.**

Full-n figures at the converged setting, for the record:

| polish (n=122) | pass% | >=10m% | p50 | p90 | polish ms | oracle pass% |
|---|---|---|---|---|---|---|
| icp d=4 i=120 | 83.6 | 8.2 | 0.94 | 2.50 | 10878 | 86.9 |
| gicp d=4 i=120 | 86.9 | **5.7** | 0.87 | 2.11 | 11600 | 86.9 |
| ndt r=3 i=120 | **88.5** | 6.6 | **0.72** | **2.06** | **3753** | **89.3** |

NDT leads pass rate and both error percentiles at **3.1x less wall time**, on
equal threads. GICP has the lowest gross-mismatch rate. The 1.6 %p pass gap is
two frames at n=122 (standard error ~3 %p) and is not yet significant; the
timing gap is.

## What polish actually contributes

The `oracle` columns separate candidate generation from selection.
`oracle_dt` is the closest of the ten raw candidates to the truth;
`oracle_polished_dt` the closest of the ten polished ones.

On exp2's 122 frames:

- the ten **raw** candidates already contain a pose within 2 m on **58.2 %** of
  frames, but the occupancy score picks it on only **24.6 %** — 33.6 %p is
  selection failure, not a missing candidate
- after polish the two collapse together: gicp d=4 i=120 scores 86.9 % actual
  against 86.9 % oracle, ndt r=3 i=120 scores 88.5 % against 89.3 %

So polish's main contribution is **not refinement, it is discrimination**. Raw
RANSAC poses all score mediocre on the dilated occupancy voxels and cannot be
told apart; polish pulls the correct one up until the score separates it.

### The gross-mismatch floor is not polish's to fix

Gross mismatch sits at 5.7-8.2 % for every method and every setting, because on
those frames all ten candidates are wrong before polish ever runs. Choosing a
different polish cannot move it. Reducing it means changing the candidate stage
— `top_k`, `min_inlier`, the RANSAC budget, or the descriptor — which is outside
this experiment.

## exp3 — what reusing the plugin's own registration costs

`out/exp3_plugin_default.csv`, `logs/exp3_plugin_default.log`. n=122, same
frames as the sweeps, 45 min.

The plugin already binds `reg_loc_` to the whole map in `loadGlobalMap()` and
already calls `calculateSourceCovariances()` on the scan in `runLocPass()`
immediately before global localization runs. A polish that reuses `reg_loc_`
therefore costs one `align()` per candidate and nothing else — no PolishEngine,
no tile target, no covariance work. `gicp_plugin` is exactly that, with every
value read off the plugin at `e9d662db`:

| | value | where |
|---|---|---|
| max correspondence distance | 1.0 | `params.yaml relocalization.gicp` -> `params.cpp:140` |
| max iterations | 32 | `gicp_max_iter_` |
| correspondence randomness | 16 | `gicp_k_correspondences_` |
| transformation / rotation epsilon | 0.01 / 0.01 | `gicp_transformation_ep_`, `gicp_rotation_ep_` |
| initial lambda factor | 1e-9 | `gicp_init_lambda_factor_` |
| threads | 16 | nano_gicp's constructor default; the plugin never calls `setNumThreads` |
| target | whole map session, bound once | `loadGlobalMap` |

| polish | pass% | >=10m% | p50 | p90 | polish ms | target prep |
|---|---|---|---|---|---|---|
| none | 24.6 | 16.4 | 2.47 | 25.86 | 7 | — |
| **gicp_plugin** | **81.1** | 8.2 | 0.97 | **7.95** | 4848 | 3.3 s once |
| gicp d=4 i=60 | 86.1 | 6.6 | 0.90 | 2.24 | 7786 | 0.10 s per tile |
| ndt r=3 i=120 | 88.5 | 6.6 | 0.72 | 2.06 | 3751 | 0.01 s per tile |

The whole kaist001 session accumulates to **4.58 M points** at 0.5 m, built in
17.8 s; binding GICP to it (target covariances over 4.58 M points) takes 3.3 s.
Both are one-time costs and are excluded from `polish_ms`.

`pcl::VoxelGrid` cannot build that map: over a multi-kilometre session at a
0.5 m leaf its voxel index overflows and it returns the input unfiltered with
only a warning. `glexp::buildFullMap` uses a 64-bit voxel key instead.

**The gap is 5.0 %p against tuned GICP and 7.4 %p against NDT.** Because all
configurations see the same frames and the same candidates, these are paired
comparisons (McNemar, exact two-sided):

| comparison | frames only A passes | only B passes | p |
|---|---|---|---|
| ndt r=3 i=120 vs gicp_plugin | 9 | 0 | **0.004** |
| gicp d=4 i=60 vs gicp_plugin | 7 | 1 | 0.070 |
| ndt r=3 i=120 vs gicp d=4 i=60 | 3 | 0 | 0.250 |

So the plugin default is significantly worse than tuned NDT, and worse than
tuned GICP at a level the sample cannot quite resolve. The p90 gap is the
sharper one: **7.95 m against 2.06-2.24 m** — the plugin's settings usually find
the pose but miss badly when they miss.

That gap has two independent causes, the registration settings and the
whole-map target, and only the settings are cheap to change on a robot (two
setter calls on the `reg_loc_` the plugin already owns). Which cause carries the
5 %p is not answered by this run; a 2x2 ablation was started and stopped by
hand at n=17, too few to report.

## exp4 — how many scan points does polish need?

`out/exp4_polish_points.csv`, `logs/exp4_polish_points.log`. n=61
(`start=5, stride=146`, half the exp2 frames), 55 min. Each method at its
converged setting, thinned by a fixed stride. Scoring always uses the whole
scan, so the columns stay comparable across point counts.

| polish | full (~11 k) | 5000 | 2000 | 1000 |
|---|---|---|---|---|
| icp d=4 i=120 — pass% / ms | 86.9 / 10660 | 86.9 / 4680 | 86.9 / 1917 | 86.9 / **974** |
| gicp d=4 i=120 — pass% / ms | 91.8 / 11671 | 91.8 / 11251 | 91.8 / 12569 | 90.2 / 11746 |
| ndt r=3 i=120 — pass% / ms | 91.8 / 3798 | 90.2 / 3179 | 88.5 / 2632 | 88.5 / 2343 |

Paired, full against 1000 points: ICP **0** discordant frames, GICP 1, NDT 2 —
no method degrades significantly at n=61.

Three things follow.

1. **ICP's cost is entirely the point count.** 11x less wall time from full to
   1000 points with an identical pass set, and that 974 ms is on **one thread**.
   Spread over the ten candidates it would be in the hundreds of milliseconds.
   ICP is not slow; it was being fed too many points.
2. **GICP's cost is irreducible.** Thinning does not speed it up at all
   (11671 -> 11746 ms). With fewer points it stops converging and runs the
   iteration cap instead.
3. **NDT is the only one whose accuracy tracks the point count** (91.8 ->
   88.5 %), and it saves only 1.6x.

The ranking `gicp >= ndt > icp` holds at every point count, but at n=61 no
between-method difference is significant (gicp vs icp p=0.25, gicp vs ndt 0
discordant frames). The ranking evidence remains the n=122 sweeps; exp4's value
is the within-method cost curves, which are paired and much tighter.

## Where this stands

On 16 threads with an ~11 k-point scan, converged: **NDT r=3 i=120** leads pass
rate, p50, p90 and wall time at n=122; GICP d=4 i=120 leads gross mismatch; ICP
is last on accuracy at every setting.

exp4 complicates that for deployment: ICP at 1000 points holds its accuracy at
974 ms single-threaded, a quarter of NDT's cost and a twelfth of GICP's, giving
up about 5 %p. Which of those trades is right depends on the robot's budget, and
is not something the measurements decide on their own.

Nothing has been changed in the plugin.

Open, not measured:

- the exp3 2x2 (settings vs target) that would say whether the plugin can keep
  reusing `reg_loc_` with two setter calls changed
- a final comparison at large n (stride 10, n~885); every ranking above rests on
  n=122 or less, where 2-3 %p is inside the noise
- the gross-mismatch floor of 5.7-8.2 %, which belongs to the candidate stage
- ICP parallelized across candidates, which exp4 makes worth measuring
