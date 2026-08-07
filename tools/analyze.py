#!/usr/bin/env python3
"""Summarise one or more polish_compare csv files as a markdown table.

    tools/analyze.py out/exp1_sample.csv [out/exp2_sweep.csv ...]

Frames that produced no candidate at all are counted as gross mismatches and
left out of the error percentiles, matching the harness's own tally.
"""

import csv
import math
import sys
from collections import defaultdict


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(fraction * len(ordered)))
    return ordered[index]


def is_number(text):
    try:
        value = float(text)
    except ValueError:
        return False
    return not math.isnan(value)


def load(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def summarise(rows):
    per_config = defaultdict(list)
    for row in rows:
        per_config[row["config"]].append(row)

    lines = []
    lines.append(
        "| config | n | pass% (<2m,<5deg) | >=10m% | p50 [m] | p90 [m] | p99 [m] "
        "| polish ms/frame | oracle pass% |"
    )
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for config, config_rows in per_config.items():
        errors = [float(r["err_trans"]) for r in config_rows if is_number(r["err_trans"])]
        passed = sum(
            1
            for r in config_rows
            if is_number(r["err_trans"])
            and float(r["err_trans"]) < 2.0
            and float(r["err_rot_deg"]) < 5.0
        )
        gross = sum(
            1
            for r in config_rows
            if not is_number(r["err_trans"]) or float(r["err_trans"]) >= 10.0
        )
        polish = sum(float(r["polish_ms"]) for r in config_rows) / len(config_rows)
        total = len(config_rows)
        # Best of the K polished candidates, whether or not the score picked it.
        oracle_column = "oracle_polished_dt"
        oracle_passed = sum(
            1
            for r in config_rows
            if is_number(r.get(oracle_column, "nan")) and float(r[oracle_column]) < 2.0
        )
        has_oracle = any(is_number(r.get(oracle_column, "nan")) for r in config_rows)
        lines.append(
            "| {} | {} | {:.1f} | {:.1f} | {:.2f} | {:.2f} | {:.2f} | {:.0f} | {} |".format(
                config,
                total,
                100.0 * passed / total,
                100.0 * gross / total,
                percentile(errors, 0.5),
                percentile(errors, 0.9),
                percentile(errors, 0.99),
                polish,
                "{:.1f}".format(100.0 * oracle_passed / total) if has_oracle else "-",
            )
        )
    return lines


def ground_truth_stats(rows):
    """One row per frame; the frame columns repeat across configs."""
    seen = {}
    for row in rows:
        seen[row["frame"]] = row
    frames = list(seen.values())

    at_truth = [float(r["gt_inlier"]) for r in frames if is_number(r["gt_inlier"])]
    probed = [
        float(r["gt_probe_dt"])
        for r in frames
        if r.get("gt_probe_ok") == "1" and is_number(r["gt_probe_dt"])
    ]
    probed_rotation = [
        float(r["gt_probe_dr"])
        for r in frames
        if r.get("gt_probe_ok") == "1" and is_number(r["gt_probe_dr"])
    ]
    lines = ["", "Ground truth consistency over {} frames:".format(len(frames))]
    lines.append(
        "  inlier fraction at the ground truth pose: p50 {:.3f}  p10 {:.3f}  min {:.3f}".format(
            percentile(at_truth, 0.5), percentile(at_truth, 0.1), min(at_truth) if at_truth else float("nan")
        )
    )
    if probed:
        lines.append(
            "  GICP-from-ground-truth offset (n={}): p50 {:.2f}m  p90 {:.2f}m  max {:.2f}m".format(
                len(probed), percentile(probed, 0.5), percentile(probed, 0.9), max(probed)
            )
        )
        lines.append(
            "  same, rotation: p50 {:.2f}deg  p90 {:.2f}deg  max {:.2f}deg".format(
                percentile(probed_rotation, 0.5),
                percentile(probed_rotation, 0.9),
                max(probed_rotation),
            )
        )
    else:
        lines.append("  GICP-from-ground-truth probe: no converged samples")

    candidates = [int(r["distinct"]) for r in frames]
    lines.append(
        "  distinct candidates per frame: p50 {}  p10 {}  min {}".format(
            percentile(candidates, 0.5), percentile(candidates, 0.1), min(candidates)
        )
    )
    oracle = [
        float(r["oracle_dt"]) for r in frames if is_number(r.get("oracle_dt", "nan"))
    ]
    if oracle:
        lines.append(
            "  closest raw candidate to the truth: <2m on {:.1f}% of frames, p50 {:.2f}m".format(
                100.0 * sum(1 for d in oracle if d < 2.0) / len(frames), percentile(oracle, 0.5)
            )
        )
    feature = [float(r["feature_ms"]) for r in frames]
    ransac = [float(r["ransac_ms"]) for r in frames]
    lines.append(
        "  candidate stage per frame: feature p50 {:.0f}ms, ransac p50 {:.0f}ms".format(
            percentile(feature, 0.5), percentile(ransac, 0.5)
        )
    )
    return lines


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    rows = []
    for path in sys.argv[1:]:
        rows.extend(load(path))
    for line in summarise(rows):
        print(line)
    for line in ground_truth_stats(rows):
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
