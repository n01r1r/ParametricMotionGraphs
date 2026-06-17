# Cyclic Recut Search

Last updated: 2026-06-17.

## Purpose

Find diagnostic-only cut-point candidates for the current Group-B walk/jog
BVHs. The search reuses the same local seam pose, root-speed, yaw-rate, and
contact evidence measured by `CyclicContinuity`.

This report recommends candidate frame windows only. It does not change BVH
files, specs, PMG artifact construction, edge thresholds, transition selection,
runtime scheduling, or viewer behavior.

## Inputs

- Source BVHs:
  - `BVH/walkCurve.bvh`
  - `BVH/walkMoreCurve.bvh`
  - `BVH/walkTightCurve.bvh`
  - `BVH/jogCurve.bvh`
- Baseline cyclic evidence:
  - `outputs/cyclic_continuity/walk_jog_cyclic.csv`
  - `docs/experiments/CYCLIC_CONTINUITY_AUDIT.md`
- Search outputs:
  - `outputs/cyclic_recut_search/group_b_walk_jog_recuts.csv`
  - `outputs/cyclic_recut_search/group_b_walk_jog_recuts.md`

The `outputs/` directory is ignored by git, so this document preserves the
tracked conclusion.

## Method

Command:

```powershell
.\build\Debug\pmg_cli.exe --search-cyclic-recuts `
  --bvh-dir .\BVH `
  --output-csv outputs\cyclic_recut_search\group_b_walk_jog_recuts.csv `
  --output-md outputs\cyclic_recut_search\group_b_walk_jog_recuts.md
```

The command scans inclusive start/end frame windows of 18-45 frames in each
raw BVH. Every candidate window is cropped in memory and scored by
`MeasureCyclicContinuity` with:

- cycle joint: `LeftAnkle`;
- contact joints: `LeftAnkle,RightAnkle`;
- minimum contact run: `3` frames;
- default `CyclicContinuityConfig` thresholds.

Candidate score is the maximum of threshold-normalized seam pose, root-speed,
yaw-rate, and contact evidence metrics. Lower is better. A score below `1.0`
means the candidate is below every current local cyclic-continuity threshold.

## Recommended Recut

| Clip | Current authored classification | Recommended start | Recommended end | Frames | Score | Seam ratio | Root ratio | Yaw ratio | Contact drift | Recommended classification |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| `walkCurve.bvh` | `weak_pose_seam` | 86 | 119 | 34 | 0.774932 | 0.287581 | 1.00741 | 1.05356 | 0.150844 | `strong` |
| `walkMoreCurve.bvh` | `weak_pose_seam` | 76 | 110 | 35 | 0.774925 | 0.481108 | 1.00740 | 1.04884 | 0.297083 | `strong` |
| `walkTightCurve.bvh` | `weak_yaw_rate` | 58 | 95 | 38 | 0.769605 | 1.12347 | 1.00049 | 1.12140 | 0.751990 | `strong` |
| `jogCurve.bvh` | `weak_yaw_rate` | 39 | 63 | 25 | 0.771054 | 0.162738 | 1.00171 | 1.15658 | 0.132768 | `strong` |

## Interpretation

Under the current local cyclic-continuity diagnostic, every searched BVH has at
least one in-memory recut window that scores below all default thresholds. The
best windows are later in the source BVHs than the current first-contact-cycle
extraction used by `PrepareMotionSpaces`, and they substantially reduce the
baseline walk pose seams and jog yaw-rate discontinuity.

This is corpus-curation evidence, not a runtime fix. Before replacing any
anchor, the candidate windows need visual inspection and a separate artifact
comparison that checks:

- walk curvature/travel semantics remain acceptable after recutting;
- contact-registration structure still matches across the walk examples;
- generated walk/jog cyclic samples remain strong or improve;
- random-walk and transition-quality reports do not regress.

## Checklist

- [x] Search uses `CyclicContinuity` metrics instead of a new seam metric.
- [x] Search writes reproducible CSV and markdown outputs.
- [x] Four requested Group-B walk/jog BVHs were searched.
- [x] No PMG runtime, spec, or BVH behavior changed.
- [x] Build passed for `pmg_cli`.
- [ ] Candidate BVH replacement/spec recut has not been applied.
- [ ] Visual/runtime validation of candidate windows has not been run.
