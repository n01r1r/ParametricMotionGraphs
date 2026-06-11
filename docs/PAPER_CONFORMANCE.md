# Paper Conformance

Audit baseline: Heck & Gleicher 2007 (*Parametric Motion Graphs*) and
Kovar, Gleicher & Pighin 2002 (*Motion Graphs*).

## Implemented

| Paper behavior | Implementation |
|---|---|
| Windowed point-cloud similarity and optimal floor alignment | `MotionDistance` |
| Distance-grid optimal transition point | `BuildDistanceGrid` / `FindOptimalTransition` |
| Seeded source and target sampling | `PmgBuilder` |
| GOOD/NEUTRAL/BAD double threshold | `PmgBuilderConfig` |
| GOOD AABB with BAD exclusion | `ParameterAabb::ShrinkToExclude` |
| Reject edge when any source sample has no target region | `BuildEdgeWithReport` |
| Average in-box GOOD transition phases | `TransitionSample` construction |
| Runtime target-box/phase interpolation | `PmgEdge::LookupInterpolated` |
| Recompute alignment at runtime | `PointCloudAlignment` |
| Random graph walk | outgoing-edge runtime selection |
| Target-directed and interactive control | `GoalDirectedLocomotion` |
| KG04-style parameter-accurate blend weights (D1) | `CalibrateParameterMetric` / `ParameterCalibration` |
| Parameter-dependent blended clip duration (D2) | `BlendedDurationSeconds` / duration-derived `GenerateClip` |

## Resolved Deviations

### D1 — Blend weights calibrated against a measured metric (resolved)

`ParametricMotionSpace` now supports KG04-style inversion: the offline build
samples the blend-weight axis between parameter-adjacent examples, measures
each blend (`parameter_metric <node> turn_rate` in the spec), and stores the
monotone (weight, measured) table. `ComputeLocalBlendWeights` inverts the
table so a requested parameter lands on the anchor-interpolated measured
value. Nodes without a declared metric keep the Shepard fallback; only 1-D
spaces and the turn-rate metric are implemented so far. The calibration
serializes with the space (`PMG_GRAPH_V5`).

### D2 — Generated clip duration follows the parameter (resolved)

`GenerateClip(parameter, fps)` derives its frame count from
`BlendedDurationSeconds` (the weighted sum of example durations), so cycle
time varies with the parameter as in the paper. The builder and runtime use
this path; the explicit-frame-count overload remains for frame-aligned
diagnostics only. Distance thresholds in the specs were recalibrated because
duration-true clips raised absolute point-cloud distances.

## Known Deviations

Ordered by priority (impact on the paper's central claims first).

### D3 — Source clip freezes when a blend crosses its end (medium)

During an active transition the source is sampled with a clamped phase
(`RuntimeController::SampleWorldClamped`) and holds its final pose. The paper
plays both motions through the whole transition window. Windows that straddle
the source clip's end blend against a frozen pose.

### D4 — Blend length and metric window are independent knobs (medium)

The runtime blend spans `blend_window_phase × target clip duration`; the
distance metric evaluates similarity over `window_size` frames. The paper ties
the transition length to the metric window, so similarity is measured over the
same temporal extent that is actually blended.

### D5 — Transition phases averaged across in-box GOOD hits (medium)

`PmgBuilder` stores one source/target phase pair per source sample: the average
over the GOOD hits inside the shrunk box. Per-target transition timing is
discarded, so the runtime uses the same target phase regardless of which target
parameter is chosen inside the box.

### D6 — Point clouds are joint positions, not mesh points (low)

Kovar 2002 builds the point cloud from downsampled skin-mesh vertices. The
implementation uses joint world positions, a coarser but common approximation.

### D7 — Optional velocity weighting is not in either paper (low)

`PointCloudWeighting::add_velocity_weight` multiplies point weights by
(1 + speed). Off by default; turning it on departs from the printed metric.

### D8 — Kovar 2002 graph machinery is out of scope (informational)

Local-minima transition harvesting, largest-SCC pruning, branch-and-bound path
search, and constraint-annotation footskate cleanup belong to the original
Motion Graphs system, not the PMG layer this project implements. Foot locking
here is an independent IK post-process.

## Local Extensions

- Contact registration and DTW refinement instantiate the smooth registered
  motion-space assumption used by the paper (registration curves' per-frame
  rigid alignment and constraint matching are approximated by root-delta
  blending and contact anchors; see D1/D2 for the remaining gap).
- Root-delta blending and optional foot locking address observed BVH artifacts.
- Complete (V5) artifacts and structured reports add reproducibility not
  specified by the original paper.

## Claim Limit

The code estimates transition validity and control behavior under this corpus,
distance metric, registration, sampling, and runtime configuration. It does not
establish general motion quality, global controllability, or reproduction of
the paper's unavailable datasets.
