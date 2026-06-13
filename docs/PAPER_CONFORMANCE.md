# Paper Conformance

Audit baseline: Heck & Gleicher 2007 (*Parametric Motion Graphs*) and
Kovar, Gleicher & Pighin 2002 (*Motion Graphs*).

## Implemented

| Paper behavior | Implementation |
|---|---|
| Kovar-derived windowed point-cloud similarity and optimal floor alignment | `MotionDistance` |
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
| KG04-style parameter-accurate blend weights (D1) | `CalibrateParameterMetrics` / `ParameterCalibration` |
| Parameter-dependent blended clip duration (D2) | `BlendedDurationSeconds` / duration-derived `GenerateClip` |
| Source playback through cycle-crossing transitions (D3) | `RuntimeController::FoldCompletedCycles` |
| Metric/blend transition-window unification (D4) | `RuntimeControllerConfigFromArtifact` |
| Per-target transition phase lookup (D5) | `TargetTransitionPhaseSample` |
| Repeated unchanged self-edge traversal for continuous streams | viewer `RuntimeControlRequest` policy |

## Resolved Deviations

### D1 — Blend weights calibrated against a measured metric (resolved)

`ParametricMotionSpace` supports KG04-style sampled inversion. One-dimensional
spaces retain monotone parameter-adjacent sampling. Multidimensional spaces
sample a deterministic authored-domain grid and store each generated metric
vector with its full example weights. `ComputeLocalBlendWeights` maps the
requested authored coordinate to anchor-interpolated measured space, then
locally inverts normalized samples. Implemented metrics are `turn_rate` and
`travel_speed`; nodes without declared metrics keep Shepard interpolation.
Vector calibration serializes in `PMG_GRAPH_V7`; V5/V6 scalar tables remain
readable.

### D2 — Generated clip duration follows the parameter (resolved)

`GenerateClip(parameter, fps)` derives its frame count from
`BlendedDurationSeconds` (the weighted sum of example durations), so cycle
time varies with the parameter as in the paper. The builder and runtime use
this path. Frame-aligned diagnostics use the isolated
`pmg::legacy::GenerateClipWithFrameCount` compatibility API. Distance
thresholds in the specs were recalibrated because duration-true clips raised
absolute point-cloud distances.

### D3 — Source continues through cycle-crossing blends (resolved)

The runtime folds each completed source cycle into its accumulated world
placement and resumes sampling at the wrapped clip-local phase. Active
transitions therefore play real source frames through the whole blend instead
of clamping at the final pose. A regression test forces a transition window
across phase 1 and checks continuous root motion.

### D4 — Blend length equals the metric window (resolved)

`transition_blend_frames` is derived from the artifact edge builds'
`DistanceGridConfig::window_size` and is used for runtime blending,
point-cloud alignment, and goal-directed calibration. Because the runtime has
one global window, artifacts whose edge builds record different window sizes
are rejected explicitly.

### D5 — Transition phases remain target-dependent (resolved)

Each source sample stores the phase pair measured at every retained GOOD
target sample inside its shrunk reachable box. Runtime lookup clamps the
requested target to the interpolated box, interpolates phases in target
parameter space, then interpolates across source samples. Target phase samples
were introduced in `PMG_GRAPH_V6` and remain in V7; V2-V5 scalar phases remain
readable as fallback values.

## Known Deviations

Ordered by priority (impact on the paper's central claims first).

### D6 — Point-cloud sampling and scale differ from Kovar 2002 (low)

Kovar 2002 builds point clouds from downsampled skin-mesh vertices, uses a
source window beginning at the candidate frame and a target window ending at
the candidate frame, and defines an unnormalized weighted squared sum. The
implementation uses joint world positions, centered endpoint-clamped windows
for both clips, and weighted mean squared distance. The closed-form floor
alignment is the same optimization, but thresholds and distance values are
specific to this implementation.

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
- Complete (V7) artifacts and structured reports add reproducibility not
  specified by the original paper.

## Claim Limit

The code estimates transition validity and control behavior under this corpus,
distance metric, registration, sampling, and runtime configuration. It does not
establish general motion quality, global controllability, or reproduction of
the paper's unavailable datasets.
