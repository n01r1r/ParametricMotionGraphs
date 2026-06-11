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

## Known Deviations

Ordered by priority (impact on the paper's central claims first). Motion-space
fidelity (D1, D2) outranks graph-layer polish; the graph layer is already
near-conformant.

### D1 — Blend weights are Shepard interpolation, not Kovar-Gleicher 2004 parameterization (high)

The paper builds motion spaces with K&G04: densely sample blend-weight space,
record which parameter each weight combination actually achieves, then invert
that map so a requested parameter is accurately reached.
`ParametricMotionSpace::ComputeLocalBlendWeights` instead applies
inverse-distance (Shepard) weights over the k = dim + 1 nearest authored
examples directly in parameter space, assuming the parameter is linear in the
weights. Observable consequence: `GoalDirectedLocomotion` must calibrate
achieved turn rates because requested curvature does not equal achieved
curvature.

### D2 — Generated clip duration ignores the parameter (high)

`GenerateClip(parameter, frame_count, fps)` produces a fixed-length clip from
configuration. The paper's blend duration is the weighted sum of the
(time-warped) example durations, so cycle time varies with the parameter.
A tight turn and a straight walk currently play over the same frame count,
distorting playback speed across the parameter range.

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
- V4 artifacts and structured reports add reproducibility not specified by the
  original paper.

## Claim Limit

The code estimates transition validity and control behavior under this corpus,
distance metric, registration, sampling, and runtime configuration. It does not
establish general motion quality, global controllability, or reproduction of
the paper's unavailable datasets.
