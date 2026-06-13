# Paper Conformance

Audit baseline: Heck & Gleicher 2007 (*Parametric Motion Graphs*) and the
underlying transition metric from Kovar, Gleicher & Pighin 2002 (*Motion
Graphs*). This document is the line-by-line conformance record: every element
of the paper pipeline, where it lives in code, and what (if anything) remains.

## How to read this

Status legend used in the tables below:

- **✓ Done** — implemented faithfully to the paper.
- **◐ Adapted** — paper behavior with a documented representation change or
  approximation (see the deviation register for the rationale).
- **✗ Gap** — paper-relevant behavior that is genuinely not implemented.
- **○ Out of scope** — belongs to the surrounding Motion Graphs / KG04 systems,
  intentionally excluded (see the Claim Limit).

Deviation ids (`D1`–`D8`) are stable and referenced from `README.md`,
`docs/IMPLEMENTATION_PLAN.md`, and `docs/adr/0003-parameter-accuracy-calibration.md`.

## §3 — Motion spaces

| Paper element | Status | Implementation | Source |
|---|---|---|---|
| Motion space = blend of registered example motions | ✓ | `ParametricMotionSpace` | `include/pmg/ParametricMotionSpace.h:62` |
| Scattered-data blend weights, `k = dim + 1` nearest, k-th-neighbor cutoff (Eq. 2) | ✓ | `ComputeLocalBlendWeights` | `src/ParametricMotionSpace.cpp:556`; `docs/adr/0002-pmg-knn-cutoff.md` |
| Parameter-accurate inverse (authored coordinate → weights that achieve it) | ✓ (D1) | `CalibrateParameterMetrics` / `ParameterCalibration` | `include/pmg/ParametricMotionSpace.h:50` |
| Blend inherits example timing (cycle length varies with the parameter) | ✓ (D2) | `BlendedDurationSeconds` → `GenerateClip` | `include/pmg/ParametricMotionSpace.h:99` |
| Time registration so blends combine corresponding moments | ◐ | contact-anchor `TimeWarp` + slope-constrained DTW refine | `include/pmg/MotionRegistration.h:22` |
| KG04 registration curves (per-frame rigid alignment + constraint matching) | ◐ | approximated by contact anchors + root-delta integration | `include/pmg/ParametricMotionSpace.h:106` |

**Registration fidelity (◐).** The paper assumes a *smooth registered motion
space* produced by KG04 registration curves: per-frame rigid frame alignment,
dynamic time warping, and constraint matching. This repository approximates
that with contact-anchor time warps, a slope-constrained DTW refinement pass,
and root-delta blending. Blends are measurably smooth (the monotone turn-rate
sweep in `tests/test_parametric_motion_space.cpp`), but this is an
approximation of the KG04 algorithm, not a reimplementation of it. This is the
largest remaining *fidelity* gap; it is accepted as a local extension because
the corpus has no skin mesh and the approximation holds on the included clips.

## §4 — Parametric motion graph construction

| Paper element | Status | Implementation | Source |
|---|---|---|---|
| Node = motion space, edge = sampled transition set | ✓ | `PmgNode` / `PmgEdge` | `include/pmg/ParametricMotionGraph.h:12` |
| Sample source × target parameter spaces | ✓ | `PmgBuilder` (50 source / 1000 target default) | `include/pmg/PmgBuilder.h:13` |
| Transition distance = windowed point cloud (Kovar 2002) | ◐ (D6) | `MotionDistance::BuildDistanceGrid` | `include/pmg/MotionDistance.h:99` |
| Closed-form 2-D floor-plane alignment | ✓ | `AlignedPointCloudDistance` | `include/pmg/MotionDistance.h:94` |
| Optimal transition cell of the grid | ✓ | `FindOptimalTransition` | `include/pmg/MotionDistance.h:107` |
| GOOD / NEUTRAL / BAD double threshold | ✓ | `PmgBuilderConfig` | `include/pmg/PmgBuilder.h:20` |
| Enclose GOOD targets in an AABB, shrink to exclude BAD | ✓ | `ParameterAabb::ShrinkToExclude` | `src/PmgBuilder.cpp` |
| Reject the edge if any source sample has no reachable box | ✓ | `BuildEdgeWithReport` | `include/pmg/PmgBuilder.h:77` |
| Per-target transition phase (not a single scalar) | ✓ (D5) | `TargetTransitionPhaseSample` | `include/pmg/TransitionTypes.h` |
| Restrict the transition search to a source-phase sub-range (§6.3) | ◐ | `DistanceGridConfig` defaults (source `[0.70, 0.95]`, target `[0.05, 0.30]`) | `include/pmg/PmgBuilder.h:31` |

**Source-range restriction (◐).** The mechanism exists and is on by default,
but the `.pmg_spec` `edge_config` line only exposes thresholds, sample counts,
and seed (`include/pmg/GraphSpec.h:54`). The phase ranges and metric window
size are hardwired builder defaults, not per-edge tunable. Making them
spec-controllable is small, bounded plumbing.

## §5 — Runtime and control

| Paper element | Status | Implementation | Source |
|---|---|---|---|
| Stream a graph walk; select an outgoing edge | ✓ | `RuntimeController` | `include/pmg/RuntimeController.h:55` |
| Interpolate the target box + phases at the source parameter (Eqs. 1–3) | ✓ | `PmgEdge::LookupInterpolated` | `include/pmg/ParametricMotionGraph.h:36` |
| Clamp the requested target into the reachable box | ✓ | `RuntimeController::TryScheduleTransition` | `include/pmg/RuntimeController.h:86` |
| Blend window equals the metric window | ✓ (D4) | `RuntimeControllerConfig::transition_blend_frames` | `include/pmg/RuntimeController.h:17` |
| Recompute alignment at runtime from the live clips | ✓ | `PointCloudAlignment` strategy | `include/pmg/RuntimeController.h:60` |
| Source plays through cycle-crossing blends | ✓ (D3) | `FoldCompletedCycles` | `include/pmg/RuntimeController.h:91` |
| Repeated unchanged self-edge streaming (align next clip, blend end→start) | ✓ | viewer request policy | `apps/viewer/PmgViewerWorkspace.cpp:417`; `docs/WALK_JOG_CONTINUITY.md` |
| Random graph walk | ✓ | `ChooseRandomOutgoingTransition` | `include/pmg/GoalDirectedLocomotion.h:87` |
| Goal-directed locomotion (thesis Ch. 6) | ◐ | `GoalDirectedLocomotion` (single steering axis) | `include/pmg/GoalDirectedLocomotion.h:48` |
| Multidimensional runtime control | ✗ | steers axis 0; remaining axes held at their midpoint | `src/GoalDirectedLocomotion.cpp:143` |

**Multidimensional control (✗).** This is the only genuine missing *feature*.
The motion-space and blend layers are N-dimensional (D1 vector calibration,
viewer N-D authoring, and a 2-D `walk_curvature_speed` artifact load and
stream). The control layer is scalar: `ParameterForRate(float)`,
`MinParameter().front()`, and a single-component request
(`src/GoalDirectedLocomotion.cpp:79,220`). A 2-D node therefore loads but only
its first axis is driven. Closing this means searching the N-D reachable box
against the vector calibration instead of inverting one scalar rate.

## Out-of-scope boundaries (○)

These belong to the surrounding Motion Graphs (2002) and KG04 (2004) systems,
not the PMG layer this project implements.

| Element | Status | Note |
|---|---|---|
| Automatic motion extraction / parameterization from large data sets (KG04) | ○ | corpus is hand-specified via `specs/*.pmg_spec` |
| Local-minimum transition harvesting over the full distance function | ○ (D8) | Motion Graphs database construction |
| Largest-SCC connectivity pruning | ○ (D8) | Motion Graphs database construction |
| Branch-and-bound graph-walk search | ○ (D8) | Motion Graphs global search |
| Constraint-annotation footskate cleanup | ○ (D8) | replaced by independent IK foot-lock (`include/pmg/FootLocking.h`) |
| Skinned-mesh point clouds, BVH export, learned validity, CUDA, mid-clip transitions | ○ | `docs/STATUS.md` optional future work |

## What's left, in priority order

1. **✗ Multidimensional runtime / goal-directed control** — the one missing
   feature that extends paper coverage. Highest value.
2. **◐ Registration fidelity** — KG04 registration curves vs. the
   contact-anchor + DTW approximation. Largest faithfulness gap; large effort,
   low marginal payoff on this corpus.
3. **◐ Spec-expose the distance-grid phase ranges and window size** — small
   plumbing that makes the §6.3 source-range restriction tunable per edge.
4. **◐ D6 metric exactness** — match Kovar's asymmetric window placement and
   unnormalized weighted sum if paper-comparable absolute distances are wanted.
   Low: does not change which transitions classify GOOD on this corpus.
5. **○ Everything else** — out-of-scope boundaries, correctly excluded per the
   Claim Limit.

The core algorithm is faithful and verified (38/38 tests). What remains is
breadth (multidimensional control) and registration depth, not correctness.

## Deviation register

Stable ids for the adaptations and gaps above.

### Resolved (formerly central-claim deviations)

- **D1 — Blend weights calibrated against a measured metric.**
  `ParametricMotionSpace` supports KG04-style sampled inversion. One-dimensional
  spaces retain monotone parameter-adjacent sampling; multidimensional spaces
  sample a deterministic authored-domain grid and store each generated metric
  vector with its full example weights. `ComputeLocalBlendWeights` maps the
  requested authored coordinate to anchor-interpolated measured space, then
  locally inverts normalized samples. Implemented metrics: `turn_rate`,
  `travel_speed`; nodes without declared metrics keep Shepard interpolation.
  Vector calibration serializes in `PMG_GRAPH_V7`; V5/V6 scalar tables remain
  readable.
- **D2 — Generated clip duration follows the parameter.**
  `GenerateClip(parameter, fps)` derives its frame count from
  `BlendedDurationSeconds`, so cycle time varies with the parameter. Builder and
  runtime use this path; frame-aligned diagnostics use the isolated
  `pmg::legacy::GenerateClipWithFrameCount` API. Spec distance thresholds were
  recalibrated because duration-true clips raise absolute point-cloud distances.
- **D3 — Source continues through cycle-crossing blends.**
  The runtime folds each completed source cycle into its accumulated world
  placement and resumes at the wrapped clip-local phase, so transitions play
  real source frames through the whole blend. A regression test forces a
  transition window across phase 1 and checks continuous root motion.
- **D4 — Blend length equals the metric window.**
  `transition_blend_frames` derives from the edge builds'
  `DistanceGridConfig::window_size` and drives runtime blending, point-cloud
  alignment, and goal-directed calibration. Artifacts whose edge builds record
  different window sizes are rejected explicitly (one global runtime window).
- **D5 — Transition phases remain target-dependent.**
  Each source sample stores the phase pair measured at every retained GOOD
  target sample inside its shrunk box. Runtime clamps the requested target,
  interpolates phases in target-parameter space, then across source samples.
  Introduced in `PMG_GRAPH_V6`, retained in V7; V2–V5 scalar phases remain
  readable as fallback.

### Known adaptations and gaps

- **D6 — Point-cloud sampling and scale differ from Kovar 2002 (low).**
  Kovar builds point clouds from downsampled skin-mesh vertices, uses a source
  window beginning at the candidate frame and a target window ending at the
  candidate frame, and defines an unnormalized weighted squared sum. This
  implementation uses joint world positions, centered endpoint-clamped windows
  for both clips, and a weighted mean squared distance. The closed-form floor
  alignment is the same optimization, but thresholds and distance values are
  specific to this implementation and corpus.
- **D7 — Optional velocity weighting is in neither paper (low).**
  `PointCloudWeighting::add_velocity_weight` multiplies point weights by
  `1 + speed`. Off by default; enabling it departs from the printed metric.
- **D8 — Kovar 2002 graph machinery is out of scope (informational).**
  Local-minima transition harvesting, largest-SCC pruning, branch-and-bound
  path search, and constraint-annotation footskate cleanup belong to the
  original Motion Graphs system, not the PMG layer. Foot locking here is an
  independent IK post-process.

## Local extensions

- Contact registration and DTW refinement instantiate the smooth registered
  motion-space assumption the paper relies on (see the §3 registration note).
- Root-delta blending and optional foot locking address observed BVH artifacts.
- Complete (V7) artifacts and structured build reports add reproducibility not
  specified by the original paper.

## Claim limit

The code estimates transition validity and control behavior under this corpus,
distance metric, registration, sampling, and runtime configuration. It does not
establish general motion quality, global controllability, or reproduction of
the paper's unavailable datasets.
