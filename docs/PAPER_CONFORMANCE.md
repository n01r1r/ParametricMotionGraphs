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

Deviation ids (`D1`–`D8`) are stable and referenced from `README.md` and
`docs/adr/0003-parameter-accuracy-calibration.md`.

## §3 — Motion spaces

| Paper element | Status | Implementation | Source |
|---|---|---|---|
| Motion space = blend of registered example motions | ✓ | `ParametricMotionSpace` | `include/pmg/ParametricMotionSpace.h:62` |
| Blend space holds one motion family (structurally similar examples only) | ✓ | `RequireBlendableMotionFamily` rejects acyclic / contact-structure-mismatched multi-example spaces | `src/MotionRegistration.cpp`; `tests/test_motion_registration` |
| Scattered-data blend weights, `k = dim + 1` nearest, k-th-neighbor cutoff (Eq. 2) | ✓ | `ComputeLocalBlendWeights` | `src/ParametricMotionSpace.cpp:556`; `docs/adr/0002-pmg-knn-cutoff.md` |
| Parameter-accurate inverse (authored coordinate → weights that achieve it) | ✓ (D1) | `CalibrateParameterMetrics` / `ParameterCalibration` | `include/pmg/ParametricMotionSpace.h:50` |
| Blend inherits example timing (cycle length varies with the parameter) | ✓ (D2) | `BlendedDurationSeconds` → `GenerateClip` | `include/pmg/ParametricMotionSpace.h:99` |
| Time registration so blends combine corresponding moments | ✓ | contact-anchor `TimeWarp`, slope-constrained DTW refine, **cubic smoothing-spline** registration curve | `include/pmg/MotionRegistration.h:47` |
| KG04 registration curves (per-frame rigid alignment + constraint matching) | ◐ | per-frame rigid alignment approximated by root-delta integration | `include/pmg/ParametricMotionSpace.h:106` |

**Registration fidelity.** KG04 builds a smooth registered motion space from
registration *curves*: a dynamic-time-warp correspondence, smoothed by a cubic
B-spline, plus per-frame rigid frame alignment and constraint matching. The
timewarp `s(u)` now matches that: the dense slope-constrained DTW correspondence
is denoised by a cubic smoothing spline (penalized second-difference form, the
natural-cubic-smoothing-spline equivalent of KG04's B-spline) before it is
sampled into warp knots — see `RefineRegistrationByDtw`. The spline denoises the
*placement* of the warp knots; `TimeWarp::Evaluate` still interpolates
piecewise-linearly between them, so the stored `s(u)` is C0 (position-continuous)
but **not C1** — its slope jumps at every knot, which under blending surfaces as
a phase-velocity discontinuity. Denser knots shrink those jumps but do not
remove them; delivering C1 at evaluation time would need a monotone-spline
`Evaluate`, not only a spline *fit* over the knots. Under the superseded
centered mean-distance diagnostic, this lowered production best self-transition
distance from `0.8878` (prior piecewise-linear refine) to `0.8816` against a
`0.8604` no-registration baseline, with runtime pop ratio flat. D6 replaced
that diagnostic scale and window placement. Under the exact asymmetric raw-sum
metric (`specs/legacy_walk_curvature`, `--validate-graph`, seed 7), current
production/authored mean-min distances are `156.654 / 145.385 = 1.0775`; the
regression gate records that corpus-specific penalty explicitly and caps it at
`1.10`. All 42 configured CTest checks pass as of 2026-06-16, including
`test_transition_quality`. The payoff is corpus-density-coupled: with only
three clean walk clips the within-segment correspondence is already near-linear,
so headroom is small.

What remains ◐ is the rest of the KG04 registration *curve*: per-frame rigid
pose alignment and constraint matching, here approximated by root-delta
integration. The corpus has no skin mesh, so this is accepted as a local
extension that holds on the included clips.

**Motion-family premise (enforced).** A parametric blend space is only defined
over structurally similar, registered example motions — one *motion family*
(Heck-Gleicher §3; KG04 extracts families before blending). Two different
families (for example a walk loop and a vault) are joined by a *transition edge*
between separate graph nodes, never by interpolating them as two examples of one
space: blending across families produces a raw-phase average whose family-unique
events (a high arm swing the other family lacks) read as a jolt, and contact
registration aligns feet, not such events. `RequireBlendableMotionFamily`
enforces this for any space with two or more examples — each must be a single
looping cycle sharing one contact structure — so a cross-family or non-looping
example fails the build (and the interactive authoring path) with guidance to
use an edge instead, rather than silently blending. Single-example nodes are
exempt because they never blend. This is the structural counterpart to the
runtime-seam analysis in [`WALK_JOG_CONTINUITY.md`](WALK_JOG_CONTINUITY.md): that
document's jolt is a same-family periodicity limit; this guard prevents the
distinct cross-family case from being authored at all.

## §4 — Parametric motion graph construction

| Paper element | Status | Implementation | Source |
|---|---|---|---|
| Node = motion space, edge = sampled transition set | ✓ | `PmgNode` / `PmgEdge` | `include/pmg/ParametricMotionGraph.h:12` |
| Sample source × target parameter spaces | ✓ | `PmgBuilder` (50 source / 1000 target default) | `include/pmg/PmgBuilder.h:13` |
| Transition distance = windowed point cloud (Kovar 2002) | ◐ (D6) | `MotionDistance::BuildDistanceGrid` | `include/pmg/MotionDistance.h:101` |
| Closed-form 2-D floor-plane alignment | ✓ | `AlignedPointCloudDistance` | `include/pmg/MotionDistance.h:94` |
| Optimal transition cell of the grid | ✓ | `FindOptimalTransition` | `include/pmg/MotionDistance.h:107` |
| GOOD / NEUTRAL / BAD double threshold | ✓ | `PmgBuilderConfig` | `include/pmg/PmgBuilder.h:20` |
| Enclose GOOD targets in an AABB, shrink to exclude BAD | ✓ | `ParameterAabb::ShrinkToExclude` | `src/PmgBuilder.cpp` |
| Reject the edge if any source sample has no reachable box | ✓ | `BuildEdgeWithReport` | `include/pmg/PmgBuilder.h:77` |
| Per-target transition phase (not a single scalar) | ✓ (D5) | `TargetTransitionPhaseSample` | `include/pmg/TransitionTypes.h` |
| Restrict the transition search to a source-phase sub-range (§6.3) | ✓ | `DistanceGridConfig` (default source `[0.70, 0.95]`, target `[0.05, 0.30]`), per-edge via `edge_phase_range` | `include/pmg/PmgBuilder.h:31`; `src/GraphSpec.cpp` |

**Source-range restriction (✓).** The mechanism is on by default and now
per-edge tunable: the `.pmg_spec` `edge_phase_range <source> <target>
<src_start> <src_end> <tgt_start> <tgt_end>` line sets the search sub-range
(the builder and `GraphIo` already consumed and serialized
`DistanceGridConfig`'s phase fields). The metric window size stays a single
global value by D4 (the runtime requires one blend window across all edges), so
it is intentionally not per-edge. Widening the range is documented as
*ineffective* against the wide-turn walk-loop jolt — it admits degenerate
same-phase transitions; see `docs/WALK_JOG_CONTINUITY.md`.

## §5 — Runtime and control

| Paper element | Status | Implementation | Source |
|---|---|---|---|
| Stream a graph walk; select an outgoing edge | ✓ | `RuntimeController` | `include/pmg/RuntimeController.h:55` |
| Interpolate the target box + phases at the source parameter (Eqs. 1–3) | ✓ | `PmgEdge::LookupInterpolated` | `include/pmg/ParametricMotionGraph.h:36` |
| Clamp the requested target into the reachable box | ✓ | `RuntimeController::TryScheduleTransition` | `include/pmg/RuntimeController.h:86` |
| Blend window equals the metric window | ✓ (D4) | `RuntimeControllerConfig::transition_blend_frames` | `include/pmg/RuntimeController.h:17` |
| Recompute alignment at runtime from the live clips | ✓ | `PointCloudAlignment` strategy | `include/pmg/RuntimeController.h:60` |
| Source plays through cycle-crossing blends | ✓ (D3) | `FoldCompletedCycles` (now bidirectional: forward fold + negative pre-roll wrap) | `include/pmg/RuntimeController.h` |
| Repeated unchanged self-edge streaming (align next clip, blend end→start) | ✓ | viewer request policy | `apps/viewer/PmgViewerWorkspace.cpp`; `docs/WALK_JOG_CONTINUITY.md` |
| Schedule a transition when its source phase is crossed | ✓ | `CrossedPhase` (wrap-aware; replaces ~1-frame point-in-window) | `src/RuntimeController.cpp` |
| Cyclic self-edge target pre-roll keeps the previous-cycle tail | ✓ | `TransitionPreRollPolicy::kWrapCyclicSelfEdges` (default); cyclicity from `RuntimeControllerConfig::cyclic_nodes` (registration `cycle_joint`) | `include/pmg/RuntimeController.h` |
| n-way pose blend is order-independent | ✓ | `BlendPoseN` hemisphere-aligned weighted quaternion mean | `src/PoseBlend.cpp` |
| Random graph walk | ✓ | `ChooseRandomOutgoingTransition` | `include/pmg/GoalDirectedLocomotion.h:87` |
| Per-transition diagnostics (chosen edge, reachable box, clamped target, phases, D, pose pop, root speed, yaw rate, foot drift, classification) | ✓ | `RuntimeTransitionDiagnostics`; `TransitionQuality`; `pmg_cli --random-walk --dump-transitions-csv` | `include/pmg/RuntimeController.h`; `include/pmg/TransitionQuality.h`; `apps/PmgRuntimeCommands.cpp` |
| Goal-directed locomotion (thesis Ch. 6) | ◐ | `GoalDirectedLocomotion` (per-axis greedy steering; not branch-and-bound) | `include/pmg/GoalDirectedLocomotion.h:48` |
| Multidimensional runtime control | ✓ | per-axis calibration + inversion drives all axes (turn_rate heading, travel_speed pace) | `src/GoalDirectedLocomotion.cpp` |

**Multidimensional control (✓).** `GoalDirectedLocomotion` steers every node
axis. It calibrates one inverse map per axis by streaming the real runtime at
swept parameter values and measuring the achieved metric (turn rate or travel
speed); `RequestForPose` then drives the `turn_rate` axis from heading error and
each `travel_speed` axis from a cruise/arrival pace policy, assembles the full
vector, and clamps it to the reachable box (`src/GoalDirectedLocomotion.cpp`).
Axis metrics resolve from the node's parameter calibration (or an explicit
config override); a one-dimensional `turn_rate` node reduces to the prior
single-axis behavior exactly. The CLI `--goto` and the viewer goto both drive
the full vector. Verified by `test_goal_directed_locomotion` (a 2-D node with
both axes driven) and the `cli_goto_walk_2d` end-to-end smoke test on the real
`demo_walk_2d_triangle` artifact. The remaining `◐` on goal-directed control is
the local-greedy vs. branch-and-bound search adaptation, not dimensionality.

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
| Skinned-mesh point clouds, BVH export, learned validity, CUDA, mid-clip transitions | ○ | optional future work |

## What's left, in priority order

1. **◐ Registration depth** — the timewarp `s(u)` knot *fit* is now a cubic
   smoothing spline over the dense DTW correspondence (KG04-faithful, measured to
   beat the prior linear warp; see §3 above). The evaluated warp is still
   piecewise-linear between knots, so `s(u)` is C0 but not C1 — slope jumps at
   knots persist (shrinking with knot density); a monotone-spline `Evaluate`
   would be needed to close C1 at sampling time. What is still approximated
   beyond timing is the rest of KG04's registration curve — per-frame rigid
   pose alignment and constraint matching — which root-delta integration stands
   in for. Closing it fully needs skinned/constraint data the corpus lacks.
   Note: an earlier attempt to cubic-*interpolate* the sparse contact anchors
   was a measured regression (it injects unsupported curvature); the spline
   belongs in the *fit* over the dense correspondence, not in the interpolation
   primitive.
2. **○ Everything else** — out-of-scope boundaries, correctly excluded per the
   Claim Limit.

Spec-exposing the distance-grid phase ranges (formerly item 2) landed via the
`edge_phase_range` line; the metric window size stays one global value by D4.
The D6 equation/window gap also landed: transition grids use source-start and
target-end windows plus Kovar's unnormalized weighted squared sum. Thresholds
were recalibrated because raw sums scale with point count and weights.
The wide-turn walk-loop jolt was diagnosed as **mostly** a corpus periodicity
limit, but a 2026-06-15 continuity audit isolated a deterministic slice: the
self-edge target pre-roll was clamped at clip start, dropping the previous-cycle
tail. The default now wraps that pre-roll for cyclic nodes
(`TransitionPreRollPolicy::kWrapCyclicSelfEdges`, bidirectional
`FoldCompletedCycles`), cutting wide/mid self-edge `pop_ratio` ~19% / ~16%; the
tight residual stays D4-window + corpus. Cyclicity is metadata
(`RuntimeControllerConfig::cyclic_nodes`, derived from each node's registration
`cycle_joint`), so a non-cyclic self-edge (e.g. a punch or duck space) clamps
instead of wrapping into undefined pre-start frames. Two further runtime-continuity
gaps closed in the same pass: transition scheduling moved from a ~1-frame
point-in-window test to a wrap-aware `CrossedPhase` (no skipped self-edge on a
variable-dt or fast-forward frame), and `BlendPoseN` replaced order-dependent
sequential SLERP with a hemisphere-aligned weighted quaternion mean
(permutation-invariant for ≥3-example blends). See `docs/WALK_JOG_CONTINUITY.md`.

Multidimensional runtime control (formerly item 1) landed: the control layer
now drives every node axis. The core algorithm is faithful and verified (38/38
core tests, 40/40 with the viewer). What remains is registration depth and small
per-edge plumbing, not correctness or feature breadth.

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
  Because the duration is parameter-dependent, the viewer's blend playback runs
  on a **canonical-phase clock** (`[0,1)`), not a seconds clock: phase is the
  structural position and is held fixed across a parameter change, while only the
  advance rate (`1 / duration`) varies. A seconds clock would instead recompute
  `phase = seconds / duration` and teleport the pose the moment the parameter
  moved.
- **D3 — Source continues through cycle-crossing blends.**
  The runtime folds each completed source cycle into its accumulated world
  placement and resumes at the wrapped clip-local phase, so transitions play
  real source frames through the whole blend. A regression test forces a
  transition window across phase 1 and checks continuous root motion.
- **D4 — Metric window vs blend placement are two concerns.**
  `transition_blend_frames` derives from the edge builds'
  `DistanceGridConfig::window_size`; `k` sampled frames span `k-1` intervals so
  the blend lasts `(k-1)/fps`. Artifacts with mixed window sizes are rejected
  (one global runtime window). The transition window splits into two source-paper
  concerns resolved through the shared `ResolveTransitionFrameWindows`:
  - **Metric (Kovar §3.1):** directional windows, source `[i, i+k-1]` / target
    `[j-k+1, j]`. PMG reuses Kovar's metric; the asymmetry aligns the end→start
    seam and the calibrated sub-ranges/thresholds depend on it. A *centered*
    metric rejects 3/4 walk_jog edges at their thresholds (up to 3.4× distance
    inflation over the directional-tuned sub-range), so the metric stays
    `kKovarDirectional`.
  - **Blend (PMG §5.2.1):** the runtime centers the blend window on the optimal
    transition point (`kPmgCentered` default). The centered blend window
    `[ref-h, ref+h]` differs from the metric window by half a window, so the
    blended frames are not exactly the metric-scored frames — an inherent
    consequence of pairing Kovar's metric with PMG's centered blend (in pure PMG
    they coincide). Measured smoother on wide/mid self + cross-node, slightly
    worse on tight self.

  Serialized as `PMG_GRAPH_V8` (stores the metric convention); legacy V2–V7 pin
  `kKovarDirectional`. A/B via `pmg_cli --random-walk --transition-convention
  <metric> --blend-placement <runtime>`; `test_transition_window_contract`
  regresses the resolver.
- **D7 — Transition distance D carried to runtime.**
  Each `TransitionSample` stores `transition_distance`, the mean Kovar metric
  distance over the GOOD targets retained inside its reachable box.
  `LookupInterpolated` carries it through the same kNN source weighting into
  `InterpolatedTransition::distance` and `RuntimeTransitionDiagnostics`, so a
  streamed transition is tagged with the offline edge quality it was scheduled
  from (observable against runtime pop via `--dump-transitions-csv`). Serialized
  as `PMG_GRAPH_V9`; V2–V8 read back zero.
- **Transition-quality classification is diagnostic, not a paper metric.**
  `TransitionQuality` measures three intervals before and after a scheduled
  transition by default; the CLI expands this to the runtime blend-frame count
  when larger. It separates local pose pop, root-speed discontinuity, signed
  yaw-rate discontinuity, and contact-foot drift. A `0.05 rad/s` yaw deadband
  suppresses near-zero ratio inflation. `--dump-transitions-csv` writes these
  fields and before/after contact states beside `D`. `cyclic_seam_mismatch` is
  reserved until the CyclicContinuity contract supplies raw seam evidence.
  Thresholds are diagnostic defaults, not universal perceptual limits, and do
  not participate in PMG edge construction or default runtime scheduling.
  An opt-in CLI-only `--quality-gate` prospectively simulates a candidate and
  skips it by root speed, yaw rate, contact drift, contact mismatch, or
  insufficient data. This is an ablation adapter, disabled by default, and is
  not claimed as a paper algorithm or production controller.
- **D5 — Transition phases remain target-dependent.**
  Each source sample stores the phase pair measured at every retained GOOD
  target sample inside its shrunk box. Runtime clamps the requested target,
  interpolates phases in target-parameter space, then across source samples.
  Introduced in `PMG_GRAPH_V6`, retained in V7–V9; V2–V5 scalar phases remain
  readable as fallback.

### Known adaptations and gaps

- **D6 — Point representation differs from Kovar 2002 (low).**
  Source windows now begin at candidate `i`, target windows end at candidate
  `j`, and distance is Kovar Equation 1's unnormalized weighted squared sum.
  The remaining adaptation is representation: joint world positions replace
  downsampled skin-mesh vertices, and directional windows clamp at clip
  boundaries. Thresholds remain specific to skeleton point count, window,
  weights, native units, and corpus.
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
- Complete (V9) artifacts and structured build reports add reproducibility not
  specified by the original paper.

## Claim limit

The code estimates transition validity and control behavior under this corpus,
distance metric, registration, sampling, and runtime configuration. It does not
establish general motion quality, global controllability, or reproduction of
the paper's unavailable datasets.

Spec-level conformance is narrower still: a spec that parses and builds may be
a minimal fixture or deliberate stress case rather than a meaningful motion
family. [`SPEC_AUDIT.md`](SPEC_AUDIT.md) classifies every current spec and
records single-example, sparse-domain, threshold, and parameter-semantic limits.
In particular, runtime clamps to the interpolated transition **AABB**, not the
convex hull of authored examples. The triangular sample layout in
`demo_walk_2d_triangle.pmg_spec` therefore does not imply triangular reachable
support; the 2026-06-15 CSV evidence is recorded in
`outputs/pmg_demo/PMG_SPEC_VALIDATION_REPORT.md`.
