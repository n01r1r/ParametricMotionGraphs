# ParametricMotionGraphs

A paper-core implementation of Parametric Motion Graphs: a parametric motion
space per action, connected by sampled transition edges, streamed at runtime
with aligned, blended transitions. Documented adaptations include joint-based
point clouds, a directional (Kovar §3.1) transition metric paired with a
PMG-centered (§5.2.1) runtime blend, and manual BVH parameterization. This
file fixes the vocabulary so code and conversation stay aligned.

## Language

### Motion data

**MotionClip**:
An ordered sequence of poses at a fixed frame rate. Sampled by normalized phase.
_Avoid_: animation, take, sequence.

**Pose**:
A root position plus one local rotation per joint, in clip-local space.
_Avoid_: frame (a frame is an index into a clip, not the pose itself).

**Phase**:
Normalized clip time in [0, 1]. The shared clock across blending and transitions.
_Avoid_: time, t, progress.

### Parametric blending

**ParametricMotionSpace**:
One action (e.g. "walk") as a set of example clips keyed by a parameter vector;
evaluates an interpolated pose for any parameter, sampling each example at its
registered (time-warped) phase. Weights come from the parameter calibration
when one is installed, otherwise from the local k-nearest Shepard stencil.
_Avoid_: blend tree, blendspace, motion set, inverse-distance weighting (the
old global scheme, replaced by the local stencil).

**ParameterCalibration**:
The KG04-style sampled inverse map from measured parameter vectors to full
example-weight vectors. Built offline by `CalibrateParameterMetrics` from one
declared `ParameterMetric` per parameter axis (`turn_rate` in rad/s or
`travel_speed` in native BVH units/s). Runtime first anchor-interpolates the
requested authored coordinate into measured space, then locally inverts the
normalized measured-space samples.
_Avoid_: weight LUT, steering calibration (that is the goal-directed module's
separate streamed-rate table).

**Blended duration**:
The cycle time of a generated clip: the weighted sum of the examples' clip
durations at the evaluated parameter. GenerateClip derives its frame count
from this, so a tight turn and a straight walk play at their own cadence.
_Avoid_: fixed frame count except through the isolated
`pmg::legacy::GenerateClipWithFrameCount` diagnostic API.

**ParameterVector**:
The control coordinates of a motion space (e.g. turn rate, speed).
_Avoid_: feature, input, control vector.

**Motion-space preparation**:
The offline Module that loads GraphSpec examples against one compatible
Skeleton, optionally extracts one cycle, installs contact Registration and DTW
refinement, and calibrates the declared ParameterMetric. It exposes authored
and registered snapshots for diagnostics while the production snapshot is the
one inserted into BuiltPmgArtifact.
_Avoid_: spec loading, diagnostic space build, registration pipeline.

### Registration

**ContactInterval**:
A maximal run of frames where one joint is planted on the floor (low height,
low world speed). Its strike and lift phases are the anchors of registration.
_Avoid_: footstep event, lock window.

**TimeWarp**:
A monotonic piecewise-linear phase-to-phase map pinned at 0 and 1, built from
matched anchor lists. One per example, mapping canonical phase onto that
example's own phase.
_Avoid_: time scaling, retiming curve.

**Canonical phase**:
The shared phase domain of a registered motion space; each contact anchor sits
at the mean of the examples' anchor phases. `EvaluatePose` takes canonical
phase and warps it per example before sampling.
_Avoid_: master clock, reference time.

**Registration**:
Detecting contacts on every example of a space, matching their anchor
structure, and installing per-example TimeWarps so blends combine structurally
corresponding moments (strike with strike, swing with swing).
_Avoid_: alignment (reserved for the rigid floor transform at transitions).

**Root-delta blending**:
How GenerateClip moves the root: blend each example's per-frame floor step in
its own heading frame, then integrate. Absolute root positions are never
averaged across examples (that bends arcs and drags planted feet).
_Avoid_: root blending, trajectory averaging.

**DTW refinement**:
A second registration pass: inside each canonical segment, slope-constrained
dynamic time warping (steps (1,1)/(1,2)/(2,1)) matches every example to the
example nearest the parameter centroid and inserts interior warp knots.
Contact anchors are hard constraints; only the timing between them moves.
_Avoid_: full DTW, unconstrained warping (flat path runs fabricate timing
jumps and were measurably worse).

**Foot locking**:
Post-process on a generated clip: detect contact intervals, then pin each
contact joint at its strike position with analytic two-bone IK (bend at the
parent, swing at the grandparent, foot world orientation preserved), eased
over blend frames at interval boundaries. For blended output only; source
mocap is not locked.
_Avoid_: foot IK pass, foot sliding fix (the metric is slide rate; locking is
the mechanism).

### Graph

**ParametricMotionGraph**:
Nodes are motion spaces; edges carry sampled transitions between them.
_Avoid_: state machine, blend graph.

**BuiltPmgArtifact**:
The complete offline output consumed online: Skeleton, registered
ParametricMotionGraph, parameter calibrations, runtime sampling rate, source
paths, registration settings, edge sampling configuration, seeds, and build
reports. Serialized as PMG_GRAPH_V7 (V2-V6 stay readable).
_Avoid_: graph file, cached graph (those omit the complete runtime contract).

**TransitionSample**:
One sampled source parameter with the reachable target parameter box and phase
pairs measured at retained GOOD target samples. Runtime evaluates this phase
field at the requested target. No alignment is stored; alignment is resolved
through `AlignmentStrategy`.
_Avoid_: edge point, transition record, baked alignment (nothing is baked).

### Alignment & placement

**RigidTransform2D**:
The single representation of a 2D rigid floor transform — yaw about +Y then an
(x, z) translation. Owns the rotation convention, pose application, and
composition. Used as a recovered alignment (metric/runtime) and the accumulated
world placement (runtime). PMG edges do not store an alignment.
_Avoid_: RigidAlignment2D, WorldTransform2D, alignment floats (all unified into this).

**AlignmentStrategy**:
The seam for how the runtime aligns a chosen target clip onto the current clip
at a transition. Resolves a `RigidTransform2D` from an `AlignmentContext`.
_Avoid_: alignment mode, alignment flag.

**PointCloudAlignment / RootOnlyAlignment**:
The two adapters behind `AlignmentStrategy`: recompute the Kovar-derived
joint-point-cloud alignment, or recompute from the root pose alone
(legacy/debug, skeleton-free tests).
_Avoid_: alignment kind, strategy enum, baked alignment.

### Runtime

**RuntimeController**:
Streams motion from a graph: advances the current clip, schedules a transition
when the source phase reaches the gate, and aligns + blends into the target,
accumulating the world placement so motion stays continuous. Takes an
`AlignmentStrategy` at construction. Blend length is measured in frames and
must match the artifact edge metric's `window_size`; completed source cycles
are folded into world placement so cycle-crossing blends never freeze.
_Avoid_: player, driver, animator.

**Goal-directed locomotion**:
The semantic control Module that maps a world-space target position and optional
final facing direction to a node ParameterVector. It calibrates against achieved
runtime turn rates rather than assuming authored clip curvature equals streamed
behavior.
_Avoid_: goto hack, steering helper.

### Viewer

**ViewerWorkspace**:
The compile-time Interface between the algorithm-neutral ViewerHost and an
algorithm-specific Adapter. A workspace owns algorithm state and diagnostics,
updates them once per frame, consumes optional world-space pointer rays, and
produces a neutral RenderScene. The ViewerHost owns camera, rendering, and
window input; it does not include or link PMG Implementation.
_Avoid_: viewer app, PMG viewer core, runtime plugin (loading is compile-time).

### Distance metric

**Point-cloud metric**:
The windowed, closed-form aligned point-cloud distance (Kovar et al. 2002) used
to score transitions. The one live transition distance.
_Avoid_: pose distance (the older single-frame metric, removed).

**DistanceGrid**:
Aligned point-cloud distance for every sampled (source frame, target frame) pair
over the transition region; its minimum cell is the optimal transition.
_Avoid_: cost matrix, heatmap (the heatmap is the viewer's rendering of it).

## Paper symbols

Mapping from the source papers' notation to the repository equivalents.

| Paper symbol | Meaning | Repository equivalent |
|---|---|---|
| `M(t)` | motion evaluated at time | `MotionClip::SampleNormalizedPhase` |
| `P(lambda)` | parameterized motion generator | `ParametricMotionSpace::GenerateClip` |
| `D(M1(t1), M2(t2))` | aligned window distance | `AlignedPointCloudDistance` |
| `Ls`, `Lt` | sampled source/target parameters | builder sample vectors |
| `Lt_GOOD`, `Lt_BAD` | classified target samples | `good_hits`, `bad_target_parameters` |
| `B(Ns, Nt)` | interpolated target transition region | `target_parameter_box` |
| `w_i` | PMG source-neighbor weight | `weights[n]` in edge lookup |
| `TGOOD`, `TBAD` | transition thresholds | `good_transition_threshold`, `bad_transition_threshold` |

`turn_rate` measures rad/s; `travel_speed` measures root floor-path speed in
native BVH units/s. `TGOOD`/`TBAD` are corpus- and metric-specific
squared-distance thresholds with no universal physical interpretation.
