# ParametricMotionGraphs

A paper-faithful implementation of Parametric Motion Graphs: a parametric
blend space per action, connected by sampled transition edges, streamed at
runtime with aligned, blended transitions. This file fixes the vocabulary so
code and conversation stay aligned.

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
evaluates an interpolated pose for any parameter via local k-nearest blend
weights, sampling each example at its registered (time-warped) phase.
_Avoid_: blend tree, blendspace, motion set, inverse-distance weighting (the
old global scheme, replaced by the local stencil).

**ParameterVector**:
The control coordinates of a motion space (e.g. turn rate, speed).
_Avoid_: feature, input, control vector.

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

### Graph

**ParametricMotionGraph**:
Nodes are motion spaces; edges carry sampled transitions between them.
_Avoid_: state machine, blend graph.

**TransitionSample**:
One sampled source parameter with the reachable target parameter box, the
source/target transition phases, and the baked alignment.
_Avoid_: edge point, transition record.

### Alignment & placement

**RigidTransform2D**:
The single representation of a 2D rigid floor transform — yaw about +Y then an
(x, z) translation. Owns the rotation convention, pose application, and
composition. Used as a recovered alignment (metric), a baked alignment (edge),
and the accumulated world placement (runtime).
_Avoid_: RigidAlignment2D, WorldTransform2D, alignment floats (all unified into this).

**AlignmentStrategy**:
The seam for how the runtime aligns a chosen target clip onto the current clip
at a transition. Resolves a `RigidTransform2D` from an `AlignmentContext`.
_Avoid_: alignment mode, alignment flag.

**PointCloudAlignment / RootOnlyAlignment**:
The three adapters behind `AlignmentStrategy`: recompute the exact point-cloud
alignment (paper-faithful), reuse the alignment baked on the edge, or recompute
from the root pose alone (legacy/debug).
_Avoid_: alignment kind, strategy enum.

### Runtime

**RuntimeController**:
Streams motion from a graph: advances the current clip, schedules a transition
when the source phase reaches the gate, and aligns + blends into the target,
accumulating the world placement so motion stays continuous. Takes an
`AlignmentStrategy` at construction.
_Avoid_: player, driver, animator.

### Distance metric

**Point-cloud metric**:
The windowed, closed-form aligned point-cloud distance (Kovar et al. 2002) used
to score transitions. The one live transition distance.
_Avoid_: pose distance (the older single-frame metric, removed).

**DistanceGrid**:
Aligned point-cloud distance for every sampled (source frame, target frame) pair
over the transition region; its minimum cell is the optimal transition.
_Avoid_: cost matrix, heatmap (the heatmap is the viewer's rendering of it).
