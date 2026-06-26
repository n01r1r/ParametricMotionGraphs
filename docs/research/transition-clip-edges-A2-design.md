# Tweak A design: transition-clip bridge edges (path A2)

Date: 2026-06-26. Status: **designed, NOT implemented** (recorded for later). Decision:
build **A2** (edge carries + plays a real bridge clip). A1 (bridge-node spec trick)
**rejected** — see the open question below. Companion to
[`registration-and-transition-edges.md`](registration-and-transition-edges.md) §"Tweak A".

This note pins *why* the obvious small versions of Tweak A cannot work, then specs
the build that can. Nothing here is wired yet.

## 1. How a transition is built and run today (no clip is ever used)

Two stages, neither uses a clip that *contains* the transition (root R1).

**(a) Build time — the edge is synthesized.** `edge walk jog` →
`PmgBuilder::BuildEdgeWithReport` looks only at the two cyclic node spaces and
searches for the crossover phases `(phi_s, phi_t)` minimizing the Kovar
point-cloud distance (Kovar et al. 2002):

```
D(A,B) = min over (theta, x0, z0) of  sum_i  w_i * || p_i - T(theta,x0,z0) q_i ||^2
```

- `p_i` = points of the source pose-window generated at source phase `phi_s`;
  `q_i` = points of the target pose-window at `phi_t`.
- `T` = a 2D ground-plane rigid transform (yaw `theta` + translation). So `D` is
  "best-aligned residual pose difference" between the two windows.

What the edge stores is **metadata, not frames** (`include/pmg/TransitionTypes.h`):

```cpp
struct TransitionSample {
    ParameterVector source_parameter;       // which source parameter we leave from
    ParameterAabb   target_parameter_box;   // which target region we land in
    float source_transition_phase = 0.85f;  // phi_s
    float target_transition_phase = 0.15f;  // phi_t
    float transition_distance = 0.0f;        // the D above (edge quality score)
    std::vector<TargetTransitionPhaseSample> target_phase_samples;
};
```

**(b) Run time — source/target are cross-faded.** During a transition the runtime
keeps *both* clips advancing and weighted-averages them; there are no stored bridge
frames (`src/RuntimeController.cpp:221-246`):

```cpp
float linear_alpha = elapsed / duration;                       // 0 -> 1
float alpha = linear_alpha*linear_alpha*(3 - 2*linear_alpha);  // smoothstep (C1)
Pose source_world = SampleWorld(current_clip_, t_src, ...);     // source@phi_s, advancing
Pose target_world = SampleWorld(next_clip_,    t_tgt, ...);     // target@phi_t, advancing
return BlendPose(source_world, target_world, alpha);           // (1-a)*src (+) a*tgt
```

i.e. each joint's mid-transition pose is

```
pose_j(alpha) = slerp( q_walk_j, q_jog_j, alpha ),   alpha : 0 -> 1
```

a direct interpolation of a source-manifold pose and a target-manifold pose.

## 2. Why this breaks across families (walk <-> jog)

Within a family the blend is fine because near pose pairs exist. Across families
they do not. Measured pose-seam distances (`fixture_walk_2d_jog_crossfamily` header;
`cmu_gait_graph` header):

```
pose-seam   jog <-> walk : 217 .. 246          walk <-> walk : 2.6 .. 50
best D      walk -> run  : ~2709  (self-edge ~250, so ~10x)
```

There is **no close pose pair inside the two cyclic spaces**. So
`(1-a)*walk (+) a*jog` traverses poses on *neither* manifold — not walk, not jog.
That off-manifold morph is the cross-family jolt (matches `cross-family-blend-guard`).
Lengthening the window only spreads the impossible poses over more frames; it does
not remove them.

## 3. Why the cheap versions of Tweak A fail

**Endpoint-only seeding** (derive `phi_s, phi_t, params` from the clip, emit a
`TransitionSample`, no model change) cannot help: the seed clip's first cycle *is* a
walk cyclic pose and its last cycle *is* a jog cyclic pose, so its endpoints live in
the same candidate set the synthesized search already minimizes over. `min D ~2709`
is a floor no endpoint choice clears. The clip's value is **not** its endpoints — it
is the **on-manifold intermediate frames** that actually connect walk to jog, and
those only help if **stored and played**:

```
walk (cyclic) --[ real walkToJog bridge frames ]--> jog (cyclic)
```

**A1 (bridge-node spec trick)** — making `walkToJog` its own node and routing
`walk -> bridge -> jog` via ordinary edges — was considered and **rejected**. It
leans entirely on the spec/node machinery treating a non-cyclic transition clip as a
"motion space" (phase registration, calibration, cyclic self-edge metric all assume
a loop; the bridge's end->start seam reintroduces the bad cross-family jump). The
deeper objection (user, 2026-06-26): this would be papering a runtime-model gap with
a spec encoding, and it raises a **fundamental doubt about whether the `.pmg_spec`
format itself is reliable** for expressing this at all (see Open questions). Do not
encode a planted transition as a degenerate node.

## 4. A2 — the edge carries and plays a real bridge clip (chosen)

The faithful Kovar planted-transition fix. The edge optionally owns the transition
frames; the runtime plays `source -> bridge -> target` instead of cross-fading.

**Model.**
```cpp
// include/pmg/ParametricMotionGraph.h
struct PmgEdge {
    int source_node = -1, target_node = -1;
    std::vector<TransitionSample> samples;
    std::optional<MotionClip> bridge;   // NEW: real transition frames, root-aligned
};
```

**Runtime.**
```cpp
// src/RuntimeController.cpp (transition execution)
if (edge.bridge) {
    // play the aligned bridge frames straight through; no cross-fade
    return SampleWorld(*edge.bridge, bridge_time, bridge_world_transform_);
}
// else: existing synthesized cross-fade (unchanged fallback)
```

**Build time.** Parse a new spec line and seed the bridge:
```
transition_edge <src> <tgt> <clip> [window]
```
- Load `<clip>` (e.g. `BVH/walkToJog.bvh`, 81f @30fps, ROOT Center).
- Trim to the bridge segment: from the last source-gait cycle end (~`phi_s`) to the
  first target-gait cycle start (~`phi_t`).
- Root-align the bridge so its first frame meets the source exit pose and its last
  frame meets the target entry pose (reuse `AlignmentStrategy`).
- Store frames in `edge.bridge`; keep a `TransitionSample` for lookup/scheduling.

**Serialization.** `GraphIo` must persist `bridge` frames in the artifact (new
version tag; absent => `nullopt` => old synthesized behavior, byte-identical).

**Scope.** Largest change in recent history: spec parse + builder trim/align +
`GraphIo` (de)serialize + runtime playback + evaluator. Touches the artifact format,
so version-bump + round-trip test required.

**Faithfulness / opt-in.** `transition_edge` is additive; a plain `edge` is
unchanged. Default path (no bridge) stays byte-identical. This is the direct,
honest fix for R1, not a tuning knob.

## 5. Evaluator / significance bar (A2)

```
pmg_cli ... --random-walk --steps N        # pop_ratio over traversed edges
```
`pop_ratio` = max per-transition pose pop / threshold (`local_pop_ratio`,
`src/TransitionQuality.cpp:191,301`). Compare **bridged edge vs synthesized edge**
on a walk<->jog graph (Center corpus: `walkCurve`/`jogCurve` nodes + `walkToJog`
bridge). Significant = cross-family `pop_ratio` drops clearly toward within-family
levels. If it does not clear the bar -> discard per faithful-first.

## 6. Open questions (recorded, not resolved)

- **Is `.pmg_spec` itself reliable enough to carry this?** (user, 2026-06-26) The A1
  rejection exposed unease that the text spec format may be the wrong/unreliable
  layer for expressing planted transitions (and possibly other constructs). Worth a
  separate audit before or alongside A2: what the format guarantees, where it
  silently degrades, whether `transition_edge` belongs in it or in a richer build
  description. A2's spec surface is deliberately one additive line to limit exposure
  to this doubt.
- Bridge alignment convention vs the existing directional-phase metric/centered-blend
  split ([`transition-window-contract`]) — confirm the bridge endpoints honor the
  same convention the runtime schedules on.
- CMU 16_08 (run->stop) is only `.amc`, not converted; walkToJog is the only ready
  Center-corpus seed. A second seed would strengthen the measurement.
