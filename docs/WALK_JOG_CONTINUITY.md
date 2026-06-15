# Walk/Jog Continuity and Control Audit

Last updated: 2026-06-16.

## Purpose

Explain the visible walk loop interruption in
`specs/demo_walk_jog_topology.pmg_spec`,
quantify it, define the `jog` and desired-parameter contracts, and separate
paper-faithful behavior from useful but non-equivalent reference code.

## Finding

The remaining walk interruption was caused by viewer request policy, not
world-space root placement.

The viewer stopped requesting the active node's self-edge when node and
parameter were unchanged. `RuntimeController` then raw-wrapped the generated
cycle. The extracted walk cycles are not joint-space periodic at their first
and last samples, so cycle folding preserves root travel but exposes a pose and
velocity seam.

The PMG runtime described by Heck and Gleicher instead repeatedly traverses the
self-edge. Near the source transition phase it aligns the next generated clip
and blends into its early phase over a short centered window. The apparent
phase reset is intentional clip concatenation; the blend is what hides it.

## Numerical Check

Setup:

- graph: `specs/demo_walk_jog_topology.pmg_spec`;
- runtime rate: 30 frames/s;
- sample duration: 20 seconds;
- transition blend: 5 frames, cubic smoothstep;
- pose step: mean Euclidean world-position displacement over all joints;
- acceleration: mean norm of the per-joint second finite difference;
- reported ratio: maximum divided by sequence median.

| Walk parameter | Playback | Max step / median | Max acceleration / median | Transitions |
|---:|---|---:|---:|---:|
| 0.0 | raw cycle wrap | 2.60 | 21.18 | 0 |
| 0.0 | self-edge blend | 1.66 | 4.73 | 33 |
| 0.5 | raw cycle wrap | 2.73 | 21.99 | 0 |
| 0.5 | self-edge blend | 1.65 | 4.47 | 33 |
| 1.0 | raw cycle wrap | 3.56 | 6.78 | 0 |
| 1.0 | self-edge blend | 1.99 | 2.68 | 31 |

The self-edge path lowers the worst acceleration ratio by 60-80% and the worst
pose-step ratio by 40-50%. This supports restoring repeated self-edge requests
in the viewer. It does not prove perceptual smoothness.

### Source-cycle seams

The same diagnostic applied directly to contact-extracted cycles:

| BVH | Skeleton group | Seam step / median | Seam acceleration / median | Assessment |
|---|---|---:|---:|---|
| `walkCurve` | B | 3.04 | 17.06 | current wide-turn anchor; raw wrap poor |
| `walkMoreCurve` | B | 3.11 | 17.85 | current middle anchor; raw wrap poor |
| `walkTightCurve` | B | 2.55 | 9.91 | current tight-turn anchor |
| `walkSpiral` | B | 1.84 | 10.99 | possible added curvature sample |
| `walkFastSpiral` | B | 9.48 | 44.08 | reject as loop anchor without cleanup |
| `walkStraightTwiceAsFast` | B | 1.67 | 3.23 | useful speed sample, not curvature-equivalent |
| `jogCurve` | B | 1.25 | 4.33 | current jog anchor |
| `walkToJog` | B | 4.56 | 28.60 | transition clip, not cyclic node example |
| `smoothWalk` | C | 1.35 | 7.57 | smoother step seam but incompatible skeleton |

`smoothWalk` cannot be mixed into the Group-B walk/jog space without explicit
retargeting. Adding an incompatible BVH would change bone offsets and violate
the blend contract. `walkSpiral` is the only immediate Group-B curvature
candidate worth a separate calibrated experiment; it should not replace an
anchor based on seam score alone.

### Blend-window experiment

Increasing the transition window was tested with permissive diagnostic
thresholds so edge rejection did not confound the comparison.

| Parameter | 5-frame acceleration ratio | 7-frame | 9-frame |
|---:|---:|---:|---:|
| walk 0.0 | 4.73 | 3.59 | 2.52 |
| walk 0.5 | 4.47 | 3.55 | 1.87 |
| walk 1.0 | 2.68 | 3.19 | 3.19 |
| jog 0.0 | 2.72 | 2.20 | 1.66 |

Nine frames help wide and middle turns and jog, but slightly worsen the tight
turn. Distance magnitude also changes with window size, so all edge thresholds
would need reproducible recalibration. The production window remains five
frames until that full edge-quality sweep exists.

## Transition-distance re-analysis (2026-06-14)

A second pass grounded the jolt in the point-cloud transition distance `D` at
the chosen optimal cell, measured with `--diagnose-graph-edge` over every
`demo_walk_jog_topology` edge (proper end->start sub-range
`[0.70,0.95]->[0.05,0.30]`):

| Edge | T_GOOD | D (min, over source samples) | Reading |
|---|---:|---|---|
| walk -> walk | 1.5 | 0.46 (tight) .. 1.23 (wide) | wide anchor weakly periodic |
| walk -> jog | 3.5 | 2.58 .. 3.23 | T raised to admit; gait gap |
| jog -> walk | 3.5 | 2.90 .. 3.24 | same |
| jog -> jog | 2.0 | 1.77 | single jog example, no self-blend |

The jolt magnitude tracks `D`, and `D` tracks which BVH anchor is active: under
one identical pipeline (same runtime, registration, range, blend) only the
anchor clip changes, yet wide-turn `D` is 1.23 while tight-turn `D` is 0.46. `D`
is the Kovar directional metric (§3.1) the edge was scored with; the runtime then
centers its blend on that optimal point (PMG §5.2.1, `kPmgCentered`). The limiter
is clip self-similarity, not code or registration smoothness. **All four edges
are data-bound:** the wide-turn anchor's periodicity floor and the single-example
`jog` node (which also forces T_GOOD up to 3.5 / 2.0 to admit its transitions).

### Phase-range widening is a degenerate trap

`edge_phase_range` (added to the spec format) makes the §6.3 search sub-range
tunable, so the obvious idea -- widen it to find a lower-`D` cell -- is now
testable. It does not help. Widening admits "transition to the same phase"
cells: `D` falls but the character stops advancing.

| walk->walk range | wide-anchor `D` | runtime median_step | transitions/20s | pop_ratio |
|---|---:|---:|---:|---:|
| `0.70-0.95 / 0.05-0.30` (default) | 1.23 | 0.403 | 33 | 1.66 |
| `0.55-0.98 / 0.02-0.45` | 0.19 | 0.227 | 98 | 1.87 |
| `0.50-1.0 / 0.0-0.5` | 0.0 | 0.366 | 21 | 3.06 |

The collapsing `median_step` and exploding transition count confirm stutter in
place. The default range is correct: it forces a genuine advancing end->start
transition, so `D = 1.23` is the true periodicity cost of the wide anchor.

### Anchor swap is a tradeoff, not a free win

Replacing the wide-turn anchor `walkCurve` with `walkSpiral` (the only
Group-B curvature candidate) lowers the proper-range `D` from 1.23 to 0.42, but
`walkSpiral` has shorter steps and a different curvature, so runtime
`median_step` drops 0.403 -> 0.271 and the normalized `pop_ratio` rises
1.66 -> 1.83. Confirming the earlier caution, an anchor cannot be swapped on
transition score alone; it changes the locomotion semantics and needs full
turn-rate re-calibration. Left as a corpus-curation decision, not applied.

### Runtime confounder audit (2026-06-15)

The "data-bound" verdict above was re-tested against three runtime/metric edge
cases flagged in an external continuity review. Each began as an opt-in ablation
toggle (PR #44); the self-edge pre-roll wrap then **landed as the default**
(`TransitionPreRollPolicy::kWrapCyclicSelfEdges`, commit `f3576c3`; the policy was
later gated on per-node cyclic metadata so only cyclic self-edges wrap), so the
improved column below is now the shipped behavior, not an opt-in. The
`demo_walk_jog_topology`
node is cyclic (registration `cycle_joint`), so its self-edge wraps. Measured on the
`demo_walk_jog_topology` self-edge, centered runtime blend, 20 s, reported as
`pop_ratio`
(max step / median step):

| configuration | wide 0.0 | mid 0.5 | tight 1.0 |
|---|---:|---:|---:|
| pre-roll clamp (`--preroll-policy clamp`) | 1.63 | 1.59 | 2.16 |
| **pre-roll wrap (default, self-edges)** | **1.32 (-19%)** | **1.34 (-16%)** | 2.16 |
| cyclic self-edge metric (`--self-edge-cyclic-metric`) | null | null | null |

Two corrections to the data-bound claim:

- **The wide-turn jolt was partly a runtime confounder, not purely corpus.** The
  target pre-roll was clamped at clip start (`std::max(0, target_phase*duration -
  lead)`); for a centered self-edge with a small target phase (walk
  `target_phase ~= 0.07`) this dropped the previous-cycle tail that supplies the
  target's blend velocity. The default now wraps the pre-roll into that tail
  (bidirectional `FoldCompletedCycles`), removing ~19% (wide) / ~16% (mid) of the
  pop deterministically. So `D = 1.23` overstated the wide anchor's *intrinsic*
  periodicity cost; the cyclic clamp, not the corpus, owned that slice.
- **The boundary-clamp metric is inert here** (downgraded, not removed). The
  wrap-aware self-edge metric chose the identical transition (byte-identical
  `pop_ratio`): the calibrated sub-range `[0.70,0.95] -> [0.05,0.30]` keeps the
  optimum off the cycle boundary. The flag's engagement is unit-tested, so the
  null is "same transition chosen", not "flag never ran".

The **tight-turn residual** is untouched by both ablations. It is the D4
metric/runtime window mismatch (the directional metric scores a different frame
support than the centered runtime blends; for the tight anchor centered is
*worse* than directional, 2.16 vs 1.92) plus genuine corpus periodicity -- not a
clamp defect.

Gate-skip (closed): the transition gate was a ~1-frame point-in-window test, so a
variable-frame-time or fast-forward live viewer could skip a self-edge schedule.
The fixed-dt CLI that produced every number here never reproduced it, so it did
not affect these measurements -- but it was a real live-viewer robustness gap.
Commit `f3576c3` replaced the point-in-window test with a wrap-aware
`CrossedPhase(previous_phase, phase_advance, gate)` so a gate crossed within an
update (or a full-cycle jump) still schedules.

### Transition-quality diagnostic Module (2026-06-15)

The earlier continuity experiments computed pose step and acceleration in
one-off command code. `TransitionQuality` now provides one reusable diagnostic
Interface over a world-pose window. The default record measures three frame
intervals before and after a transition and writes:

```text
D, local_max_step, local_pop_ratio
pre/post root speed, root_speed_ratio
pre/post signed yaw rate, yaw_rate_ratio
left/right foot drift, max_contact_drift
left/right contact state before and after
quality_classification
```

Contact state uses the source node's declared contact joints when available;
otherwise drift remains observable and contact state is `unknown`. The
classification is evidence for separating pose, velocity, yaw-rate, contact,
and future cyclic-seam failure modes. The CLI window is at least three
intervals on each side and expands to the runtime blend-frame count. Near-zero
yaw rates use a `0.05 rad/s` deadband. `cyclic_seam_mismatch` is reserved until
the CyclicContinuity contract provides raw seam evidence. This diagnostic does
not change PMG thresholds, scheduling, alignment, or blending.

The first 30-second evidence sweep (2026-06-16, runtime seed 99) is recorded in
`outputs/transition_quality/TRANSITION_QUALITY_SWEEP.md`. The minimal self-edge
demo completed 47 transitions with 47 CSV rows and maximum local pop ratio
`1.615`, below the requested `3.0` gate. The walk/jog topology demo completed
42 transitions, including 24 cross-node transitions; none of those cross-node
transitions wrapped pre-roll.

Every transition in both runs classified as `yaw_rate_discontinuity`. The
self-edge median yaw-rate ratio was `3.630`; the walk/jog median was `9.649`
and maximum was `42.116`. Cross-node edges also had worse D, pose pop, and root
speed ratios than the `0 -> 0` self-edge. This is evidence for future
velocity/yaw/contact-aware rejection, not permission to tune thresholds or
claim a production walk-jog controller. No transition classified as
`contact_mismatch`; foot drift remains observational when the same foot is not
in contact on both sides.

## Yaw-rate diagnostic audit

The yaw-rate metric averages wrapped root-facing deltas independently before
and after the transition, in rad/s. The random-walk CSV uses
`max(3, transition_blend_frames)` intervals on each side: five intervals for
these runs. Its signed discontinuity ratio is:

```text
1 + |pre_yaw_rate - post_yaw_rate|
    / max(min(|pre_yaw_rate|, |post_yaw_rate|), 0.05 rad/s)
```

Both rates inside the `0.05 rad/s` deadband produce ratio `1`. Otherwise, one
small side makes the ratio large. Classification first handles cyclic-seam and
contact cases, then chooses the largest threshold-normalized pose, root-speed,
or yaw score. Ties prefer pose, then yaw, then root speed.

| Evidence | count | median pre yaw (rad/s) | median post yaw (rad/s) | median yaw ratio | max yaw ratio | corr(yaw, D) | corr(yaw, pop) |
|---|---:|---:|---:|---:|---:|---:|---:|
| walk self-edge | 47 | -1.469 | 0.967 | 3.630 | 5.411 | -0.770 | -0.782 |
| walk/jog topology | 42 | 0.259 | 0.798 | 9.649 | 42.116 | 0.133 | 0.148 |

| Walk/jog edge | count | median yaw ratio | max yaw ratio |
|---|---:|---:|---:|
| walk -> walk | 10 | 4.198 | 10.125 |
| walk -> jog | 12 | 5.338 | 28.800 |
| jog -> walk | 12 | 37.693 | 42.116 |
| jog -> jog | 8 | 12.976 | 12.976 |

Yaw failure is global, not localized to cross-gait transitions. Every minimal
walk self-edge transition also fails the current `1.3` threshold despite a
maximum local-pop ratio of only `1.615`. Within that self-edge run, yaw ratio is
negatively correlated with both `D` and local pop; in the walk/jog run its
correlation with either is weak. Yaw ratio therefore does not track the existing
pose-quality signals consistently.

**Conclusion:** signed yaw-rate change is real in the sampled runtime motion,
especially where direction reverses, but the current ratio and `1.3` threshold
are too sensitive for hard rejection. A permissive `2.5` ablation still rejects
every candidate and stalls the random walk. Keep yaw gating diagnostic-only
until the metric, support, and threshold have independent validation.

## Transition-quality gate ablation

`pmg_cli --quality-gate` is opt-in experimental diagnostic tooling. It is
disabled by default, so default random-walk scheduling and motion output remain
unchanged. Gate-disabled transition and motion CSV SHA-256 hashes matched the
previous baseline exactly.

The 30-second walk/jog ablation used runtime seed `99`. The baseline completed
42 transitions, including 24 cross-node transitions.

| Gate | Completed transitions | Cross-node transitions | Result |
|---|---:|---:|---|
| disabled | 42 | 24 | baseline unchanged |
| root-speed ratio `1.4` | 22 | 0 | removed all cross-node motion |
| yaw-rate ratio `2.5` | 0 | 0 | rejected every encountered candidate |
| contact drift `5.0` | 1 | 0 | nearly stalled; only one self-edge completed |
| combined | 0 | 0 | stalled |

All active gate configurations removed or effectively stalled cross-node
walk/jog transitions. The current yaw-rate ratio and thresholds are not
suitable for hard rejection: yaw failure is global, including low-pop
self-edges, rather than a reliable discriminator for bad cross-node
transitions. These results do not justify build-time candidate rejection,
threshold changes, or production runtime control.

**Status: PASS WITH LIMITATION.** The diagnostic is reproducible and default
behavior is unchanged, but `demo_walk_jog_topology` remains a corpus-limitation
case and not a production walk/jog controller. Keep the gate diagnostic-only.
Before production control, collect cyclic-continuity evidence and improve clip
cut points and corpus coverage.

### Conclusion

The residual walk-loop jolt is **mostly** a corpus periodicity limit (wide-turn
anchor and single-example jog), but not purely: ~19% (wide) / ~16% (mid) of the
self-edge pop was a deterministic pre-roll-clamp confounder, now removed -- the
self-edge pre-roll wrap is the shipped default for cyclic nodes
(`kWrapCyclicSelfEdges`, `f3576c3`). The
boundary-clamp metric is inert on this corpus, and the tight-turn residual is the
D4 window mismatch plus corpus, not a defect. The `edge_phase_range` plumbing is
retained as a real conformance/tunability feature (§6.3) but is documented here
as ineffective against this particular jolt.

## Paper Comparison

The implementation choice is based on primary sources:

- [Heck thesis, Chapter 6: Parametric Motion Graphs](https://pages.cs.wisc.edu/~heckr/Thesis/6-PMGs.pdf)
  defines a locomotion node as short, two-step clips and repeatedly selects an
  outgoing edge, target parameter, alignment, and centered linear-blend
  transition. It also states that PMG requires smooth parametric motion spaces.
- [Kovar and Gleicher 2004](https://graphics.cs.wisc.edu/Papers/2004/KG04/)
  builds those smooth spaces through automatic related-motion extraction,
  constrained time alignment, registration, blending, and sampled inverse
  parameterization. Its walk experiment reduced 96 examples to 46 after
  redundancy removal.
- This repository uses three manually selected walk examples, contact anchors,
  slope-constrained DTW, a cubic smoothing-spline registration curve (the KG04
  spline step, in penalized-second-difference form), and root-delta
  reconstruction. It does not reproduce the KG04 match web, match graph/reference
  selection, the per-frame rigid pose alignment + constraint matching of the full
  registration curve, or the original walk corpus. Smoothness is therefore
  partially reproduced (the timing curve) and partially still an input
  assumption (the per-frame pose alignment).

PMG does not continuously change the parameter inside an already generated
clip. It chooses the next parameter at a graph transition. Short clips provide
responsiveness.

## `maxxgx/motion-graphs` Comparison

Reference: [maxxgx/motion-graphs](https://github.com/maxxgx/motion-graphs),
inspected at commit `45372d896ea84ca1b7f14b69f76d74f60106b8e3`.

Useful agreement:

- root translation uses linear interpolation;
- joint rotations use quaternion slerp;
- transition weight is cubic with zero endpoint slopes;
- both source and target windows participate in the blend.

Material differences from the original Motion Graphs and PMG papers:

- it implements discrete Motion Graphs, not parametric motion spaces or PMG
  reachable target boxes;
- its graph construction excludes transitions within the same source
  animation, so it cannot model this PMG self-edge case;
- its point-cloud distance sums the square of an already squared point
  distance, producing fourth-power distance;
- it does not perform the paper's optimal floor-plane alignment in that
  distance function;
- local-minimum and graph-validity handling is not equivalent to the original
  SCC/path machinery.

Only its cubic pose blend is corroborating implementation evidence. Copying its
graph or distance code would reduce paper fidelity.

## Why Jog Cannot Be Adjusted

`demo_walk_jog_topology.pmg_spec` declares:

```text
node jog 1
example jog 0.0 ../BVH/jogCurve.bvh
```

One example produces parameter domain `[0.0, 0.0]`. The viewer slider has equal
minimum and maximum, generated requests clamp to 0, and the runtime has no
second jog example to blend toward. Declaring dimension 1 does not create a
controllable degree of freedom. The viewer now reports this as a fixed
parameter instead of displaying an inert slider.

A controllable jog node needs at least two compatible, semantically ordered jog
examples, preferably several curvature samples, followed by registration,
metric calibration, and edge rebuild. No such jog family exists in the current
Group-B corpus.

## Desired Parameter Contract

`desired_parameter` is the requested coordinate in the target node's
parametric motion space:

1. viewer chooses `desired_node` and target-space coordinate;
2. outgoing edge lookup computes the reachable target box;
3. request is clamped to that box;
4. target node generates the next short clip at the clamped coordinate;
5. runtime aligns and blends into it at the stored transition phases.

For `demo_walk_jog_topology`:

- `walk`: authored scalar 0..1, calibrated against measured `turn_rate`;
- `jog`: singleton scalar fixed at 0.

For `demo_walk_2d_triangle.pmg_spec`:

- axis 0: turn/curvature;
- axis 1: gait/speed, calibrated with `travel_speed`;
- goal-directed goto now drives both axes (turn_rate heading + travel_speed
  pace); the manual blend slider still exposes axis 0 and holds axis 1 at its
  midpoint.

`legacy_walk_curvature.pmg_spec` remains available for one-slider steering
regression, but its non-monotone signed response excludes it from the curated
demo set. `demo_walk_2d_triangle.pmg_spec` is the 2-D data contract and is
steerable under goto; only the manual per-axis slider UI is still 1-D.
