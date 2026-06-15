# PMG Spec Audit

Last updated: 2026-06-15.

## Purpose

Audit whether `specs/*.pmg_spec` describe semantically meaningful Parametric
Motion Graph nodes and edges, and classify each file as curated demo,
validation fixture, or retained legacy regression asset.

Claim boundary:

- Motion Graphs 2002 supplies the directional point-cloud transition metric.
- KG04 supplies the smooth, registered, parameterized motion-space assumption.
- PMG nodes are motion spaces; PMG edges are sampled valid transition regions.
- This repository may manually author spaces and omit database extraction,
  match webs, SCC pruning, and global path search.
- Manual authorship does not remove the need for coherent node parameters,
  compatible motions, meaningful edge regions, and advancing runtime streams.

## Executive Summary

Current specs are not one uniform set of canonical PMG graphs. They mix
legitimate minimal PMG fixtures, edge-construction stress cases, and one
cross-gait limitation case.

| Classification | Specs |
|---|---|
| Canonical minimal PMG demo | `demo_walk_self_edge_minimal.pmg_spec` |
| Canonical sparse 2-D AABB interpolation demo, not hull enforcement | `demo_walk_2d_triangle.pmg_spec` |
| Canonical topology demo and cross-gait limitation case | `demo_walk_jog_topology.pmg_spec` |
| Legitimate edge-classification fixture | `fixture_edge_selective_good_bad.pmg_spec` |
| Builder stress fixture, not a meaningful motion graph | `fixture_transition_box_shrink.pmg_spec` |
| Valid sparse PMG node retained for regression, but misleading as a demo | `legacy_walk_curvature.pmg_spec` |

Main conclusions:

1. Parser, builder, artifact, and runtime semantics are internally consistent.
   Specs select compatible examples, deterministic sampling, thresholds, and
   runtime-consumed edge regions.
2. Parser validity is weaker than PMG semantic validity. It does not test
   logical motion similarity, sample-rank sufficiency, parameter monotonicity,
   independent axes, or perceptual transition quality.
3. `demo_walk_self_edge_minimal` is the canonical 1-D self-edge demo. Its two anchors
   produce a monotone achieved turn trend in runtime diagnostics.
4. `legacy_walk_curvature` is a valid sparse walk motion space, but its
   "wide to tight" scalar is not monotone in measured signed turn rate. Anchors
   measured approximately `-0.148`, `+0.522`, and `-2.236 rad/s`.
5. `demo_walk_jog_topology` is not a full walk/jog PMG controller. Walk is
   parameterized; jog
   is a single-example non-controllable node. Cross-gait edges require much
   higher corpus-specific thresholds than same-gait edges.
6. `demo_walk_2d_triangle` has enough non-collinear samples to exercise sparse
   2-D AABB interpolation and control. It does not enforce the authored
   triangle as a reachable hull or establish a robust rectangular turn-speed
   family: the tight-jog corner is missing, and the jog anchor changes both
   turn rate and speed.
7. The viewer now exposes persistent root travel, edge-colored transition
   events, and exact metric/runtime frame supports. This makes advancing versus
   stuttering behavior inspectable, but does not prove perceptual smoothness.

## Audit Evidence

Evidence came from current source, specs, generated V8 artifacts, and these CLI
paths:

```powershell
pmg_cli --diagnose-graph-edge <spec> <source> <target>
pmg_cli --space-sweep <spec> <node> --sweep-steps 9
pmg_cli --random-walk <spec> --seconds 20 --walk-seed 7
pmg_cli --goto <spec> 10 10 --seconds 30 --tolerance 3
pmg_cli --inspect-graph <artifact>
```

### 2026-06-15 demo runtime validation

A 30-second, seed-99 runtime evidence pass is recorded in
`outputs/pmg_demo/PMG_SPEC_VALIDATION_REPORT.md`. The three demo artifacts all
built and streamed:

| Spec | Transitions | Cross-node | Mean D | Max local pop ratio | Clamp evidence |
|---|---:|---:|---:|---:|---|
| `demo_walk_self_edge_minimal` | 47 | 0 | 152.807 | 1.615 | 5 outside-box requests, all clamped |
| `demo_walk_2d_triangle` | 14 | 0 | 49.405 | 1.503 | no clamp; 5 requests outside the authored triangle accepted unchanged |
| `demo_walk_jog_topology` | 42 | 24 | 246.002 | 2.795 | topology exercised; singleton jog domain remained fixed |

This strengthens the minimal self-edge and topology claims. It also narrows the
2-D claim further: comments correctly describe triangular authored samples, but
the current transition boxes are rectangular AABBs. Five requests with
`axis0 + axis1 > 1` were accepted unchanged because every observed box was
`[0,0]..[1,1]`. The spec therefore demonstrates sparse 2-D AABB
interpolation/calibration, not projection to triangular support.

`--validate-graph-spec` passed for all three files. The former
`--validate-graph` fallback to global defaults was fixed in core commit
`333853f`: validation now starts from each edge's authored `edge_config`, prints
the effective thresholds/sample counts/window/convention, and applies only
explicit CLI overrides. Revalidation passed for all demos and matched artifact
edge sample counts: self-edge `14`, 2-D `6`, topology `11/11/1/1`.

Distances are raw weighted squared sums in native BVH units. They are only
comparable under this skeleton, window, weighting, registration, and sampling
configuration.

No spec declares `edge_phase_range`. Every edge uses:

```text
source phase [0.70, 0.95]
target phase [0.05, 0.30]
window size 5
metric convention kovar_directional
runtime blend placement pmg_centered
```

Stored references remain near late-source/early-target regions for all built
edges. This supports advancing intent. It does not prove smooth advancement.

## Spec Taxonomy

| Spec file | Intended role | Nodes and samples | Edge types | Validity classification | Visualization expectation | Main risk |
|---|---|---|---|---|---|---|
| `demo_walk_self_edge_minimal.pmg_spec` | Minimal PMG core demo | `walk`, 1-D, 2 walks | self-edge | Valid PMG node; canonical minimal demo | Advancing curved path, repeated markers, reduced jolt vs raw wrap | Sparse interpolation; not robust path following |
| `demo_walk_2d_triangle.pmg_spec` | 2-D machinery demo | `walk_2d`, 2-D, 3 walk/walk/jog anchors | self-edge | Minimal calibrated 2-D demo; not robust full family | Held-axis turn/speed trends; triangle visible | Missing tight-jog corner, coupled axes, no registration |
| `demo_walk_jog_topology.pmg_spec` | Multi-node topology demo | parameterized walk; singleton jog | self and cross edges | Walk valid; jog fixed; graph exposes topology and corpus limit | Gait changes marked; larger cross-gait seam expected | Threshold-forced connectivity may look stronger than it is |
| `fixture_edge_selective_good_bad.pmg_spec` | GOOD/NEUTRAL/BAD fixture | `walk`, 1-D, same 3 walks | self-edge | Legitimate edge-validation fixture | Reachable interval narrows for weak source regions | Not a controller-quality claim; no calibration |
| `fixture_transition_box_shrink.pmg_spec` | Conservative AABB shrink fixture | singleton source; walk/jog/walk target | cross-edge | Stress-test nodes; target is not a meaningful family | Pre/post box and BAD exclusion | Playback cannot validate node semantics |
| `legacy_walk_curvature.pmg_spec` | Three-anchor regression/audit asset | `walk`, 1-D, 3 walks | self-edge | Valid sparse node; parameter description misleading | Continuous stream; turn response plot exposes sign reversal | Scalar is not monotone turn tightness; permissive edge |

## Spec Configuration Catalog

All edges below use default phase ranges
`[0.70,0.95] -> [0.05,0.30]`, five-frame directional metric windows, and
centered runtime blend placement.

| Spec | Node configuration | Edge configuration |
|---|---|---|
| `demo_walk_self_edge_minimal` | `walk`: 1-D; `(0) walkCurve`, `(1) walkTightCurve`; cycle `LeftAnkle`; contacts both ankles; min contact `3`; DTW on; no parameter metric | `walk->walk`: `TGOOD/TBAD=225/250`, random source/target samples `12/60`, seed `7` |
| `demo_walk_2d_triangle` | `walk_2d`: 2-D; `(0,0) walkCurve`, `(1,0) walkTightCurve`, `(0,1) jogCurve`; no cycle/contact registration or DTW; metrics `turn_rate, travel_speed`; calibration density `5` | `walk_2d->walk_2d`: `300/400`, `3/6`, seed `41` |
| `demo_walk_jog_topology` | `walk`: same three-example calibrated walk; `jog`: 1-D singleton `(0) jogCurve`; both cycle-normalized/contact-registered; walk DTW on, jog DTW off | `walk->walk`: `225/250`, `8/40`, seed `17`; `walk->jog`: `450/500`, `8/20`, seed `19`; `jog->walk`: `300/350`, `4/40`, seed `23`; `jog->jog`: `80/100`, `4/20`, seed `29` |
| `fixture_edge_selective_good_bad` | Same three-example 1-D walk and registration; no parameter metric | `walk->walk`: `203/210`, `12/60`, seed `7` |
| `fixture_transition_box_shrink` | `source_walk`: 1-D singleton `(0) walkMoreCurve`; `nonconvex_target`: `(0) walkCurve`, `(0.5) jogCurve`, `(1) walkTightCurve`; no cycle/contact registration, DTW, metrics, or calibration | `source_walk->nonconvex_target`: `10/100`, `4/100`, seed `31` |
| `legacy_walk_curvature` | `walk`: 1-D; `(0) walkCurve`, `(0.5) walkMoreCurve`, `(1) walkTightCurve`; same registration; `turn_rate`; default calibration density `9` | `walk->walk`: `225/250`, `12/60`, seed `7` |

Builder sample sets also include authored example parameters. Actual source and
target sample totals can therefore exceed the random counts above, while
duplicate samples collapse in zero-volume domains.

## Per-Spec Findings

### `demo_walk_self_edge_minimal.pmg_spec`

#### What it claims

- One 1-D `walk` motion space.
- `walkCurve` at `0` and `walkTightCurve` at `1`.
- Cycle extraction, ankle registration, and DTW refinement.
- One self-edge with `TGOOD=225`, `TBAD=250`, `12/60` samples, seed `7`.

#### Node validity

This is a valid PMG node and legitimate minimal fixture:

- examples are skeleton-compatible Group-B locomotion clips;
- examples are logically similar cyclic walks;
- two distinct examples are minimum useful support for 1-D interpolation;
- normalized endpoints have interpretable wide/tight meaning;
- runtime-held measurements changed about `+0.088 -> -0.871 -> -1.756 rad/s`
  at parameters `0`, `0.5`, and `1`.

It is not a robust motion family. Two examples cannot demonstrate dense
coverage, broad local smoothness, or arbitrary curvature control. No
`parameter_metric` is declared, so the scalar remains authored normalized
control rather than a calibrated physical turn-rate request.

#### Edge validity

The self-edge is conceptually correct for repeated locomotion and uses the
advancing default phase policy. At the widest-turn source, targets above about
`0.69` become NEUTRAL; no targets become BAD because `TBAD=250` exceeds the
observed maximum. Other sampled sources admit the full target interval.

This is meaningful but permissive. It demonstrates self-edge streaming, not
strong transition-region discrimination.

The 20-second random stream completed `32` transitions with
`pop_ratio=1.86`. Useful regression evidence, not perceptual proof.

#### Correct visualization

Should show:

- continuous advancing world path;
- transition markers near every self-edge traversal;
- no root teleport;
- lower seam jolt than raw cycle wrap;
- turn trend changing from parameter `0` to `1`;
- alternating contacts with bounded visible drag.

Should not be expected:

- arbitrary path following;
- symmetric left/right steering;
- dense-family smoothness;
- zero foot slide or zero seam acceleration.

Failure interpretation:

- in-place stutter with low distance: phase/search or runtime issue;
- advancing path with residual jolt: likely corpus periodicity;
- root/facing discontinuity: runtime alignment or placement issue;
- non-monotone endpoint trend: parameter/spec issue.

### `legacy_walk_curvature.pmg_spec`

#### What it claims

- One 1-D walk space with wide, middle, and tight authored samples.
- Scalar described as turn tightness.
- `turn_rate` calibration.
- Repeated self-edge runtime.

#### Node validity

Examples are compatible and logically similar, so this is a valid sparse PMG
node. Three samples are reasonable support for a minimal 1-D family.

The parameter claim is not valid as written:

| Parameter | Example | Measured turn rate |
|---:|---|---:|
| `0.0` | `walkCurve` | `-0.148 rad/s` |
| `0.5` | `walkMoreCurve` | `+0.522 rad/s` |
| `1.0` | `walkTightCurve` | `-2.236 rad/s` |

The middle anchor reverses signed turn direction. Runtime-held values similarly
measured about `+0.097`, `+0.704`, and `-1.739 rad/s`. The axis is a piecewise
authored path through turn behavior, not monotone `wide -> tight` control.

Calibration makes each segment measurable; it does not remove global ambiguity
or make the authored scalar a unique physical turn rate. Goal-directed control
can work over the measured range, but inverse-choice behavior is implementation
policy rather than a clean semantic axis.

#### Edge validity

The self-edge is meaningful repeated locomotion, but current thresholds are
completely permissive:

```text
GOOD=945, NEUTRAL=0, BAD=0
all sampled source boxes=[0,1]
```

This supports runtime connectivity. It does not validate selective target
regions. Stored references stay around source `0.690..0.714` and target
`0.035..0.067`, consistent with an advancing wrap.

#### Correct visualization

Should show:

- continuous repeated walking;
- response plot exposing sign reversal near the middle anchor;
- transition markers and path progression;
- raw-wrap vs self-edge comparison.

Should not be expected:

- monotone turn tightness;
- one-to-one authored parameter to physical turn rate;
- proof that every parameter pair is equally good because all are below
  `TGOOD`.

Failure interpretation:

- sign reversal: spec/corpus parameterization issue;
- poor wide-turn loop seam: corpus periodicity;
- full-domain boxes: threshold behavior, not universal compatibility.

### `fixture_edge_selective_good_bad.pmg_spec`

#### What it claims

- Same three-example 1-D walk family.
- No parameter metric calibration.
- Lower thresholds (`203/210`) to expose GOOD/NEUTRAL/BAD classification and
  parameter-dependent reachable intervals.

#### Node validity

The node is a valid sparse PMG node for edge testing. Its parameter has the same
non-monotone physical-turn caveat as `legacy_walk_curvature`; lack of
calibration makes
it unsuitable as a strong physical-control demo.

#### Edge validity

This is a meaningful Section 3.2 validation fixture:

```text
GOOD=854, NEUTRAL=62, BAD=29
```

Wide/weak sources produce restricted intervals such as `[0.669, 0.869]`;
tighter sources often retain `[0,1]`. No AABB shrink is needed because sampled
BAD points already fall outside the GOOD enclosure.

It validates classification and source-dependent reachability. It does not
validate `ShrinkToExclude`; that role belongs to
`fixture_transition_box_shrink`.

#### Correct visualization

Should show:

- GOOD/NEUTRAL/BAD coverage, not only minimum distance;
- reachable target interval changing with source parameter;
- phase spread inside each interval;
- BAD count and confirmation that no BAD sample remains inside a box.

Should not be expected:

- better runtime motion than `legacy_walk_curvature`;
- calibrated turn-rate control;
- AABB shrink events in this build.

### `demo_walk_2d_triangle.pmg_spec`

#### What it claims

- One 2-D node.
- Axis 0: authored turn class.
- Axis 1: authored walk/jog or speed class.
- Three non-collinear examples at `(0,0)`, `(1,0)`, `(0,1)`.
- Calibration against `turn_rate` and `travel_speed`.
- One self-edge.

#### Node validity

This is a legitimate sparse 2-D AABB interpolation fixture. Three
non-collinear examples are minimum support for a 2-D local stencil and vector
calibration.

This demonstrates sparse 2-D AABB interpolation machinery, not convex-hull
support enforcement. Requests outside the authored triangle are accepted
because current reachable domains are AABBs.

It is not a robust rectangular 2-D family.

| Parameter | Example | Turn rate | Travel speed |
|---|---|---:|---:|
| `(0,0)` | `walkCurve` | `+0.339 rad/s` | `9.975 units/s` |
| `(1,0)` | `walkTightCurve` | `-2.099 rad/s` | `9.921 units/s` |
| `(0,1)` | `jogCurve` | `+1.287 rad/s` | `23.228 units/s` |

Consequences:

- speed/gait anchor also changes turn rate;
- no `(1,1)` tight-jog example exists;
- implementation domain is the examples' AABB, not their triangular hull;
- missing-corner requests synthesize a three-way blend;
- calibration inverts behavior generated by this sparse blend. It cannot add
  missing motion support.

Held-axis runtime calibration is non-degenerate:

```text
axis 0 turn_rate: +0.397 -> -0.485 rad/s
axis 1 travel_speed: 12.385 -> 16.466 units/s
```

That supports calling it a sparse 2-D AABB interpolation/controller fixture. It
does not support uniform quality or independent control over the full rectangle.

Registration is disabled and clips are not cycle-normalized. Acceptable for a
calibration/runtime fixture, but weaker than the registered walk nodes.

#### Edge validity

The self-edge builds, but thresholds are fully permissive:

```text
GOOD=54, NEUTRAL=0, BAD=0
all source boxes=[0,0]..[1,1]
```

Stored phases span source `0.697..0.940` and target `0.073..0.293`. The edge is
valid for 2-D lookup and runtime. It does not establish that the whole rectangle
is a high-quality transition domain.

#### Correct visualization

Should show:

- authored triangle in 2-D parameter space;
- requested point and local blend weights;
- held-axis turn and speed trends;
- warning or shading for unsupported `(1,1)` corner;
- path speed changing under axis 1;
- turn trend changing under axis 0;
- transition markers over the slower long-clip cadence.

Should not be expected:

- full rectangular coverage with uniform quality;
- true tight-jog endpoint;
- physical axis independence;
- perfect walk/jog blending without registration.

Failure interpretation:

- coupled response: corpus/spec sampling limitation;
- poor missing-corner blend: spec design/corpus density limitation;
- non-advancing transitions: edge/runtime issue;
- calibration matching sparse generated behavior but not user intent:
  parameterization issue.

### `demo_walk_jog_topology.pmg_spec`

#### What it claims

- Parameterized walk node.
- Jog node declared 1-D but containing only `jogCurve` at `0`.
- Walk and jog self-edges plus both directed cross-gait edges.

#### Node validity

`walk` is the same valid sparse node as `legacy_walk_curvature`, with the same
non-monotone turn-axis caveat.

`jog` is a single-example non-controllable node:

- declared dimension: 1;
- sample count: 1;
- domain: `[0,0]`;
- every request clamps to `0`;
- no jog interpolation or jog steering exists.

It is valid as a fixed graph node. It is a degenerate parameterized node and not
a meaningful jog motion family.

#### Edge validity

Conceptually, all four directed edges are reasonable. Empirically:

| Edge | `TGOOD/TBAD` | Current evidence | Audit reading |
|---|---:|---|---|
| walk -> walk | `225/250` | all sampled targets GOOD | permissive valid self-edge |
| walk -> jog | `450/500` | one fixed target; `D=304.6..428.1` | threshold-forced cross-gait stress |
| jog -> walk | `300/350` | all walk targets GOOD; `D=269.9..296.3` | plausible but high-cost cross-gait edge |
| jog -> jog | `80/100` | one fixed pair; `D=74.3` | fixed-node loop edge |

Cross edges are meaningful corpus-limitation measurements because they retain
late-source/early-target searches. They are not evidence of a smooth
bidirectional walk/jog family.

The 20-second random walk completed `29` transitions with `pop_ratio=3.07`,
substantially worse than walk-only fixtures. This is consistent with gait-gap
stress, not proof of runtime failure.

#### Correct visualization

Should show:

- node identity and gait-change event at each cross-edge;
- persistent path with markers colored by edge type;
- walk/jog speed and cadence change;
- larger cross-gait seam diagnostics than same-gait transitions;
- jog desired parameter fixed at `0`;
- raw vs blended cross-gait overlay.

Should not be expected:

- jog curvature control;
- smooth interpolation within jog;
- seamless gait changes from one jog example;
- same quality as walk self-transitions.

Failure interpretation:

- inert jog slider: correct consequence of degenerate node;
- visible jolt with continuous root path: corpus/threshold limit;
- root teleport or facing snap: runtime/alignment defect;
- acceptance only after high thresholds: threshold/corpus issue, not proof of
  semantic compatibility.

### `fixture_transition_box_shrink.pmg_spec`

#### What it claims

- Singleton source walk.
- Target sequence `walk, jog, walk` on one scalar.
- GOOD endpoints may enclose BAD interior targets.
- Builder must shrink target AABB to exclude BAD samples.

#### Node validity

Neither node is a normal parametric family:

- `source_walk` is a single-example non-controllable node;
- `nonconvex_target` orders dissimilar walk/jog/walk clips on one scalar;
- target interpolation crosses a semantic gait discontinuity by construction;
- registration and calibration are disabled.

These are valid stress-test nodes, not meaningful PMG locomotion nodes.

#### Edge validity

The edge is valid for its narrow algorithmic purpose:

```text
GOOD=15, NEUTRAL=53, BAD=35
box before=[0.000, 0.992]
box after =[0.000, 0.345]
```

This directly exercises BAD exclusion. It should not support runtime,
controller, or motion-family claims.

#### Correct visualization

Should show:

- target samples colored by class;
- pre-shrink and post-shrink AABBs;
- BAD samples excluded from final box;
- source parameter shown as fixed;
- playback labeled diagnostic-only.

Should not be expected:

- smooth motion over the target scalar;
- meaningful interpolation through jog;
- reusable locomotion control.

## Cross-Cutting Issues

### Semantic validation is absent

`LoadGraphSpec` and `PrepareMotionSpaces` validate syntax, dimensions, paths,
skeleton compatibility, registration inputs, and metric declarations. They do
not validate:

- logical similarity;
- effective sample rank;
- singleton controllability;
- axis monotonicity;
- axis independence;
- unsupported AABB corners;
- threshold separation quality;
- advancing world-space behavior.

This is acceptable for a low-level format, but inspection should report obvious
semantic warnings.

### Degenerate single-example nodes

`demo_walk_jog_topology:jog` and
`fixture_transition_box_shrink:source_walk` are zero-volume
parameter spaces despite positive declared dimension. They are fixed nodes, not
controllable parametric families.

### Sparse 2-D domain

`demo_walk_2d_triangle` uses a triangular sample set but exposes its rectangular
AABB. Positive local weights make every request executable; executable is not
equivalent to supported by nearby examples.

### Calibration is not controllability

Calibration can measure generated blends, normalize metric units, and invert a
sampled generated map. It cannot create missing examples, make coupled anchors
independent, repair wrong parameter ordering, or establish perceptual quality.

### Threshold over-admission

Several runtime-demo edges classify every sampled target as GOOD. Useful for
connectivity, but weak evidence for selective transition-region construction.
Specs should identify thresholds as:

- runtime-permissive;
- classification-selective;
- AABB-shrink stress;
- cross-gait forced-connectivity.

### Phase-range misuse risk

All specs use the advancing default. Existing evidence shows wider ranges can
find same-phase, low-distance cells that stutter. Low distance is insufficient;
path advancement must be measured and visualized.

### Transition convention interaction

Artifacts store `kKovarDirectional` as metric convention. Runtime defaults to
`kPmgCentered` blend placement around stored references. CLI can compare
conventions on raw clip pairs and override blend placement.

Remaining ambiguity:

- raw-vs-centered comparison is not available in one visual view;
- some human-readable artifact metadata still says V7 despite V8 semantics.

The viewer now displays the exact directional metric and centered runtime
supports during an active transition. Policy is inspectable, but no integrated
raw-vs-centered visual comparison exists.

### Raw distance scale dependence

Thresholds are not semantic units. They depend on point count, window,
weighting, BVH scale, registration, generated timing, and phase range.

## Diagnostic Support

| Diagnostic | Current support | Audit status |
|---|---|---|
| Graph nodes/edges and sample counts | Viewer and `--inspect-graph` | Sufficient |
| GOOD/NEUTRAL/BAD counts | Edge CLI and build CSV/report | Sufficient numerically |
| Reachable target boxes | CLI; viewer for 1-D runtime | Partial for N-D |
| Phase references | Artifact, CLI, viewer | Partial |
| Exact metric support frames | CLI and active-transition viewer row | Available |
| Exact runtime support frames | Active-transition viewer row | Available |
| Directional vs centered comparison | CLI clip-pair command | No integrated overlay |
| Path trail | Viewer PMG Runtime, bounded persistent trail | Available |
| World transition markers | Viewer PMG Runtime, colored by source/target edge | Available |
| Raw wrap vs blended overlay | Numerical experiments only | Missing, P1 |
| Before/after aligned overlay | Synthetic CLI metrics only | Missing |
| Foot contact timeline | Viewer motion-space panel | Available |
| Foot stamps / world slide trail | None | Missing |
| Root velocity discontinuity | Pairwise synthetic CLI metric | Partial |
| Yaw/facing discontinuity | Synthetic CLI and live scalar | Partial |
| Phase coverage distribution | Recoverable from artifact | Not summarized |
| BAD exclusion count | Build data and shrink count | Partial |
| Phase variance inside boxes | Stored samples only | Missing summary |
| 2-D support / missing-corner warning | Anchors shown, no support quality | Missing |

## Recommendations

### P0 - Documentation and classification

1. Expose only the three `demo_*` specs as the curated showcase set.
2. Keep `fixture_*` specs for edge classification and BAD-exclusion validation.
3. Keep `legacy_walk_curvature.pmg_spec` for regression/audit commands, not as
   a presentation demo.
4. Preserve explicit "Demonstrates" and "Does not demonstrate" boundaries in
   every curated demo.

### P1 - Small diagnostics

Implemented on `dev/ui` in `ed8a488`:

1. Persistent root path trail.
2. World transition markers, colored by edge.
3. Exact metric and runtime frame supports from existing
   `TransitionFrameWindows`.

Remaining:

1. Add CLI edge summary:
   - class fractions;
   - full/restricted/shrunken box counts;
   - phase min/max and variance;
   - BAD exclusion count;
   - zero-volume target-domain warning.
2. Add raw-wrap vs PMG-blended toggle/overlay for active self-edge.

### P2 - Optional spec redesign

1. Curate a monotone 1-D turn family. `demo_walk_self_edge_minimal` currently
   has the cleanest
   measured trend; add a third anchor only after verifying measured ordering.
2. Add missing tight-jog example before presenting a rectangular 2-D family.
3. If no clip exists, keep unsupported AABB regions explicit. Hull-aware
   projection is tracked in
   [GitHub issue #48](https://github.com/n01r1r/ParametricMotionGraphs/issues/48).
4. Add multiple jog curvature examples before claiming jog controllability.

### P3 - Optional code changes

1. Emit semantic warnings from `--validate-graph-spec`:
   - examples fewer than `dimension + 1`;
   - zero-volume axis;
   - rank-deficient multidimensional samples;
   - AABB corners unsupported by authored samples;
   - non-monotone measured metric response.
2. Keep stress fixtures parseable; warnings or role metadata are preferable to
   unconditional rejection.
3. Include convention, phase ranges, and support-frame policy in human-readable
   artifact reports.

## Final Assessment

Specs are more than convenient files that happen to move a skeleton: builder
and runtime semantics implement PMG nodes, sampled edges, reachable regions,
phase lookup, alignment, and repeated streaming.

Only some specs are meaningful motion-graph demonstrations:

- `demo_walk_self_edge_minimal` is the canonical minimal PMG core demo.
- `legacy_walk_curvature` is a valid node with a misleading scalar-control
  story and remains regression-only.
- `demo_walk_2d_triangle` is valid as minimal 2-D machinery, not robust 2-D
  locomotion family.
- `demo_walk_jog_topology` is valid as a topology and limitation demo, not a
  parameterized walk/jog controller.
- `fixture_*` specs are algorithm validation assets, not showcase graphs.

Immediate problem is taxonomy and diagnostic visibility, not need for large
parser/runtime rewrites.
