# PMG Next Roadmap — Phase-by-Phase Prompts

Current branch:
`reachable-region-audit`

Current status:

* `reachable-region-audit` completes A1-A4 and B1-B3 as report-only
  validation/diagnostic phases for a single-node `walk_2d` self-edge PMG. B1.5,
  C1, C2, and D1 are phase-closed. D1 found insufficient cyclic run data, so
  conditional phases D2-D4 are not authorized by their corpus gate.
* Architecture / ADR / README / CONTEXT checkpoint is complete.
* CTest is passing.
* ViewerRuntimeModule / paper-first architecture is considered the stable baseline.
* Do not start by adding more graph complexity. First validate whether the current single-node PMG is actually paper-faithful and animation-quality acceptable.

Global constraints:

* Do not change PMG artifact format unless explicitly required.
* Do not change runtime transition semantics unless a phase explicitly requests it.
* Do not add foot locking before report-only contact/foot-slide diagnostics.
* Do not add a second node before a compatible BVH corpus audit.
* Keep each phase as an independent diff with independent tests and report artifacts.
* Prefer audit/report-only phases before behavior-changing phases.

---

## Phase A — Paper-faithful validation

Goal:
Validate that the current single-node triangulated locomotion PMG actually behaves like a parameterized motion space with valid sampled transitions.

### Phase A1 — Parameter Response Audit — COMPLETE (2026-06-19)

Completed artifacts: `build/parameter_response_audit.csv` and
`build/parameter_response_audit.md`. Result: `PASS_PARAMETER_RESPONSE` (62
samples; anchors, triangle centroids, unique edge midpoints, support grid, and
projected outside request; all generated motions finite/valid). Targeted
parameter/runtime/viewer-runtime tests: 4/4 passed.

Implement a parameter-response audit for the current `walk_2d` triangulated motion space.

Purpose:
We need to verify that authored parameters correspond to actual generated locomotion behavior. A PMG is not merely a blend UI; the requested parameter should predictably affect the generated motion.

Questions to answer:

* Does `(-0.3, 0.0)` produce left-curving walk?
* Does `(0.0, 0.0)` produce base walk?
* Does `(1.0, 0.0)` produce right/tight curve?
* Does `(0.0, 1.0)` produce jog / fast locomotion?
* Does `(0.15, 0.75)` behave like a meaningful interior fast-straight blend?
* Does interpolation vary smoothly across the triangulated support?

Add CLI command if needed:

```powershell
.\build\Debug\pmg_cli.exe --audit-parameter-response `
  .\build\walk_2d_triangulated_t120_b234.pmg `
  --output-csv .\build\parameter_response_audit.csv `
  --output-md .\build\parameter_response_audit.md
```

Use actual CLI conventions if different.

For each sampled parameter point, report:

* requested parameter `(p0,p1)`
* projected support parameter
* generated frame count
* root displacement `(dx,dz)`
* total path length
* average speed
* signed heading change
* approximate signed curvature
* endpoint heading
* whether generated motion is finite / valid
* nearest anchor label
* triangle / barycentric weights if available

Sample at least:

* all authored anchors
* triangle centroids
* edge midpoints
* a small grid over the support
* outside requests projected onto support, e.g. `(0.8,0.8)`

Output:

```text
build/parameter_response_audit.csv
build/parameter_response_audit.md
```

Report conclusions:

* `PASS_PARAMETER_RESPONSE`
* `WARN_WEAK_INTERIOR_MEANING`
* `FAIL_PARAMETER_DOES_NOT_CONTROL_SPEED`
* `FAIL_PARAMETER_DOES_NOT_CONTROL_CURVATURE`
* `FAIL_INTERPOLATION_DISCONTINUITY`

Acceptance criteria:

* anchors produce distinguishable and plausible root trajectories.
* interior points interpolate smoothly.
* outside requests project to valid support points.
* no runtime/controller behavior changes.
* tests pass.

Verification:

```powershell
cmake --build build --config Debug --target pmg_cli
.\build\Debug\pmg_cli.exe --audit-parameter-response .\build\walk_2d_triangulated_t120_b234.pmg --output-csv .\build\parameter_response_audit.csv --output-md .\build\parameter_response_audit.md
ctest -C Debug -R "parameter|runtime|viewer_runtime"
```

---

### Phase A2 — Registration / Phase Alignment Audit — COMPLETE (2026-06-19)

Completed artifacts: `build/registration_phase_alignment_audit.csv` and
`build/registration_phase_alignment_audit.md`. Result:
`WARN_WEAK_CYCLE_SEAM` (5 clip rows; 10 pair rows; canonical origin/heading
verified; 4 weak seams; 0 jog pair warnings). No registration/runtime behavior
change.

Implement a report-only audit for registration and phase alignment quality.

Purpose:
Transition quality depends heavily on whether clips are phase-aligned and registered consistently. Before adding more transitions or nodes, validate the cyclic/phase assumptions.

For each source BVH / motion-space example, report:

* clip name
* frame count
* fps
* first root position and heading after canonicalization
* final relative displacement
* final heading change
* estimated cycle seam pose distance
* root velocity seam discontinuity
* yaw-rate seam discontinuity
* contact-state seam mismatch if contact data exists
* phase samples used for transition search

For pairs of anchors, report:

* root trajectory similarity
* phase alignment offset if available
* pose seam compatibility
* velocity seam compatibility
* whether pair is safe for transition sampling

Output:

```text
build/registration_phase_alignment_audit.csv
build/registration_phase_alignment_audit.md
```

Conclusions:

* `PASS_PHASE_ALIGNMENT`
* `WARN_WEAK_CYCLE_SEAM`
* `WARN_PHASE_MISMATCH_FOR_JOG`
* `FAIL_REGISTRATION_INCONSISTENT`
* `FAIL_CLIP_NOT_CYCLIC_ENOUGH`

Non-goals:

* do not change registration.
* do not add timewarping unless the audit proves it is required.
* do not reject clips yet; report first.

Acceptance criteria:

* all five current anchors are included.
* canonical origin/heading assumptions are verified.
* weak seams are identified explicitly.
* no behavior changes.

---

### Phase A3 — Reachable Target Region Audit — COMPLETE (2026-06-19)

Completed artifacts: `build/reachable_region_audit.csv`,
`build/reachable_region_audit.md`, and `build/reachable_region_maps/`. Result:
`WARN_SOURCE_DEPENDENT_REGION_SHRINKAGE` (8 source samples; 77 target samples;
no empty accepted regions). Main shrinkage case: `(0, 1)` only retains 2/77
accepted targets while most walk/turn sources retain 71-77/77. No graph or
runtime behavior change.

Implement source-parameter-dependent reachable region analysis for the current self-edge.

Purpose:
A PMG edge should encode valid transition ranges, not just a global “edge exists” flag. We need to understand which source parameters can transition to which target regions under the current metric and threshold.

Use current threshold candidate:

```text
TGOOD=120
TBAD=234
blend_frames=5
```

For each sampled source parameter:

* list accepted target parameter samples
* compute target bounding box
* compute target support coverage
* count GOOD / NEUTRAL / BAD candidates
* compute worst accepted transition distance
* compute root jump / heading jump / velocity jump
* note if target region is empty

Output:

```text
build/reachable_region_audit.csv
build/reachable_region_audit.md
build/reachable_region_maps/
```

Maps:

* per-source target region visualization if feasible
* heatmap of accepted transition density
* heatmap of worst accepted D
* highlight jog-related high-risk transitions

Conclusions:

* `PASS_REACHABLE_REGION_STABLE`
* `WARN_SOURCE_DEPENDENT_REGION_SHRINKAGE`
* `WARN_JOG_TARGET_FRAGILE`
* `FAIL_EMPTY_REGION_FOR_VALID_SOURCE`
* `FAIL_THRESHOLD_TOO_LOOSE`

Acceptance criteria:

* every major anchor retains a valid transition route.
* accepted target regions do not rely on obviously bad high-D transitions.
* source-dependent boxes are documented.
* no runtime behavior changes.

---

### Phase A4 — Threshold Visual Acceptance and Default Decision — COMPLETE (2026-06-19)

Completed artifacts: `build/threshold_default_decision.md` and
`docs/audits/threshold-default-decision-20260619.md`. Decision:
`COMMIT_120_234_AS_DEFAULT`. Quantitative evidence plus manual viewer
inspection accepted `120/234`, so the triangulated demo spec now uses
`edge_config walk_2d walk_2d 120 234 3 6 41` as the committed default.

Convert the existing `120/234` threshold candidate from experimental to either default or documented experimental.

Inputs:

* `transition_threshold_sweep.md`
* `threshold_visual_acceptance_t120_b234.md`
* new parameter response audit
* new reachable region audit

Manual viewer checklist:

* load `walk_2d_triangulated_t120_b234.pmg`
* request anchors:

  * `(-0.3, 0.0)`
  * `(0.0, 0.0)`
  * `(1.0, 0.0)`
  * `(0.0, 1.0)`
  * `(0.15, 0.75)`
  * `(0.8, 0.8)` outside request
* compare against loose `300/400` if available
* inspect pop, delayed transition, heading discontinuity, root jump

Output:

```text
build/threshold_default_decision.md
```

Decision:

* `COMMIT_120_234_AS_DEFAULT`
* `KEEP_120_234_EXPERIMENTAL`
* `TRY_INTERMEDIATE_THRESHOLD`
* `BLOCKED_BY_VISUAL_QUALITY`

If committing defaults:

* update config/spec defaults only.
* update docs.
* add regression test showing new default.
* do not change metric implementation.

Acceptance criteria:

* threshold decision is backed by quantitative and visual evidence.
* no unverified default change.

---

## Phase B — Animation quality

Goal:
Move from “PMG structure works” to “animation transitions are visually acceptable.”

### Phase B1 — Transition Montage Audit

Status: COMPLETE (2026-06-19)

Completed artifacts: `build/transition_montage_report.md` and
`build/transition_montages/transition_montage_manifest.csv`. Result:
`FAIL_VISIBLE_TRANSITION_POP` (52 ranked replay rows over 2756
source/target evaluations; accepted BAD transition at `(0.1875, 0.375) ->
(0.025, 0.875)` with `D=245.926`; 159 rejected jog/walk requests; no PMG
artifact or runtime behavior change). CLI smoke test passed.

Create a transition montage generator or viewer mode.

Purpose:
Threshold numbers are not enough. We need to see worst accepted and near-rejected transitions.

Generate montage clips for:

* worst accepted transitions under `120/234`
* worst accepted transitions under `300/400`
* high-D rejected jog/walk transitions
* outside-request projected transitions
* anchor-to-anchor transitions

Each montage should show:

* source segment
* blend window
* target segment
* transition point marker
* root trajectory overlay
* D score
* root jump
* heading jump
* velocity jump

Output:

```text
build/transition_montage_report.md
build/transition_montages/
```

If video export is too costly, implement viewer-side replay list and report screenshots/manual notes.

Conclusions:

* `PASS_TRANSITIONS_VISUALLY_ACCEPTABLE`
* `WARN_MINOR_POP`
* `WARN_JOG_TRANSITION_WEAK`
* `FAIL_VISIBLE_TRANSITION_POP`
* `FAIL_THRESHOLD_REJECTS_TOO_MUCH`

Non-goals:

* do not implement foot locking.
* do not change thresholds in this phase.
* do not change runtime semantics.

---

### Phase B2 — Contact / Foot Sliding Report-Only Metric

Status: COMPLETE (2026-06-20)

Completed artifacts: `build/test_artifacts/walk_2d/contact_transition_audit.csv`
and `build/test_artifacts/walk_2d/contact_transition_audit.md`. Result:
`WARN_CONTACT_MISMATCH_COMMON` (2021 accepted transition pairs; smoothstep
blend contact mismatch, contact-foot velocity, skate distance, and per-foot
contact confidence reported). No runtime or transition rejection change.

Implement report-only foot contact and foot sliding diagnostics.

Purpose:
Before adding foot locking or IK cleanup, measure the problem.

For each transition:

* estimate left/right foot contact frames
* detect contact mismatch across transition
* measure foot world velocity during contact
* estimate skate distance during blend
* record worst foot sliding transition
* correlate with transition metric D

Output:

```text
build/contact_transition_audit.csv
build/contact_transition_audit.md
```

Metrics:

* contact mismatch count
* max foot velocity during contact
* total skate distance over blend
* per-foot contact confidence
* transition rank by contact artifact

Conclusions:

* `PASS_CONTACT_ARTIFACT_LOW`
* `WARN_FOOT_SLIDING_VISIBLE`
* `WARN_CONTACT_MISMATCH_COMMON`
* `FAIL_NEED_CONTACT_AWARE_TRANSITION_FILTER`
* `FAIL_NEED_FOOT_LOCKING_OR_IK`

Non-goals:

* no foot locking yet.
* no IK yet.
* no transition rejection change unless explicitly approved after report.

---

### Phase B3 — Blend Window Comparison

Status: COMPLETE (2026-06-20)

Completed artifact: `build/blend_window_comparison.md`. Decision:
`KEEP_BLEND_5`. The 8-frame build rejected more sampled requests, retained
accepted BAD transitions, increased worst skate distance, and adds 0.1 s of
blend latency at 30 fps. No runtime policy change.

Compare fixed blend windows and propose a policy.

Use:

```text
blend_frames = 5
blend_frames = 8
optional: adaptive candidate
```

For each blend setting:

* rebuild artifact if required
* run transition metric audit
* run reachable region audit
* run contact audit
* run visual montage audit

Compare:

* response latency
* transition count
* worst root pop
* worst heading pop
* velocity discontinuity
* contact artifacts
* visual quality

Output:

```text
build/blend_window_comparison.md
```

Decision:

* `KEEP_BLEND_5`
* `USE_BLEND_8_FOR_DEMO`
* `USE_ADAPTIVE_BLEND_POLICY`
* `REJECT_LONGER_BLEND_DUE_TO_RESPONSIVENESS`
* `NEED_REPHASING_BEFORE_BLEND_CHANGE`

Non-goals:

* no adaptive implementation until decision is approved.
* no foot locking.

---

### Phase B4 — Optional Foot Locking / IK Cleanup

Only start this phase if Phase B2/B3 conclude that contact artifacts are significant.

Implement minimal optional post-process:

* report-controlled toggle
* off by default
* no change to PMG artifact semantics
* viewer/runtime post-process only

Possible steps:

* detect planted foot
* hold planted foot world position during contact
* adjust root or lower-limb IK if existing skeleton utilities support it
* measure before/after skate distance

Output:

```text
build/foot_locking_experiment.md
```

Decision:

* `DO_NOT_ENABLE_FOOT_LOCKING`
* `KEEP_AS_OPTIONAL_VIEWER_POSTPROCESS`
* `ENABLE_FOR_DEMO_ONLY`
* `REQUIRES_REAL_IK_MODULE`

---

## Phase C — Application demos

Goal:
Turn the PMG from a parameter-debug tool into an animation-control system.

### Phase C1 — Direct Steering Controller — COMPLETE (2026-06-21)

Completed artifact: `build/direct_steering_demo_report.md`. Result:
`PASS_DIRECT_STEERING_DEMO`. Arrow-key direction/speed input maps through the
existing calibrated `GoalDirectedLocomotion` controller, projects requests to
node support, and preserves the parameter canvas as debug control. Targeted
goal-directed/viewer tests passed (3/3).

Implement direct steering control on top of the current `walk_2d` node.

Current debug control:

```text
user clicks parameter p
```

Target application control:

```text
user gives desired direction / speed
controller maps it to parameter p
PMG runtime generates motion stream
```

Add viewer mode:

```text
Direct Steering Mode
```

Inputs:

* WASD or arrow keys
* desired forward direction
* optional desired speed
* current root heading

Controller:

* compute desired curvature from heading error
* compute speed parameter from desired speed
* clamp/project into support
* send raw parameter request to ViewerRuntimeModule

Do not remove parameter canvas. Keep it as debug mode.

Output:

```text
build/direct_steering_demo_report.md
```

Tests:

* steering request maps to valid raw parameter
* zero input decays or holds safely
* outside steering request projects safely
* runtime behavior remains deterministic

Conclusion:

* `PASS_DIRECT_STEERING_DEMO`
* `WARN_CONTROL_FEELS_DELAYED`
* `WARN_PARAMETER_MAPPING_WEAK`
* `FAIL_NOT_APPLICATION_READY`

---

### Phase C2 — Path Following Demo — COMPLETE (2026-06-21)

Completed artifact: `build/path_following_demo_report.md`. Result:
`PASS_SINGLE_NODE_PATH_FOLLOWING`. Viewer path mode appends right-clicked
waypoints, renders the polyline, and advances through it with the existing
calibrated single-node goal controller.

Implement simple lookahead path following using the current single-node PMG.

Input:

* polyline path in floor plane
* current root position and heading
* lookahead distance

Controller:

* pick lookahead target
* compute heading error
* estimate curvature
* estimate desired speed
* request parameter through runtime module

Viewer:

* draw path
* draw lookahead target
* draw current root
* draw generated trajectory
* show path tracking error

Output:

```text
build/path_following_demo_report.md
```

Metrics:

* path tracking error
* endpoint error
* transition count
* responsiveness
* visual artifacts

Conclusion:

* `PASS_SINGLE_NODE_PATH_FOLLOWING`
* `WARN_POOR_TIGHT_TURNS`
* `WARN_SPEED_CONTROL_WEAK`
* `FAIL_NEEDS_MULTI_NODE_GRAPH`

Non-goals:

* no graph search yet.
* no obstacle avoidance.
* no action sequencing.

---

### Phase C3 — Scripted Demo / Replay Export

Implement reproducible scripted control sequences.

Purpose:
Manual demos are hard to compare. Add replayable input scripts.

Script format:

```text
time, command, parameters
```

Examples:

* circle path
* S-curve
* accelerate to jog
* outside request projection
* stop/reset/restart if supported

Output:

```text
build/demo_replays/
build/scripted_demo_report.md
```

Acceptance:

* same script produces deterministic transition sequence.
* viewer can load and replay script.
* report records transition count and path metrics.

---

### Phase C4 — Compare Debug PMG vs Application Controller

Compare:

* direct parameter canvas control
* direct steering controller
* path following controller

Report:

* controllability
* responsiveness
* transition count
* animation artifacts
* user-facing limitations

Output:

```text
build/application_controller_comparison.md
```

Decision:

* `KEEP_CANVAS_AS_DEBUG_ONLY`
* `DIRECT_STEERING_IS_PRIMARY_DEMO`
* `PATH_FOLLOWING_IS_PRIMARY_DEMO`
* `NEED_MULTI_NODE_BEFORE_APPLICATION_CLAIM`

---

## Phase D — PMG expansion

Goal:
Move from a single-node parametric locomotion space to a more paper-faithful multi-node PMG.

### Phase D1 — Compatible BVH Corpus Audit — COMPLETE (2026-06-21)

Completed artifacts: `build/compatible_bvh_corpus_audit.csv` and
`build/compatible_bvh_corpus_audit.md`. Decision:
`ONLY_SINGLE_NODE_IS_JUSTIFIED`. The 41-file corpus contains only one clear
cyclic run candidate; `walkToJog.bvh` is transitional. D2-D4 remain gated.

Do not add a second node until this audit passes.

Scan available BVH corpus for:

* skeleton compatibility
* frame rate compatibility
* joint naming consistency
* locomotion/action category
* cyclicity
* root trajectory quality
* contact consistency

Classify candidates:

* `WALK_COMPATIBLE`
* `RUN_COMPATIBLE`
* `STOP_START_COMPATIBLE`
* `TURN_IN_PLACE_COMPATIBLE`
* `ACTION_COMPATIBLE`
* `REJECT_SKELETON_MISMATCH`
* `REJECT_NOT_CYCLIC`
* `REJECT_BAD_ROOT_MOTION`

Output:

```text
build/compatible_bvh_corpus_audit.csv
build/compatible_bvh_corpus_audit.md
```

Decision:

* `READY_FOR_SECOND_NODE`
* `NEED_MORE_DATA`
* `ONLY_SINGLE_NODE_IS_JUSTIFIED`

Acceptance:

* no graph topology change.
* no new node until candidate group is justified.

---

### Phase D2 — Second Parametric Node

Only proceed if Phase D1 concludes `READY_FOR_SECOND_NODE`.

Candidate priority:

1. `run_1d` or `run_2d`
2. `stop_start`
3. `turn_in_place`
4. action node such as `punch` or `reach`, only if compatible data exists

For the new node:

* define parameter meaning
* define support
* audit parameter response
* audit registration
* build intra-node self-edge
* run transition quality audit

Output:

```text
build/second_node_build_report.md
build/second_node_parameter_response_audit.md
```

Acceptance:

* new node has coherent parameter meaning.
* new node is not merely a dumping ground for incompatible clips.
* generated motion is smooth over support.

---

### Phase D3 — Inter-node Edge

Add directed edges between existing `walk_2d` and the second node.

For each edge:

* source node
* target node
* source parameter samples
* target parameter box
* transition metric stats
* reachable target region
* worst accepted transitions
* contact artifacts

Output:

```text
build/inter_node_edge_audit.md
```

Required edges depend on node:

* `walk_2d -> run`
* `run -> walk_2d`
* `walk_2d -> stop`
* `stop -> walk_2d`
* or action-specific equivalent

Acceptance:

* edge is not globally assumed valid.
* target region is source-dependent if needed.
* runtime transition works under phase gate.
* visual montage passes.

---

### Phase D4 — Graph Walk / Action Sequencing Demo

Build a small multi-node PMG application demo.

Examples:

* walk → run → walk
* walk → stop → walk
* walk to point → action → walk away
* path following with speed changes

Viewer:

* show current node
* show desired node
* show active edge
* show transition diagnostics
* show node-specific parameter canvas or controller

Output:

```text
build/multinode_pmg_demo_report.md
```

Conclusion:

* `PASS_MULTINODE_PMG_DEMO`
* `WARN_WEAK_INTER_NODE_TRANSITIONS`
* `WARN_DATA_LIMITED`
* `FAIL_NOT_PAPER_FAITHFUL_YET`

---

### Phase B1.5  Accepted-BAD Transition Root Cause Audit — COMPLETE (2026-06-20)

Completed artifacts: `build/transition_acceptance_consistency.csv` and
`build/transition_acceptance_consistency.md`. Result:
`FAIL_ACCEPTED_BAD_TRANSITION` / `FAIL_EDGE_BOX_OVERREACH` (2,756 evaluated
rows; one accepted BAD transition at `(0.1875, 0.375) -> (0.025, 0.875)`,
`D=245.926`, `TBAD=234`). Requested and effective target parameters match, so
support projection/interpolation artifact is not the cause. The fixed threshold
correctly classifies the measured transition as BAD, so a loose threshold is not
the cause. Jog/walk source-dependent shrinkage rejects and projects other cases;
it does not explain this accepted row. Root cause: interpolated edge-box
membership extends beyond the independently measured `D < TBAD` region.

Targeted CTest:
`cli_audit_transition_acceptance_consistency_walk_2d` passed (1/1).

Implement report-only acceptance-consistency audit for current self-edge.

Purpose:
Explain why a transition can be accepted by interpolated edge region while its
measured transition metric is already `>= TBAD`.

Required outputs:

```text
build/transition_acceptance_consistency.csv
build/transition_acceptance_consistency.md
```

Must report:

* source parameter
* requested target parameter
* effective/projected target parameter
* accepted-by-box flag
* actual `D`
* metric class `GOOD/NEUTRAL/BAD/REJECTED`
* acceptance violation flag `accepted_by_box && D >= TBAD`
* `D - TBAD`
* source coverage from reachable-region audit if available
* source/target phase and frame
* root/heading/velocity jump
* nearest source/target anchor labels
* note for likely cause

Must explicitly rank:

* all accepted BAD transitions
* near-threshold accepted transitions
* known bad case around `(0.1875, 0.375) -> (0.025, 0.875)`
* source-dependent shrinkage cases, especially source `(0, 1)`
* jog/walk transition cases

Conclusions:

* `PASS_ACCEPTANCE_REGION_CONSISTENT`
* `WARN_NEAR_BAD_ACCEPTED`
* `FAIL_ACCEPTED_BAD_TRANSITION`
* `FAIL_EDGE_BOX_OVERREACH`

Non-goals:

* do not change thresholds
* do not change runtime semantics
* do not add foot locking or IK
* do not proceed to controller or multi-node work

---

### Phase B1.6  Accepted-BAD Report-to-Policy Gate — COMPLETE (2026-06-21)

Selected policy: intersect target boxes from source samples with positive
interpolation weight. Empty intersections expose no transition. This is the
smallest evaluated representation change; it adds no metric evaluation or new
artifact data at runtime.

Canonical acceptance audit result:

* accepted BAD transitions: `1 -> 0`;
* accepted rows: `2512 -> 2456`;
* rejected rows: `244 -> 300`;
* accepted coverage: `91.1466% -> 89.1147%` (`-2.0319` percentage points);
* conclusion: `WARN_NEAR_BAD_ACCEPTED` (five accepted rows remain below but
  near `TBAD`);
* known failing case `(0.1875, 0.375) -> (0.025, 0.875)` is rejected and
  projected to `(0.025, 0.760583)`.

Targeted checks: `test_edge_lookup` passed; canonical artifact build passed;
acceptance-consistency audit evaluated all 2,756 rows and reported zero
`accepted_by_box && D >= TBAD` violations.

Purpose: prevent runtime acceptance of a target whose independently sampled
transition distance is `D >= TBAD`, without changing calibrated thresholds or
adding foot locking.

Evaluate, in order:

1. interpolated target-box shrink;
2. source-dependent accepted-region hull;
3. runtime target validation against sampled `D < TBAD` support;
4. stricter target-box construction in `PmgBuilder`.

Required regression: canonical `demo_walk_2d_triangulated.pmg_spec` produces
zero `accepted_by_box && D >= TBAD` rows while preserving deterministic seed,
reported coverage, artifact metadata, and existing GOOD transitions. Record
rejected count and coverage change; do not silently skip candidates.

Policy selection rule: choose smallest representation/runtime change that
eliminates accepted BAD rows. Keep report-only audit as verification oracle.
Do not add foot locking or IK in this phase.

---

## Final order

### Feasible paper-reimplementation path — DECIDED (2026-06-20)

Current implementation already covers the paper's core mechanism: registered
parametric motion spaces, sampled edge construction, double-threshold reachable
target boxes, k-nearest edge lookup, and aligned runtime blending. Current
evidence is limited to a single 2-D locomotion node and self-edge; it does not
yet demonstrate the paper's multi-node graph or applications.

Use this critical path:

```text
C1 Direct Steering Controller
C2 Path Following Demo
D1 Compatible BVH Corpus Audit
D2 Second Parametric Node
D3 Inter-node Edge
D4 Graph Walk / Action Sequencing Demo
```

Defer C3, C4, and B4. Add them only if replay evidence, controller comparison,
or measured contact artifacts require them. Acceptance target is a faithful
functional reimplementation of the published method, not reproduction of the
paper's unavailable corpus, exact timings, graph sizes, or visual results.

Before new implementation, verify the existing multi-config build with:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

Run phases in this order:

```text
A1 Parameter Response Audit
A2 Registration / Phase Alignment Audit
A3 Reachable Target Region Audit
A4 Threshold Visual Acceptance

B1 Transition Montage Audit
B1.5 Accepted-BAD Transition Root Cause Audit
B1.6 Accepted-BAD Report-to-Policy Gate
B2 Contact / Foot Sliding Audit
B3 Blend Window Comparison
B4 Optional Foot Locking / IK, only if justified

C1 Direct Steering Controller
C2 Path Following Demo
C3 Scripted Demo / Replay Export
C4 Application Controller Comparison

D1 Compatible BVH Corpus Audit
D2 Second Parametric Node
D3 Inter-node Edge
D4 Graph Walk / Action Sequencing Demo
```

Do not reorder D before A/B. Multi-node expansion without parameter-response and transition-quality evidence will reduce paper faithfulness rather than improve it.
