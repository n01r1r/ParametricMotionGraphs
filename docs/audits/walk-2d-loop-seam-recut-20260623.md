# walk_2d Loop-Seam Recut — 2026-06-23

## Phase

Discontinuity-fix track, phase 4 (cause #4: loop-seam / cyclic continuity).
Acts only on the canonical single-family `specs/demo_walk_2d.pmg_spec`. No
runtime code, no transition guarding, no blend-weight change; jog not
reintroduced.

## Problem

`walk_2d` declared `registration ... -` (no cycle joint), so
`--audit-cyclic-continuity` returned zero rows: the loop seam (last frame ->
first frame of a cycle, what repeats when the runtime stays in the node) was
unmeasured. The separate registration-phase-alignment audit had been flagging
`WARN_WEAK_CYCLE_SEAM` on the full-clip anchors.

## Measurement

Declaring `cycle_joint LeftAnkle` enabled the audit (9 rows: 4 authored, 5
generated). With naive first-cycle auto-extraction the pose seam was weak across
the board (`weak_pose_seam`, seam-step ratio up to 2.74), and the auto-cropped
cycles were so short (16-29 frames) that **every edge sample was rejected and no
transition could be built** -- which is why the original demo kept full clips.

## Fix: foot-strike-aligned cycle windows (recut)

For each windowless anchor, `--extract-candidate-windows` ranked inclusive frame
windows by rigid-aligned endpoint pose similarity. The chosen windows
(start_frame/end_frame baked into the spec, so the builder uses them verbatim):

| anchor | param | window | endpoint pose dist |
|---|---|---|---|
| walkMoreCurve | [-0.3,0] | 1-34 | 0.064 |
| walkCurve | [0,0] | 127-160 | 0.012 |
| walkTightCurve | [1,0] | 33-67 (kept) | n/a (tight turn) |
| walkStraightTwiceAsFast | [0.15,0.75] | 27-60 | 0.010 |

walkTightCurve has no candidate window: its per-cycle heading delta exceeds the
extractor's `max_heading_delta_radians` filter (a tight turn rotates a lot per
stride), so its hand-authored 33-67 window is retained.

## Result (seam-step ratio = pose pop relative to median in-clip step)

| sample | before (auto first-cycle) | after (recut) |
|---|---|---|
| authored walkMoreCurve | 2.74 weak_pose_seam | 0.54 |
| authored walkCurve | 2.67 weak_pose_seam | 0.33 |
| authored walkStraightTwiceAsFast | weak (16-frame crop) | 0.16 |
| generated [-0.3,0] | 2.74 weak_pose_seam | 0.54 |
| generated [-0.3,0.75] | 1.36 | 0.19 |
| generated [0.35,0.375] | 1.01 | 0.13 |

Pose loop seam now well under the 2.0 threshold everywhere, authored and
generated. The dominant visible loop pop is removed.

## Residual: weak_yaw_rate

Every row still classifies `weak_yaw_rate` (yaw-rate ratio 1.6-3.4 vs the 1.5
threshold). This is not a pose pop:

- walkMoreCurve 1.80 -> 1.13 rad/s, walkTightCurve -1.67 -> -1.02: same-sign,
  natural intra-stride easing of turn rate through a curving stride.
- walkCurve -0.40 -> +0.54 rad/s: sign wobble around zero (it is the
  straight-ish walk), a small-magnitude ratio artifact, not a real reversal.

Removing it would need a yaw-rate-continuity term added to the candidate-window
scorer, which risks regressing the pose alignment just fixed. Not pursued: the
perceptible loop artifact (pose) is resolved, and the yaw signal is intra-stride
variation inherent to windowing a continuously-turning gait.

## Acceptance

- cyclic audit non-empty: 9 rows (was 0).
- loop seam reported before/after (above).
- `ctest -C Release`: 49/49, no regression.
- canonical accepted-BAD: 0 (`PASS_NO_ACCEPTED_BAD_TRANSITIONS`), unchanged.
- transition montage conclusion unchanged: `FAIL_THRESHOLD_REJECTS_TOO_MUCH`
  before and after (a pre-existing report-only proxy; the command exits 0). Raw
  accept/reject moved 2116/46 -> 2020/142: shorter single-cycle clips give the
  self-edge a smaller reachable box, so coverage dips slightly. This is the
  inherent single-clean-cycle vs transition-coverage tradeoff; the verdict label
  did not change and accepted-BAD stayed 0.
