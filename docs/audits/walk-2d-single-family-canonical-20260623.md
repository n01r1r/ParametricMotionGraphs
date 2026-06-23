# walk_2d Single-Family Canonical Demo — 2026-06-23

## Phase

Discontinuity-fix track, phase 2 (cause #1: cross-family example inside one
motion space). Acts on the verdict of
`jolt-cross-family-diagnosis-20260623.md`.

## Change

The canonical viewer/CTest demo no longer blends a jog into the walk space.

- `specs/demo_walk_2d_singlefamily.pmg_spec` -> `specs/demo_walk_2d.pmg_spec`
  (canonical, four walking-family anchors: walkMoreCurve, walkCurve,
  walkTightCurve, walkStraightTwiceAsFast).
- `specs/demo_walk_2d_triangulated.pmg_spec` (the old jog-in-hull spec) ->
  `specs/fixtures/fixture_walk_2d_jog_crossfamily.pmg_spec`, retained as a
  cross-family stress fixture, not a viewer demo.
- Re-pointed CMake (validate/build/goto), root + specs READMEs,
  `test_root_canonicalization` (NumExamples 5 -> 4).
- No runtime code changed. No region is guarded or hidden; the bad data was
  removed from the canonical space, not masked.

The speed axis now tops out at the authored `walkStraightTwiceAsFast` anchor
(~0.75); the old `[0,1]` jog corner is intentionally dropped. It was the
cross-family jolt source, not a usable walking speed.

## Probe-test re-derivation

The single-family graph has no cross-family BAD region, so the old jog-corner
reject probes now accept. Expectations were re-derived against the actual graphs
(honest, observed decisions):

| test | artifact | request | decision |
|---|---|---|---|
| `cli_probe_transition_self_good` | canonical | [0,0]->[0,0] | accept |
| `cli_probe_transition_outside_projection` | canonical | [0,0]->[0,1] (out-of-hull) | reject (outside_target_box) |
| `cli_probe_transition_fail_vis_001` | jog fixture | [-0.04,0.8]->[0.5,0] | reject (root_speed) |

The cross-family quality-gate reject only exists where a jog sits in the walk
hull, so that probe now targets the stress fixture, where the fail genuinely
lives.

## Verification (canonical vs jog fixture)

Registration phase-alignment (pose seam, the within-hull jolt signal):

| pair set | canonical | jog fixture |
|---|---|---|
| walk x walk / walk x fast-walk | 2.6 - 50.6, all GOOD, PASS | same GOOD |
| anything x jog | none | 217 - 246, BAD/NEUTRAL, WARN_PHASE_MISMATCH_FOR_JOG (4 pairs) |

Canonical: 6/6 authored pairs `PASS_PAIR_PHASE_ALIGNMENT`, no unsafe pair.

Acceptance-consistency (edge-box overreach):

| metric | canonical | jog fixture |
|---|---|---|
| conclusion | PASS_NO_ACCEPTED_BAD_TRANSITIONS | FAIL_OVERREACH_REMAINS |
| accepted_bad_by_box | 0 | 5 |
| accepted_bad_after_quality_gate | 0 | 3 |

So the jog anchor was the entire source of the long-standing
`FAIL_OVERREACH_REMAINS` (accepted-bad 3). Removing it from the canonical demo
drops accepted-bad to 0 and turns the suite fully green.

`ctest -C Release`: 49/49 (was 47/48; the formerly-failing
`cli_audit_transition_acceptance_consistency_walk_2d` now passes, and a build
test for the jog fixture was added).

## Residual / next

Both specs still report `WARN_WEAK_CYCLE_SEAM` on the per-clip rows: a loop-seam
(cyclic-continuity) concern, present independent of the cross-family issue. That
is cause #4 in this track and is addressed in a later phase. `walk_2d` declares
`registration ... -` (no cycle joint), so `--audit-cyclic-continuity` returns no
rows for it; loop-seam continuity is currently unmeasured for this node.
