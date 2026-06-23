# TimeWarp C1 Monotone-Hermite — 2026-06-23

## Phase

Discontinuity-fix track, phase 1 of the `#1..#6` priority list
([paper-faithfulness gap discussion]). Target cause: temporal discontinuity
introduced by the registration time warp itself.

## Problem

`TimeWarp` mapped the canonical phase domain onto each example's phase domain
with **piecewise-linear** interpolation between contact knots. Piecewise-linear
is C0: phase *velocity* (dy/dx) jumps at every interior knot. Because
`ParametricMotionSpace::EvaluatePoseFromWeights` samples each example through its
warp, that velocity kink propagates into every blended/generated clip as a small
temporal jerk at the contact knots — present even for a single in-family example,
independent of blend-weight or cross-family issues.

## Fix

Replace the linear segment evaluation with **monotone cubic Hermite**
(Fritsch-Carlson tangents), the strictly-increasing spline of KG04 §4.1:

- passes through every knot (registration anchors unchanged),
- never overshoots, so a monotone mapping stays monotone (phase cannot run
  backwards),
- C1 across knots → continuous phase velocity → no kink.

Tangents precomputed once in `FromAnchors`; `Evaluate` does the Hermite basis on
the containing segment. Identity warp and degenerate (zero-span) segments are
handled explicitly.

Files: `include/pmg/TimeWarp.h`, `src/TimeWarp.cpp`, `tests/test_time_warp.cpp`.

## Verification

- `ctest -C Release`: 47/48.
- The one failure, `cli_audit_transition_acceptance_consistency_walk_2d`
  (`FAIL_OVERREACH_REMAINS`, accepted-bad-after-gate 3), is pre-existing on clean
  `main` (see `jolt-cross-family-diagnosis-20260623.md`); accepted-bad count is
  unchanged by this work, i.e. C1 did not regress it. It is a separate edge-box
  overreach concern scheduled later in the same track.

## Fixture follow-on

C1 evaluation shifts registered-phase pose distances up ~3 raw units, so
`specs/fixtures/fixture_edge_selective_good_bad.pmg_spec` raises its D6 raw-sum
band +5 (203/210 -> 208/215) to keep the GOOD/NEUTRAL/BAD split selective. The
reachable box still shrinks (~0.87); classification semantics are preserved.

## Scope

Time-warp continuity only. Blend-weight C0 behavior (linear barycentric +
Shepard, a separate velocity-kink source across simplex faces) is a later phase
and is intentionally untouched here.
