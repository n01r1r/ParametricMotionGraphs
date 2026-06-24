# Calibration-Grid Wobble — 2026-06-24

## Phase

Discontinuity-fix track, follow-up to cause #2. The
[blend-weight C1 measurement](blend-weight-c1-measurement-20260623.md) closed the
*triangulation* C0 (simplex faces not the dominant kink) but attributed the
residual background to "the 5x5 inverse-calibration grid boundaries **and**
frame-quantization noise" without separating the two. This audit isolates the
calibration grid as the dominant parameter-axis cause and acts on it.

## Question

Felt motion unnaturalness on `walk_2d` was first suspected to be a time/frame or
transition-window artifact. Two-axis disambiguation: the wobble is on the
**parameter axis** (velocity ripple as the requested parameter sweeps), not the
time axis (per-frame) or the transition window. Given that, which
parameter-axis source dominates — the inverse-calibration grid C0, the
triangulation C0, hull projection, or frame-quantization noise?

## Method

`--audit-parameter-response --samples 41` on the canonical artifact, holding the
sample count fixed and varying **only** `parameter_calibration N` (the inverse
grid knots per axis: N = 3, 5, 9, 13). The parameter-axis non-smoothness metric
is the RMS of the second difference of average speed along the response sweep
(velocity ripple). If the grid C0 dominates, the metric falls monotonically as N
rises; if frame-quantization noise dominates, it floors out independent of N.

Corroborating: `--audit-cyclic-continuity` for the loop seam (cause confounder),
and the existing triangulation/projection result from the prior doc.

## Result

| grid N | RMS speed 2nd-diff | max 2nd-diff |
|--------|--------------------|--------------|
| 3      | 0.35               | 2.17         |
| 5      | 0.31               | 1.81         |
| 9      | 0.23               | 0.95         |
| 13     | 0.16               | 0.54         |

- **Monotone with grid density** — the ripple is the inverse-calibration grid C0
  at cell boundaries, not frame-quantization noise (which would not respond to N).
  The prior doc's "below the noise floor" verdict mislabeled the grid signal: a
  ~0.3 floor is real, but the cal3/cal5 grid signal sat **above** it.
- Convergence is **O(1/N)** — halving the wobble needs doubling N. Grid density
  alone has diminishing returns; a C1 inverse interpolant is the real lever.
- Anchors stay **exact** across all grids (one-hot blend weights,
  achieved-speed delta 0), so refining the grid carries no accuracy regression —
  interior values converge while the four authored corners stay pinned.
- Loop seam measured clean: `--audit-cyclic-continuity` = 0 weak_pose /
  0 weak_root_speed, seam_step_ratio 0.13–0.59 << 1; the only flag is
  `weak_yaw_rate` (natural intra-stride turn-rate variation, already explained by
  the [loop-seam recut](walk-2d-loop-seam-recut-20260623.md)). Not a confounder.
- Triangulation C0 and hull projection: 0 alignment with the residual kinks
  (prior doc). Not the cause.

## Decision

Two changes, both landed on `dev/misc`, 49/49 ctest green:

1. **(B) `parameter_calibration 5 -> 9`** in `specs/demo_walk_2d.pmg_spec`.
   9 is the smallest grid whose RMS (0.23) drops below the ~0.3 frame-quantization
   floor; finer grids (13) only chase the floor at O(1/N) cost. Evidence comment
   inline in the spec.

2. **(C-1) Extract the inverse map into a deep module.** The inverse-distance
   lookup moved from `ParametricMotionSpace::CalibratedBlendWeights` into
   `ParameterCalibration::BlendWeightsFor(uncalibrated_weights)` — pure over the
   stored table, directly unit-testable without a motion space, clip, or artifact.
   The space now delegates one line: it owns the support geometry (uncalibrated
   weights), the calibration owns the inverse measured-parameter map. The move is
   behavior-preserving (verbatim body); a new seam test in
   `tests/test_parametric_motion_space.cpp` asserts
   `BlendWeightsFor == ComputeLocalBlendWeights` and a convex (sum-to-one) blend.

`BlendWeightsFor` is now the swap point for a **C1 inverse interpolant** — the
piecewise-linear lookup can be replaced there without touching the space, and its
parameter-axis smoothness is unit-testable in isolation. That is the next lever
once the corpus justifies it (the O(1/N) data says the grid alone won't reach a
flat response).

## Track status

Reopens and resolves the calibration half of cause #2 (the prior doc closed only
the triangulation half):
- #2a triangulation C0: not dominant, no rewrite (prior doc).
- #2b calibration grid C0: **dominant on the parameter axis**; grid 5 -> 9 +
  `BlendWeightsFor` deep seam (this doc).
Residual (out of track): a C1 inverse interpolant behind `BlendWeightsFor`,
deferred until a corpus shows the grid-9 residual is perceptible.
