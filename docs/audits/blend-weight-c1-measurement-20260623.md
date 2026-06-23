# Blend-Weight C1 Measurement — 2026-06-23

## Phase

Discontinuity-fix track, phase 2 (cause #2: blend weights are C0 at simplex
faces). Per user direction this is **measurement only** -- no C1 rewrite unless a
kink is measured as visible. Conclusion: no rewrite.

## Question

`ParametricMotionSpace::ComputeLocalBlendWeights` uses barycentric weights over
the triangulated support (and a sampled inverse-calibration table). Both are
piecewise-linear, i.e. C0: the weight *derivative* jumps when the requested
parameter crosses a triangle face (or a calibration cell boundary). As the
runtime steers and the parameter sweeps across a face, that derivative jump could
appear as a velocity kink in the generated motion. Phase 1 already made the
registration time warp C1; this asks whether the residual weight C0 is visible.

## Method

`--audit-parameter-response` on the canonical artifact at `--samples 41` (828
finite samples over the hull), which reports per-sample root-trajectory
descriptors (average speed, signed curvature) plus the blend weights. The
canonical support has one interior face, edge 1-3 (from [0,0] to [0.15,0.75]),
which crosses the row p1=0.375 at p0=0.075. The face C0 kink, if visible, is a
localized spike in the second difference of average speed along that row at
p0=0.075, standing out above the smooth-interior background.

## Result

- Audit verdict: `PASS_PARAMETER_RESPONSE`, finite/valid yes, 828/828 finite.
- Along p1=0.375, average speed varies ~14.4-15.4 (about 7%).
- Second difference of speed at the face crossing (p0=0.075-0.090): 0.20 / -0.17.
- Second difference at several **non-face** points is larger: 0.44 at p0=0.48,
  0.50 at p0=0.545, 0.29 at p0=0.35.

The face crossing is therefore **not** the dominant non-smoothness; its second
difference is below the background, which is driven by the 5x5 inverse-calibration
grid boundaries and frame-quantization noise in the measured speed, not by the
triangulation. There is no prominent, localized velocity kink attributable to the
simplex-face blend-weight C0.

Corroborating evidence:
- Runtime `--goto` popping (the parameter sweeps continuously across faces while
  steering) stays at pop_ratio ~1.2-2.1, far below the 8.0 gate.
- Phase 1 already removed the dominant within-clip kink (time-warp C0 -> C1).

## Decision

No C1 blend-weight rewrite. Barycentric interpolation over scattered examples is
the standard KG04 synthesis method; its C0-at-faces here is below the
calibration-grid / measurement-noise floor and not perceptible. A C1 scattered
interpolant (Sibson/Farin or RBF) would add real complexity for no measured gain
and risks perturbing the parameter accuracy the calibration provides. Revisit
only if a future corpus shows a measured face-localized kink above background.

## Track status

Closes the discontinuity-fix track #1-#6:
- #1 cross-family: removed (single-family canonical) -- single-family doc.
- #2 blend-weight C1: measured, no rewrite (this doc).
- (time-warp C1: done, timewarp-c1-monotone-hermite doc.)
- #4 loop-seam: recut to clean cycle windows -- loop-seam-recut doc.
- #5 edge-box overreach: resolved with #1 (accepted-bad 3 -> 0).
- #6 goto arrival-ease: default fixed -- goto-arrival-ease doc.
Residual (out of this track): targets-behind min-turning-radius geometry;
per-clip weak_yaw_rate intra-stride variation.
