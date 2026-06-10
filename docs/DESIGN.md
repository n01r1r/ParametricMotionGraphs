# Design Notes

## Purpose

The Phase-1 implementation reproduces the original Parametric Motion Graph concept with minimal moving parts.

> Status note: this file is the original design sketch. The implementation has
> since reached a paper-faithful core (Phases A–E); see
> `docs/IMPLEMENTATION_PLAN.md` for current progress and known deviations.

The central representation is:

```text
ParametricMotionSpace: parameter p, phase φ → pose (blending-based synthesis)
MotionDistance:        Kovar'02 point-cloud metric + closed-form 2D rigid align
ParametricEdge:        source parameter p_s → target AABB + transition phases (+ optional alignment)
ParametricMotionGraph: nodes + directed parametric edges; k-NN edge interpolation
RuntimeController:      current state + control request → aligned, C¹-blended transition
```

## Inputs

- BVH files with a shared skeleton hierarchy when used together in one parametric motion space.
- Example parameters supplied by the caller, such as curvature values `[-1, 0, +1]`.

## Outputs

- Runtime poses sampled from generated motion clips.
- Optional generated PMG edges built from sampled transition quality.

## Assumptions

- BVH clips in one motion space have compatible skeletons.
- Each motion space has a low-dimensional continuous parameter vector.
- Nearby parameters are expected to generate nearby motions.
- Phase is normalized to `[0, 1]`.
- Rotations are stored as local joint quaternions.
- Offsets and root position use the BVH file's **native units**; no unit scaling
  is applied at load. Display scaling is render-time only (viewer `display_scale_`),
  so the metric, alignment, and thresholds stay in native units (paper-scale).

## Failure Modes

The code throws `std::runtime_error` when:

- a BVH file cannot be opened;
- required BVH tokens are missing;
- a motion clip is empty;
- pose joint counts mismatch;
- parameter dimensions mismatch;
- PMG runtime is started with invalid node or parameter.

## Current Limitations

- Transition distance is the faithful Kovar'02 point-cloud metric (yaw + floor
  translation aligned) — no longer simplified. Remaining metric gaps: no
  foot-contact term; clip blending is not time-registered (deviation D6).
- AABB shrinking is conservative, single-dimension, minimal (Fig 4c).
- k-NN falloff cutoff uses the (k+1)-th neighbor (Allen'02 intent), not the paper's
  literal `lₖ` (deviation D4).
- Graph persistence (Phase F) and the control applications (Phase G) are not built.
- Rendering lives only in the optional `pmg_viewer` target; `pmg_core` is dependency-free.

## Extension Boundary

Do not add learned validity, manifold embedding, CUDA, or contact-aware metrics into
`pmg_core` unless a concrete limitation is observed and documented (see
`docs/PHASE2_NOTES.md`). Keep third-party deps out of `pmg_core`.
