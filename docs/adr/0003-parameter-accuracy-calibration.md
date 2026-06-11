# ADR-0003: Parameter Accuracy via Measured-Metric Calibration

## Status

Accepted

## Context

PMG inherits its motion spaces from Kovar & Gleicher 2004, where requested
parameters are accurately achieved by inverting a sampled map from blend
weights to measured motion properties. The previous implementation used
Shepard (inverse-distance) weights over the parameter axis directly, which
assumes the parameter is linear in the weights. It is not: goal-directed
locomotion had to calibrate achieved turn rates separately, and the requested
curvature did not equal the produced curvature (deviation D1). Generated
clips also used a fixed frame count, distorting playback speed across the
parameter range (deviation D2).

## Decision

- `CalibrateParameterMetric` samples the blend weight between every
  parameter-adjacent example pair of a 1-D space, generates each blend,
  measures a declared metric (`turn_rate`: net wrapped root-heading change per
  second), and stores the curve forced monotone toward its endpoints.
- `ComputeLocalBlendWeights` inverts that table: a requested parameter maps to
  the anchor-interpolated measured value, and the blend weight that achieves
  it. Spaces without a declared metric keep Shepard weights.
- `GenerateClip(parameter, fps)` derives its frame count from
  `BlendedDurationSeconds` (weighted example durations). The explicit
  frame-count overload remains for frame-aligned diagnostics.
- The calibration serializes with the space; introduced in `PMG_GRAPH_V5`.
  Current `PMG_GRAPH_V6` adds per-target transition phases.
- Spec syntax: `parameter_metric <node> <turn_rate|none>`, restricted to
  one-dimensional nodes.

## Consequences

- Requested parameters now carry measured meaning on calibrated nodes; the
  walk corpus achieves anchor-interpolated turn rates from generated clips.
- Duration-true clips raised absolute point-cloud distances (the old fixed 48
  frames stretched one gait cycle to 1.6 s), so spec thresholds were
  recalibrated: walk self-edges 1.0/1.4 -> 1.5/2.0, walk-jog 3.0/4.0 ->
  3.5/4.5, jog self 1.5/2.0 -> 2.0/2.5, selective case 0.75/0.8 -> 1.3/1.4.
- Goal-directed locomotion keeps its streamed-rate calibration: per-cycle
  self-transitions replay only a phase slice, so achieved streamed rates still
  differ from clip rates. Retiring it depends on transition-window work
  (D3-D5), not on weight accuracy.
- Multi-dimensional spaces and additional metrics require extending the
  segment table; this ADR covers the 1-D turn-rate case.
