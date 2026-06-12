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

- `CalibrateParameterMetrics` accepts one metric per parameter axis. For
  multidimensional spaces it samples a deterministic regular grid over the
  authored parameter domain, generates each local blend, and stores its
  measured metric vector plus full example-weight vector.
- Measured-space distances are normalized by each metric's sampled range.
  Implemented metrics are `turn_rate` (signed wrapped root-heading change per
  second) and `travel_speed` (mean root floor-path speed in native BVH
  units/second).
- The one-dimensional path retains parameter-adjacent, monotone-segment
  sampling.
- `ComputeLocalBlendWeights` maps a requested authored coordinate to the
  anchor-interpolated measured vector, then locally inverts the sampled map.
  Spaces without declared metrics keep Shepard weights.
- `GenerateClip(parameter, fps)` derives its frame count from
  `BlendedDurationSeconds` (weighted example durations). Frame-aligned
  diagnostics use `pmg::legacy::GenerateClipWithFrameCount`.
- Scalar calibration serialization was introduced in `PMG_GRAPH_V5`; V6 added
  per-target transition phases. `PMG_GRAPH_V7` stores vector metrics, measured
  samples, metric scales, and full example weights. V5/V6 scalar tables are
  converted while reading.
- Spec syntax: `parameter_metric <node> <turn_rate|travel_speed|none>` remains
  as the one-dimensional compatibility form. Multidimensional nodes use
  `parameter_metrics <node> <metric0> ... <metricN-1>`.
  `parameter_calibration <node> <samples_per_axis>` exposes deterministic grid
  density; the default is 9.

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
- Calibration cost grows as `samples_per_axis ^ parameter_dimension` and is
  capped at 100,000 grid samples. Local scattered inversion estimates
  parameter accuracy under this motion-space model; it does not establish a
  globally one-to-one parameterization.
