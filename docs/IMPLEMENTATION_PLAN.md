# PMG Implementation Status

## Completed Paper-Core

- Kovar-style windowed point-cloud distance and floor-plane alignment.
- Full transition-region distance grid and optimal transition phases.
- Seeded source/target sampling with GOOD/NEUTRAL/BAD classification.
- Conservative AABB shrink and whole-edge rejection.
- k-nearest edge lookup and runtime point-cloud alignment.
- Contact-anchored registration, slope-constrained DTW refinement, and
  root-delta motion synthesis.
- KG04-style parameter-accuracy calibration (turn-rate metric, 1-D nodes) and
  parameter-dependent blended clip durations.
- V5 complete offline artifact with V2/V3/V4 read compatibility.
- Artifact-driven random walk, goal-directed locomotion, and viewer playback.
- Real-BVH one-node, two-node, selective-region, and box-shrink specs.
- Reproducible build reports (`config.json`, `metrics.json`, Markdown, CSV).

## Verification

The canonical command is:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

Real-corpus CLI tests cover registration quality, edge construction, random
streaming, and goal-directed locomotion. Artifact-specific tests cover V5
round-trip, V2/V3/V4 compatibility, outgoing-edge selection, semantic control,
and parameter-calibration round-trip.

## Remaining Work (priority order)

Deviation ids reference [PAPER_CONFORMANCE.md](PAPER_CONFORMANCE.md). Motion
spaces come first: the graph layer is near-conformant, while motion-space
fidelity gates the paper's parameter-accuracy claim.

1. Transition window unification (D3, D4): tie blend length to the metric
   window and keep the source playing through the whole blend.
2. Per-target transition phases (D5): stop collapsing in-box GOOD phases to
   one average per source sample.
3. Multi-dimensional and multi-metric parameter calibration (D1 currently
   covers 1-D turn-rate nodes); revisit whether goal-directed streaming
   calibration can be retired once self-transition slicing is addressed.
4. Mid-clip transitions, partial source-domain edges, and continuously
   changing parameters.
5. Better global target search and explicit reachability maps.
6. Mesh-vertex point clouds (D6), automatic extraction/parameterization,
   BVH export, skinned rendering, optional learned validity.
