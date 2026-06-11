# PMG Implementation Status

## Completed Paper-Core

- Kovar-style windowed point-cloud distance and floor-plane alignment.
- Full transition-region distance grid and optimal transition phases.
- Seeded source/target sampling with GOOD/NEUTRAL/BAD classification.
- Conservative AABB shrink and whole-edge rejection.
- k-nearest edge lookup and runtime point-cloud alignment.
- Contact-anchored registration, slope-constrained DTW refinement, and
  root-delta motion synthesis.
- V4 complete offline artifact with V2/V3 read compatibility.
- Artifact-driven random walk, goal-directed locomotion, and viewer playback.
- Real-BVH one-node, two-node, selective-region, and box-shrink specs.
- Reproducible build reports (`config.json`, `metrics.json`, Markdown, CSV).

## Verification

The canonical command is:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

Real-corpus CLI tests cover registration quality, edge construction, random
streaming, and goal-directed locomotion. Artifact-specific tests cover V4
round-trip, V2/V3 compatibility, outgoing-edge selection, and semantic control.

## Remaining Work (priority order)

Deviation ids reference [PAPER_CONFORMANCE.md](PAPER_CONFORMANCE.md). Motion
spaces come first: the graph layer is near-conformant, while motion-space
fidelity gates the paper's parameter-accuracy claim.

1. Parameter-accurate blend weights (D1): sample blend-weight space, map
   weights to achieved parameters, invert per K&G04. Should retire the
   goal-directed turn-rate calibration.
2. Parameter-dependent clip duration (D2): blended cycle time = weighted sum
   of time-warped example durations.
3. Transition window unification (D3, D4): tie blend length to the metric
   window and keep the source playing through the whole blend.
4. Per-target transition phases (D5): stop collapsing in-box GOOD phases to
   one average per source sample.
5. Mid-clip transitions, partial source-domain edges, and continuously
   changing parameters.
6. Better global target search and explicit reachability maps.
7. Mesh-vertex point clouds (D6), automatic extraction/parameterization,
   BVH export, skinned rendering, optional learned validity.
