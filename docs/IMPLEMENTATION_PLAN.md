# PMG Implementation Status

## Completed Paper-Core

- Kovar-style windowed point-cloud distance and floor-plane alignment.
- Full transition-region distance grid and optimal transition phases.
- Seeded source/target sampling with GOOD/NEUTRAL/BAD classification.
- Conservative AABB shrink and whole-edge rejection.
- k-nearest edge lookup and runtime point-cloud alignment.
- Contact-anchored registration, slope-constrained DTW refinement, and
  root-delta motion synthesis.
- KG04-style multidimensional parameter-accuracy calibration (turn rate and
  travel speed) and parameter-dependent blended clip durations.
- Unified metric/runtime transition windows and cycle-continuous source
  playback through blends.
- Per-target transition phase fields over retained GOOD samples.
- V7 complete offline artifact with V2-V6 read compatibility.
- Artifact-driven random walk, goal-directed locomotion, and viewer playback.
- Real-BVH one-node, two-node, selective-region, and box-shrink specs.
- Reproducible build reports (`config.json`, `metrics.json`, Markdown, CSV).

## Verification

The canonical command is:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

Real-corpus CLI tests cover registration quality, edge construction, random
streaming, and goal-directed locomotion. Artifact-specific tests cover V7
round-trip, V2-V6 compatibility, outgoing-edge selection, semantic control,
parameter-calibration round-trip, and per-target phase round-trip.

## Remaining Work (priority order)

Deviation ids reference [PAPER_CONFORMANCE.md](PAPER_CONFORMANCE.md). Motion
spaces come first: the graph layer is near-conformant, while motion-space
fidelity gates the paper's parameter-accuracy claim.

1. Mid-clip transitions, partial source-domain edges, and continuously
   changing parameters.
2. Revisit whether goal-directed streaming calibration can be retired once
   self-transition slicing is addressed.
3. Better global target search and explicit reachability maps.
4. Mesh-vertex point clouds (D6), automatic extraction/parameterization,
   BVH export, skinned rendering, optional learned validity.
