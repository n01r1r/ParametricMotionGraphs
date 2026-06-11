# ParametricMotionGraphs

Paper-core reference implementation of **Parametric Motion Graphs** (Heck &
Gleicher, 2007) using BVH motion clips.

The implementation covers the paper's transition construction and lookup:

```text
BVH examples
  -> cycle normalization + contact/DTW registration
  -> parametric motion spaces
  -> point-cloud transition distance grids
  -> sampled GOOD/NEUTRAL/BAD target regions
  -> interpolated PMG edges
  -> V5 offline artifact
  -> point-cloud-aligned online runtime
```

It is not a reproduction of the paper's unavailable boxing/platform datasets
or Kovar-Gleicher's automatic database extraction. Claims are limited to the
included BVH corpus and the implemented acquisition/runtime assumptions.

## Build

```powershell
cmake -S . -B build -DPMG_BUILD_VIEWER=OFF
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

The viewer is optional:

```powershell
cmake -S . -B build -DPMG_BUILD_VIEWER=ON
cmake --build build --config Debug --target pmg_viewer
.\build\Debug\pmg_viewer.exe .\outputs\paper_core_walk\artifact.pmg
```

## Offline Artifact

Graph specs explicitly record registration and edge sampling:

```text
node walk 1
registration walk LeftAnkle LeftAnkle,RightAnkle 3 1
parameter_metric walk turn_rate
example walk 0.0 ../BVH/walkCurve.bvh
example walk 0.5 ../BVH/walkMoreCurve.bvh
example walk 1.0 ../BVH/walkTightCurve.bvh
edge walk walk
edge_config walk walk 1.5 2.0 12 60 7
```

`parameter_metric` enables KG04-style parameter accuracy: the build samples
blend weights between adjacent examples, measures each blend's turn rate, and
stores the inversion table so requested parameters achieve their measured
meaning. Generated clips also derive their length from the blended example
durations, so cycle time follows the parameter.

Build a complete V5 artifact:

```powershell
.\build\Debug\pmg_cli.exe --build-graph `
  .\specs\walk_curvature.pmg_spec `
  .\outputs\paper_core_walk\artifact.pmg
```

The output directory contains:

```text
outputs/paper_core_walk/
|-- artifact.pmg
|-- config.json
|-- metrics.json
|-- report.md
`-- tables/
    `-- edge_samples.csv
```

V5 stores the Skeleton, registered motion spaces, TimeWarps, parameter
calibrations, transition samples, runtime sampling rate, source paths, seeds,
thresholds, and edge build reports. V4 files remain readable (without
parameter calibrations); V2/V3 remain readable but lack the Skeleton required
for standalone point-cloud runtime alignment.

## Runtime

Both commands accept either a spec or a built artifact:

```powershell
.\build\Debug\pmg_cli.exe --random-walk `
  .\outputs\paper_core_walk_jog\artifact.pmg --seconds 30

.\build\Debug\pmg_cli.exe --goto `
  .\outputs\paper_core_walk\artifact.pmg 10 10 `
  --seconds 60 --tolerance 3

.\build\Debug\pmg_cli.exe --goto `
  .\outputs\paper_core_walk\artifact.pmg 10 10 `
  --facing-degrees 90 --facing-tolerance-degrees 15
```

Random walk selects only actual outgoing edges from the current node.
Goal-directed locomotion is shared by the CLI and viewer and calibrates
parameter-to-achieved-turn-rate behavior through the runtime graph.

## Validation Specs

- `specs/walk_curvature.pmg_spec`: permissive one-node runtime graph.
- `specs/walk_jog.pmg_spec`: two-node walk/jog cross-transition graph.
- `specs/walk_curvature_selective.pmg_spec`: real-BVH
  GOOD/NEUTRAL/BAD classification.
- `specs/transition_box_shrink.pmg_spec`: real-BVH non-convex target stress
  case that triggers conservative AABB shrink.

Foot locking remains an optional generated-clip diagnostic/post-process. It is
not part of the stored PMG runtime contract.

## Documentation

- [CONTEXT.md](CONTEXT.md) — project vocabulary; code and conversation use
  these terms exactly.
- [docs/DESIGN.md](docs/DESIGN.md) — module structure, contracts, limitations.
- [docs/PAPER_CONFORMANCE.md](docs/PAPER_CONFORMANCE.md) — what matches the
  papers, the prioritized deviation list (D1–D8), and the claim limit.
- [docs/IMPLEMENTATION_PLAN.md](docs/IMPLEMENTATION_PLAN.md) — completed
  paper-core and prioritized remaining work.
- [docs/adr/](docs/adr) — accepted architecture decisions (complete artifact
  seam, k-NN cutoff interpretation, parameter-accuracy calibration).
