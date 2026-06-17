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
  -> V8 offline artifact
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

All 37 tests pass on MSVC 2022 Debug (39 with the viewer enabled). See
[docs/REPRODUCTION.md](docs/REPRODUCTION.md) for the full build, validation, and
runtime command set.

The viewer is optional:

```powershell
cmake -S . -B build -DPMG_BUILD_VIEWER=ON
cmake --build build --config Debug --target pmg_viewer
.\build\Debug\pmg_viewer.exe .\outputs\paper_core_walk\artifact.pmg
```

Viewer UI is organized around the runtime model, not the BVH container:

- `Motion Space`: authored parameter samples, local blend weights, canonical
  phase registration, and detected contact intervals.
- `Transition Grid`: source/target phase distance, GOOD/NEUTRAL/BAD regions,
  and selected alignment.
- `PMG Runtime`: graph node/edge selection and the live chain from source
  parameter through reachable target AABB, transition phases, alignment, and
  runtime blend window. Its topology canvas renders each
  `ParametricMotionSpace` as a node and each directed transition as an arrow;
  self-transitions remain visible as loops.
- `Inputs`: BVH selection plus hierarchy/channel diagnostics.
- `Display`: PMG-native-to-viewer scale controls.

Camera, rendering, and input live in the PMG-free `ViewerHost`. PMG playback
and diagnostics are supplied through the compile-time `ViewerWorkspace`
Interface, so another algorithm can provide its own Adapter and `RenderScene`.

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
edge_phase_range walk walk 0.70 0.95 0.05 0.30
edge_config walk walk 225 250 12 60 7
```

`parameter_metric` enables KG04-style parameter accuracy: the build samples
blend weights between adjacent examples, measures each blend's turn rate, and
stores the inversion table so requested parameters achieve their measured
meaning. Generated clips also derive their length from the blended example
durations, so cycle time follows the parameter.

Transition thresholds use Kovar Equation 1's raw weighted squared-sum scale.
They therefore depend on skeleton point count, window size, weights, and native
BVH units; calibrate them when any of those inputs change.

An optional `edge_phase_range <source> <target> <src_start> <src_end>
<tgt_start> <tgt_end>` line restricts the transition search to a per-edge
phase sub-range (paper §6.3); the default forces an advancing end→start
transition (`[0.70, 0.95]→[0.05, 0.30]`).

Multidimensional nodes declare one measured property per parameter axis:

```text
node locomotion 2
parameter_metrics locomotion turn_rate travel_speed
parameter_calibration locomotion 7
```

The calibration samples a deterministic grid over the authored parameter
domain, measures the generated metric vector, normalizes metric-space
distances by sampled range, and stores the full example-weight vector.
`parameter_calibration` exposes deterministic grid density; default is 9
samples per axis.

Build a complete V8 artifact:

```powershell
.\build\Debug\pmg_cli.exe --build-graph `
  .\specs\demo_walk_self_edge_minimal.pmg_spec `
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

V8 stores the Skeleton, registered motion spaces, TimeWarps, multidimensional
parameter calibrations, transition samples, runtime sampling rate, source
paths, seeds, thresholds, edge build reports, and metric-window convention.
V5/V6 scalar calibration tables remain readable and are converted to the
unified in-memory form; V5 also uses scalar transition-phase fallback. V4 lacks
parameter calibrations. V2/V3 remain readable but lack the Skeleton required
for standalone point-cloud alignment.

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

## Curated Demo Specs

- `specs/demo_walk_self_edge_minimal.pmg_spec`: PMG core through one 1-D walk
  node and repeated self-edge streaming.
- `specs/demo_walk_2d_triangle.pmg_spec`: sparse 2-D AABB interpolation over
  three non-collinear samples. This demonstrates sparse 2-D AABB interpolation machinery, not convex-hull support enforcement.
  Requests outside the authored triangle remain valid AABB requests.
- `specs/demo_walk_jog_topology.pmg_spec`: multi-node graph topology and
  cross-gait transitions; jog remains a fixed single-example node.

Validation-only assets stay visible but are not showcase graphs:

- `specs/fixture_edge_selective_good_bad.pmg_spec`: GOOD/NEUTRAL/BAD
  classification.
- `specs/fixture_transition_box_shrink.pmg_spec`: non-convex target stress case
  for conservative AABB shrink.
- `specs/legacy_walk_curvature.pmg_spec`: three-anchor regression/audit fixture
  with non-monotone signed turn response.

Foot locking remains an optional generated-clip diagnostic/post-process. It is
not part of the stored PMG runtime contract.

## Documentation

- [docs/README.md](docs/README.md) — reading order for paper and codebase docs.
- [docs/PAPER_GUIDE.md](docs/PAPER_GUIDE.md) — integrated explanation of
  *Motion Graphs*, *Parametric Motion Graphs*, and repository scope.
- [docs/PAPER_CODE_MAP.md](docs/PAPER_CODE_MAP.md) — paper sections and
  equations mapped to implementation and tests.
- [docs/PAPER_CONFORMANCE.md](docs/PAPER_CONFORMANCE.md) — line-by-line paper
  audit, the prioritized deviation list (D1–D8), what's left, and the claim
  limit.
- [docs/SPEC_AUDIT.md](docs/SPEC_AUDIT.md) — semantic classification of every
  `.pmg_spec`, including expected diagnostics and visualization.
- [docs/DESIGN.md](docs/DESIGN.md) — module structure, offline/online pipeline,
  contracts, failure boundaries, and limitations.
- [docs/REPRODUCTION.md](docs/REPRODUCTION.md) — build, validation, runtime,
  deterministic inputs, and output schemas.
- [docs/MOTION_CORPUS.md](docs/MOTION_CORPUS.md) — BVH skeleton-compatibility
  groups and the minimal 2-D calibration fixture.
- [CONTEXT.md](CONTEXT.md) — project vocabulary and paper-symbol map; code and
  conversation use these terms exactly.
- [docs/adr/](docs/adr) — accepted architecture decisions (complete artifact
  seam, k-NN cutoff interpretation, parameter-accuracy calibration).
