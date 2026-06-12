# Reproduction Guide

## Purpose

Build, test, inspect, and run the repository with explicit inputs and outputs.
Commands assume PowerShell at the repository root.

This guide reproduces repository behavior on the included BVH corpus. It does
not reproduce the original papers' unavailable datasets or perceptual studies.

## Prerequisites

- CMake 3.20 or newer.
- C++20 compiler.
- PowerShell.
- OpenGL development support only when building the viewer.
- Network access during first viewer configuration because CMake FetchContent
  downloads pinned GLFW, GLEW, GLM, and Dear ImGui revisions.

The core library, CLI, and tests have no downloaded runtime dependency.

## Build Core and Tests

```powershell
cmake -S . -B build -DPMG_BUILD_VIEWER=OFF
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

Expected contract:

- configure succeeds;
- `pmg_core`, `pmg_cli`, and test executables build;
- CTest reports no failed tests.

Do not treat passing tests as proof of paper-level animation quality.

## Validate a Graph Spec

```powershell
.\build\Debug\pmg_cli.exe --validate-graph-spec `
  .\specs\walk_curvature.pmg_spec
```

This checks syntax and named node relationships. Full BVH and edge validation
occurs during artifact construction.

## Build the Canonical Walk Artifact

```powershell
.\build\Debug\pmg_cli.exe --build-graph `
  .\specs\walk_curvature.pmg_spec `
  .\outputs\paper_core_walk\artifact.pmg
```

Outputs:

```text
outputs/paper_core_walk/
|-- artifact.pmg
|-- config.json
|-- metrics.json
|-- report.md
`-- tables/
    `-- edge_samples.csv
```

### `config.json`

Contains:

- format version;
- units;
- runtime frame settings;
- source BVH paths;
- node registration settings;
- edge thresholds;
- source/target sample counts;
- seed.

### `metrics.json`

Contains:

- build time and artifact size;
- node and edge counts;
- source report count;
- GOOD/NEUTRAL/BAD sample totals;
- number of shrunken boxes;
- simulated runtime frames and transitions;
- measured runtime throughput;
- optional minimum distance to target `(10, 10)`.

Build time and throughput are machine-dependent. Transition classifications
should be deterministic for identical inputs, compiler floating-point behavior,
and seeds.

### `tables/edge_samples.csv`

One row per sampled source parameter:

```text
edge
source_parameter
good
neutral
bad
min_distance
median_distance
box_shrunk
accepted
reject_reason
```

## Inspect the Artifact

```powershell
.\build\Debug\pmg_cli.exe --inspect-graph `
  .\outputs\paper_core_walk\artifact.pmg
```

The runtime requires a V4+ artifact with a skeleton. Current writes use V6.

## Run Paper Applications

### Random graph walk

```powershell
.\build\Debug\pmg_cli.exe --random-walk `
  .\outputs\paper_core_walk\artifact.pmg `
  --seconds 30 `
  --walk-seed 1234
```

Contract:

- choose only outgoing edges;
- choose target parameters from the target domain;
- clamp through PMG lookup;
- align and blend through the artifact-configured window.

### Goal-directed locomotion

```powershell
.\build\Debug\pmg_cli.exe --goto `
  .\outputs\paper_core_walk\artifact.pmg `
  10 10 `
  --seconds 60 `
  --tolerance 3
```

Optional final facing:

```powershell
.\build\Debug\pmg_cli.exe --goto `
  .\outputs\paper_core_walk\artifact.pmg `
  10 10 `
  --facing-degrees 90 `
  --facing-tolerance-degrees 15
```

Goal-directed locomotion currently requires a one-dimensional node with useful
turn-rate variation and a usable self-transition.

## Run Diagnostic Commands

### Inspect one clip

```powershell
.\build\Debug\pmg_cli.exe --bvh .\BVH\walkCurve.bvh
.\build\Debug\pmg_cli.exe --list-bvh-joints .\BVH\walkCurve.bvh
```

### Inspect contacts

```powershell
.\build\Debug\pmg_cli.exe --inspect-contacts `
  .\BVH\walkCurve.bvh `
  LeftAnkle,RightAnkle
```

### Inspect a clip-pair transition

```powershell
.\build\Debug\pmg_cli.exe --inspect-transition `
  .\BVH\walkCurve.bvh `
  .\BVH\walkMoreCurve.bvh
```

### Export a distance grid

```powershell
.\build\Debug\pmg_cli.exe --dump-distance-grid `
  .\BVH\walkCurve.bvh `
  .\BVH\walkMoreCurve.bvh `
  .\distance_grid.csv
```

### Validate registered motion-space behavior

Registration options are fallback values for nodes without an explicit
`registration` entry. When the spec declares registration, supplied options
must match it so the diagnostic and production artifact use the same
preparation assumptions.

```powershell
.\build\Debug\pmg_cli.exe --space-sweep `
  .\specs\walk_curvature.pmg_spec `
  walk `
  --cycle-joint LeftAnkle `
  --min-contacts 2 `
  --max-foot-slide 0.25 `
  --max-adjacent-step 1.8 `
  --assert-no-regression `
  --dtw-refine `
  --foot-lock
```

### Validate production PMG edge construction

```powershell
.\build\Debug\pmg_cli.exe --validate-graph `
  .\specs\walk_curvature.pmg_spec `
  --cycle-joint LeftAnkle `
  --source-samples 12 `
  --target-samples 60 `
  --seed 7 `
  --tgood 1.5 `
  --tbad 2.0 `
  --min-edge-samples 10 `
  --min-good-fraction 0.9 `
  --assert-no-regression
```

## Validation Specs

| Spec | Purpose |
|---|---|
| `walk_curvature.pmg_spec` | canonical 1-D walk self-transition runtime |
| `walk_jog.pmg_spec` | two-node walk/jog cross-transition graph |
| `walk_curvature_selective.pmg_spec` | selective GOOD/NEUTRAL/BAD classification |
| `transition_box_shrink.pmg_spec` | non-convex target stress case for BAD exclusion |

`transition_box_shrink.pmg_spec` deliberately violates smooth-space assumptions.
Use it only to test conservative AABB shrink behavior.

## Build and Run Viewer

```powershell
cmake -S . -B build -DPMG_BUILD_VIEWER=ON
cmake --build build --config Debug --target pmg_viewer
.\build\Debug\pmg_viewer.exe `
  .\outputs\paper_core_walk\artifact.pmg
```

Manual checks:

1. The artifact's first graph node has a one-dimensional parameter space.
2. Motion Space shows authored samples and parameter weights.
3. Canonical phase and contact markers advance consistently.
4. Transition Grid shows a finite minimum and selected alignment.
5. PMG Runtime shows node/edge topology.
6. During transition, requested parameter, reachable box, actual parameter,
   phase pair, alignment, and blend progress are visible.
7. World root motion has no obvious position or facing discontinuity.

Viewer inspection is not part of headless CTest.

## Determinism

Deterministic fields:

- graph spec;
- source file paths and contents;
- sampling seed;
- sample counts;
- thresholds;
- registration settings;
- frame rate;
- transition window and phase ranges.

Random PMG edge sampling uses `PmgBuilderConfig::seed`. Random runtime walk uses
the separate `--walk-seed`.

Machine-dependent fields:

- wall-clock build time;
- measured throughput;
- small floating-point differences across compilers/platforms.

## Failure Interpretation

- **Spec parse failure**: malformed keyword, missing value, unknown node, invalid
  dimension, or invalid metric declaration.
- **Skeleton failure**: hierarchy/channel/offset mismatch between examples.
- **Registration failure**: missing contacts or inconsistent contact structure.
- **Edge rejection**: at least one sampled source lacks a valid target region.
- **Runtime config failure**: artifact edges disagree on transition window.
- **Goal-control failure**: node is not one-dimensional or has insufficient
  achieved turn-rate range.

Do not increase thresholds before inspecting the edge report. A higher threshold
trades transition quality for connectivity.

## Reproducibility Checklist

- [ ] Record commit and dirty-worktree state.
- [ ] Record compiler and CMake versions.
- [ ] Keep the exact graph spec.
- [ ] Keep source BVH files unchanged.
- [ ] Keep `config.json`, `metrics.json`, report, CSV, and artifact together.
- [ ] Record runtime command and `--walk-seed` when applicable.
- [ ] Report viewer checks separately from automated tests.
- [ ] State that results apply to the included corpus and configuration.
