# ParametricMotionGraphs

BVH-based Parametric Motion Graph offline builder, runtime, CLI, and optional
OpenGL viewer.

This project implements the core PMG mechanism on a sparse, hand-authored,
single-node walking motion space: offline sampled transition construction, V12
artifact persistence, phase-gated runtime lookup, conservative reachable-box
intersection, direct steering, and single-node path following.

## Current demo

`specs/demo_walk_2d.pmg_spec` defines one 2-D locomotion space, four
authored single-family (walking-only) anchors, triangulated support, and one
self-edge. The cross-family jog corner is intentionally excluded (kept as a
stress fixture); missing domain corners are unsupported; runtime requests are
projected into authored support and the selected transition's reachable target
box.

## Module map

| Area | Modules | Role |
| --- | --- | --- |
| Domain data | `ParametricMotionSpace`, `ParametricMotionGraph`, `TransitionTypes` | motion spaces, nodes, edges, samples |
| Offline algorithm | `MotionSpacePreparation`, `PmgBuilder`, `PmgOfflinePipeline` | registration, transition search, artifact construction |
| Runtime algorithm | `RuntimeController`, `GoalDirectedLocomotion`, `AlignmentStrategy` | phase-gated transitions, alignment, pose stream |
| Input/persistence | `GraphSpec`, `GraphIo` | spec adapter; V2-V12 artifact compatibility |
| CLI | `apps/Pmg*Commands.cpp` | validate, build, inspect, diagnose, run |
| Viewer | `ViewerHost`, `PmgViewerWorkspace`, `ViewerRuntimeModule` | platform/render host, PMG UI, runtime adapter |

Headers live in `include/pmg/`; implementations in `src/`; contract tests in
`tests/`. Viewer-only tests also live in `apps/viewer/tests/`.

## Paper concept -> implementation

`parametric motion space` -> `ParametricMotionSpace`; `sampled transition` ->
`TransitionSample`; target bounding box lookup -> `PmgEdge::LookupInterpolated`;
offline construction -> `BuildPmgOfflinePipeline`; runtime stream ->
`RuntimeController::Update` / `CurrentPose`. Full map:
[docs/paper-concept-to-code-symbol.md](docs/paper-concept-to-code-symbol.md).

## Build and runtime flow

```text
.pmg_spec -> LoadGraphSpec -> BuildPmgOfflinePipeline -> BuiltPmgArtifact
          -> GraphIo V12 file -> RuntimeController -> world-space Pose stream
                                      ^
                         CLI direct / ViewerRuntimeModule adapter
```

```powershell
cmake -S . -B build -DPMG_BUILD_VIEWER=ON -DPMG_BUILD_TESTS=ON
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
.\build\Debug\pmg_cli.exe --validate-graph-spec .\specs\demo_walk_2d.pmg_spec
.\build\Debug\pmg_cli.exe --build-graph .\specs\demo_walk_2d.pmg_spec .\build\viewer_out\walk_2d.pmg
.\build\Debug\pmg_viewer.exe .\build\viewer_out\walk_2d.pmg
```

## Paper-compatible vs extension path

Paper-compatible defaults use directional Kovar point-cloud transition distance
and the offline/runtime contracts above. Extensions are explicit config/spec
choices; `dynamics_window` is currently opt-in. Do not treat an extension result
as paper-equivalent without reporting its config.

## Known limitations

- Demo support is sparse; missing 2-D corners are projected, not synthesized.
- Current transition montage reports visible pops, and contact audit reports
  common contact mismatch. The reachable-box intersection gate prevents the
  measured accepted-BAD overreach case; visual/contact diagnostics remain
  report-only.
- Five-frame blending remains the demo default. Eight frames increased latency,
  rejected more requests, and did not improve worst transition artifacts.
- Builder may reject declared edges; all rejected edges make build fail.
- Offline preparation still opens BVH filesystem paths.
- Reader compatibility covers V2-V12; writer emits V12 only.
- Viewer requires optional OpenGL dependencies enabled by `PMG_BUILD_VIEWER`.

## Completion path

Core paper mechanisms, direct steering, and single-node waypoint path following
are implemented. A reproducible corpus audit found only one clear cyclic run
candidate, so the required compatible second node and therefore inter-node
edges and graph-walk demo remain blocked by source data. A 2026-06-22 recut
follow-up kept that gate closed: `jogCurve_recut_000_024.bvh` is only a single
barely viable cyclic run sample, while `walkToJog_recut_005_034.bvh` still
shows seam contact mismatch and remains transitional. `PHASE.md` records the
evidence and deferrals. Exact paper corpus, timing, graph-size, and visual-result
reproduction is not claimed.

Start code exploration with [CONTEXT.md](CONTEXT.md). Supported specs:
[specs/README.md](specs/README.md). Audit order and current decisions:
[PHASE.md](PHASE.md).

External CMU BVH gate: `pmg_cli audit-corpus --corpus-root /path/to/cmubvh --out outputs/cmu_audit`.
See [docs/cmu_bvh_gap_closure.md](docs/cmu_bvh_gap_closure.md).
