# ParametricMotionGraphs

Course term project implementation of a BVH-based Parametric Motion Graph viewer/runtime.

## Scope

This repository implements a compact PMG-style pipeline:

```
BVH clips
  -> motion-space preparation
  -> transition distance search
  -> PMG artifact construction
  -> runtime playback
  -> optional OpenGL viewer
```
Currently, the stable demo uses a triangulated 2D scattered support over five authored locomotion anchors.
It can be run from `specs/demo_walk_2d_triangulated.pmg_spec` using a single self-edge.

## Architecture notes

- `PmgViewerWorkspace` owns the ImGui/OpenGL adapter state for the viewer.
- The viewer's Graph/Coverage tab draws the authored parameter samples and, for
  the sparse 2-D demo, the triangulated support formed by the clips. Red
  corner rings mark missing domain corners such as tight-turn jog.
- The Graph/Runtime transition panel reports requested vs actual target
  parameters. Requested is the live runtime target; actual is projected into
  the authored support and the current edge's reachable transition box. The UI
  also distinguishes `REQUESTED RAW` and `REQUESTED PROJECTED`.
- Graph runtime installation goes through shared private lifecycle helpers so
  loaded artifacts and sandbox graphs reset controller, steering, target, and
  trace state consistently.
- `RuntimeController` and `GoalDirectedLocomotion` remain the core PMG runtime
  modules; the viewer adapts their state into `RenderScene` diagnostics.

## Build
```
cmake -S . -B build -DPMG_BUILD_VIEWER=ON -DPMG_BUILD_TESTS=ON
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```
### Build the demo graph
```
.\build\Debug\pmg_cli.exe --validate-graph-spec `
  .\specs\demo_walk_2d_triangulated.pmg_spec

.\build\Debug\pmg_cli.exe --build-graph `
  .\specs\demo_walk_2d_triangulated.pmg_spec `
  .\build\viewer_out\walk_2d_triangulated.pmg
```
### Run viewer
```
.\build\Debug\pmg_viewer.exe .\build\viewer_out\walk_2d_triangulated.pmg
```
---
