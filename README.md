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
Currently, the stable demo can be run from `specs/demo_walk_2d_triangle.pmg_spec` which uses 3 locomotion anchors in 2D parametric motion space, and a single self-edge.
## Build
```
cmake -S . -B build -DPMG_BUILD_VIEWER=ON -DPMG_BUILD_TESTS=ON
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```
### Build the demo graph
```
.\build\Debug\pmg_cli.exe --validate-graph-spec `
  .\specs\demo_walk_2d_triangle.pmg_spec

.\build\Debug\pmg_cli.exe --build-graph `
  .\specs\demo_walk_2d_triangle.pmg_spec `
  .\build\viewer_out\walk_2d_triangle.pmg
```
### Run viewer
```
.\build\Debug\pmg_viewer.exe .\build\viewer_out\walk_2d_triangle.pmg
```
---