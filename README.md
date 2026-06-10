# ParametricMotionGraphs

C++ implementation of **Parametric Motion Graphs (PMG)** (Heck & Gleicher 2007)
using BVH motion clips as the motion source.

```text
BVH clips (native units)
→ skeleton / pose / motion clip
→ parametric motion spaces (blending-based synthesis)
→ point-cloud transition metric + distance grid
→ sampled PMG edges (TGOOD/TBAD, AABB transition regions)
→ runtime graph traversal with point-cloud-aligned C¹ blends
→ OpenGL/ImGui viewer (playback, parametric blend, distance heatmap, graph runtime)
```

**Status:** PMG core scaffold with paper-conformance corrections and Phase-F0
build diagnostics. Phases **A–F0 are implemented, tests 14/14 pass**. This is
not yet a full Heck & Gleicher reproduction: the current `ParametricMotionSpace`
is a minimal inverse-distance / normalized-phase placeholder, not the
Kovar-Gleicher time-registered synthesis system required for high-fidelity real
corpus claims. See
[`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md) for the phase plan and
progress.

## Reason

PMG is not a motion-blending demo. It uses blending-based parametric synthesis
inside graph nodes and transition relations between parameterized motion spaces to
synthesize continuous, controllable motion streams. This codebase is organized for
inspection and iterative, paper-faithful extension rather than maximum abstraction.

`pmg_core` is dependency-free; third-party libraries (GLFW/GLEW/GLM/ImGui) are
confined to the optional viewer target.

## What

Implemented:

- minimal BVH loader (native units; display scaling is render-time only);
- skeleton / pose / quaternion / motion-clip contracts; normalized phase sampling;
- two-way and N-way pose blending; minimal inverse-distance parametric synthesis placeholder;
- Kovar'02 **point-cloud distance** with closed-form 2D rigid alignment (yaw + floor translation);
- **distance grid** + optimal transition point (paper §3.1, Fig 3);
- **sampled edge builder**: random L_s/L_t, TGOOD/TBAD GOOD/BAD/neutral, AABB +
  conservative shrink (Fig 4c), empty-box edge rejection, in-box transition-point average;
- **k-NN edge interpolation** using the paper literal k-th-neighbor cutoff;
- **runtime controller**: per-transition point-cloud alignment recomputed at scheduling time, accumulated world
  transform, C¹ (smoothstep) blend;
- CLI tools (summary, transition inspection, grid dump, threshold calibration, graph spec validation, edge diagnosis, graph build/inspect);
- OpenGL/ImGui viewer (lit floor + shadows, orbit camera, multi-viewport panels,
  distance-grid heatmap, parametric blend, graph runtime);
- `assert`-based executable tests, one per core area.

Not yet implemented:

- the three control applications + CLI flags (Phase G);
- BVH export, skinned mesh, IK/contact correction, CUDA, learned validity (deferred).

## How

Core + tests (canonical):

```bash
cmake -S . -B build -DPMG_BUILD_VIEWER=OFF
cmake --build build
ctest --test-dir build --output-on-failure       # 14/14
```

Viewer (pulls GLFW/GLEW/GLM/ImGui via FetchContent):

```bash
cmake -S . -B build
cmake --build build --config Debug --target pmg_viewer
./build/Debug/pmg_viewer            # Windows: build\Debug\pmg_viewer.exe
```

CLI examples:

```bash
./build/pmg_cli --synthetic
./build/pmg_cli --bvh BVH/SneakLoopA.bvh
./build/pmg_cli --inspect-transition BVH/SneakLoopA.bvh BVH/standStill.bvh
./build/pmg_cli --calibrate-thresholds BVH locomotion_manifest.txt
./build/pmg_cli --validate-graph-spec graph_spec.txt
./build/pmg_cli --diagnose-graph-edge graph_spec.txt walk walk --tgood 0.5 --tbad 0.7
./build/pmg_cli --build-graph graph_spec.txt out.pmg --tgood 0.5 --tbad 0.7
./build/pmg_cli --inspect-graph out.pmg
```

Windows MSVC note: if you hit `error C1090: PDB API ... code '3'` (an `mspdbsrv`
sandbox quirk, not a code error), configure with embedded debug info:
`-DCMAKE_POLICY_DEFAULT_CMP0141=NEW -DCMAKE_MSVC_DEBUG_INFORMATION_FORMAT=Embedded`.

## Directory

```text
ParametricMotionGraphs/
├── CMakeLists.txt
├── README.md
├── locomotion_manifest.txt               # calibration clip-list / action-map
├── BVH/                                   # motion corpus
├── docs/
│   ├── IMPLEMENTATION_PLAN.md             # phase plan A–I + progress
│   ├── DESIGN.md
│   └── PHASE2_NOTES.md
├── apps/
│   ├── pmg_cli.cpp
│   └── viewer/                            # Camera, MeshPrimitives, SkeletonRenderer, ViewerApp, main
├── include/pmg/                           # public headers (units/assumptions documented)
├── src/                                   # pmg_core implementation
└── tests/                                 # 14 assert-based tests (one per core area)
```

## Checklist

- [x] PMG core scaffold: point-cloud metric, distance grid, sampled edges, k-NN interp, runtime.
- [x] Paper-conformance corrections: alignment no longer baked; k-NN uses k-th cutoff; empty edges are diagnosable.
- [x] BVH native units; display scale render-only; thresholds remain corpus-dependent.
- [x] OpenGL/ImGui viewer with distance heatmap + graph runtime.
- [x] CLI transition inspection + threshold calibration.
- [x] `ctest` green (14/14) after the full code-level update.
- [x] F3 — centered blend window (deviation D5 resolved).
- [x] Phase F0 — graph spec validation, edge diagnostics, graph spec build + V2 text save/load.
- [ ] Phase G — control applications; blocked until real-BVH edges are non-empty and diagnostically understood.
- [ ] Time-registration / phase alignment before claiming high-fidelity PMG reproduction on real BVH corpus.
