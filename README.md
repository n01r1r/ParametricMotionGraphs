# ParametricMotionGraphs

C++ implementation of **Parametric Motion Graphs (PMG)** (Heck & Gleicher 2007)
using BVH motion clips as the motion source.

```text
BVH clips (native units)
→ skeleton / pose / motion clip
→ parametric motion spaces (blending-based synthesis)
→ point-cloud transition metric + distance grid
→ sampled PMG edges (TGOOD/TBAD, AABB transition regions)
→ runtime graph traversal with point-cloud-aligned linear blends
→ OpenGL/ImGui viewer (playback, parametric blend, distance heatmap, graph runtime)
```

**Status:** faithful core (paper §3–§4) implemented and paper-aligned — phases
**A–E done, tests 9/9**. Viewer renders a live graph runtime. See
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
- two-way and N-way pose blending; inverse-distance parametric synthesis;
- Kovar'02 **point-cloud distance** with closed-form 2D rigid alignment (yaw + floor translation);
- **distance grid** + optimal transition point (paper §3.1, Fig 3);
- **sampled edge builder**: random L_s/L_t, TGOOD/TBAD GOOD/BAD/neutral, AABB +
  conservative shrink (Fig 4c), empty-box edge rejection, in-box transition-point average;
- **k-NN edge interpolation** (Allen'02 weights, k = dim+1; paper §4 eqs 1–3);
- **runtime controller**: per-transition point-cloud alignment, accumulated world
  transform, C¹ (smoothstep) blend;
- CLI tools (summary, transition inspection, grid dump, threshold calibration);
- OpenGL/ImGui viewer (lit floor + shadows, orbit camera, multi-viewport panels,
  distance-grid heatmap, parametric blend, graph runtime);
- `assert`-based executable tests, one per core area.

Not yet implemented:

- graph spec build + `<50KB` text save/load (Phase F);
- the three control applications + CLI flags (Phase G);
- BVH export, skinned mesh, IK/contact correction, CUDA, learned validity (deferred).

## How

Core + tests (canonical):

```bash
cmake -S . -B build -DPMG_BUILD_VIEWER=OFF
cmake --build build
ctest --test-dir build --output-on-failure       # 9/9
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
└── tests/                                 # 9 assert-based tests (one per core area)
```

## Checklist

- [x] Faithful core: point-cloud metric, distance grid, sampled edges, k-NN interp, runtime.
- [x] Paper-faithfulness audit done; D1/D2/D3 fixed, D4/D6 documented.
- [x] BVH native units; display scale render-only; paper thresholds (0.5/0.7) usable.
- [x] OpenGL/ImGui viewer with distance heatmap + graph runtime.
- [x] CLI transition inspection + threshold calibration.
- [x] `ctest` green (9/9) after each phase.
- [x] F3 — centered blend window (deviation D5 resolved).
- [ ] Phase F — graph spec build + text save/load.
- [ ] Phase G — control applications.
- [ ] Real-corpus tuning by the user with project data.
