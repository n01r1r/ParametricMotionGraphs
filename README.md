# Parametric Motion Graphs — Viewer Demo

Welcome. This is a small, **runnable** demo of a BVH-based **Parametric Motion
Graph (PMG)** system: a real-time OpenGL viewer built on the `pmg_core` library.
Load a motion spec, then drive a character with sliders and a click-to-go target
while the graph blends and stitches motion clips together live.

This branch is the **viewer-only distribution** — the `pmg_core` library plus the
viewer. The CLI, test suite, audit docs, and paper source are kept off this
branch to keep the tree small; they live on the `dev/core` development branch.

> It is a faithful **PMG-core demo**, not a full paper reproduction. It
> implements the core pipeline on single-family and small multi-node locomotion
> spaces: sampled transition-region construction, a Kovar-style transition
> metric, runtime lookup, cyclic-window cleanup, and target-directed steering.

## See the idea in ten seconds

![PMG concept scenes: pipeline, a node as a parameterized motion space, and edge
sampling](docs/figures/concept_scenes.gif)

Three explanatory scenes (intuition only): the PMG **pipeline** (motion examples
→ motion-space node → quality-gated transition graph → generated stream), a
**node** as a parameterized motion space with a limited convex support region
(`turn_rate` primary, `travel_speed` a single-example secondary axis), and
**edge sampling** (a source × target candidate matrix scored by a Kovar-style
distance, accepted on enough good samples).

## How it fits together

![Implementation structure: BVH data flows through the offline build into the
runtime and the viewer](docs/figures/slide11_architecture_centered.png)

| Area | Modules | Role |
| --- | --- | --- |
| Domain data | `ParametricMotionSpace`, `ParametricMotionGraph` | motion spaces, nodes, edges, samples |
| Offline algorithm | `MotionSpacePreparation`, `PmgBuilder`, `PmgOfflinePipeline` | registration, transition search, artifact build |
| Runtime algorithm | `RuntimeController`, `GoalDirectedLocomotion`, `AlignmentStrategy` | phase-gated transitions, alignment, pose stream |
| Input/persistence | `GraphSpec`, `GraphIo` | spec adapter; V2–V13 artifact compatibility |
| Viewer | `ViewerHost`, `PmgViewerWorkspace`, `ViewerRuntimeModule` | render host, PMG UI, runtime adapter |

Headers live in `include/pmg/`, implementations in `src/`, the viewer in
`apps/viewer/`.

## Prerequisites

- **CMake ≥ 3.20** and a **C++20** compiler. Verified with **MSVC 2022** on
  Windows 10/11 (the commands below are PowerShell). Other C++20 toolchains are
  untested; the MinGW GCC bundled with some setups is too old to build.
- A GPU and driver with **OpenGL 3.3 core profile** support (the viewer context).
- **Network access on the first configure** — CMake `FetchContent` downloads the
  windowing / GL / UI libraries, then caches them under `build/_deps`.

## Quick start

```powershell
cmake -S . -B build              # PMG_BUILD_VIEWER defaults ON here
cmake --build build --config Release --target pmg_viewer
.\build\Release\pmg_viewer.exe specs\demo_walk_2d.pmg_spec
```

The first configure fetches GLFW / GLEW / GLM / ImGui via CMake `FetchContent`
(network required once; cached in `build/_deps` afterwards).

Once the window is open:

- Set **Mode = Parametric blend** and **Foot-lock (IK) = ON**, then drag the
  **curvature / speed** sliders to steer the character.
- Turn on **Param-sweep paths** and tilt to a top-down camera to see the
  parameter → trajectory fan.
- **Right-click the floor** to drop a goto target; the character steers toward it.
- **WASD** flies the camera, **left-drag** orbits, **scroll** zooms.

## Dependencies

The viewer's third-party libraries are pulled automatically by CMake
`FetchContent` at configure time — nothing to install by hand. The `pmg_core`
library itself has no external dependencies; only the viewer pulls these:

| Library | Version | Purpose | License |
| --- | --- | --- | --- |
| [GLFW](https://github.com/glfw/glfw) | 3.4 | window, input, GL context | zlib/libpng |
| [GLEW](https://github.com/Perlmint/glew-cmake) | 2.3.1 | OpenGL extension loader | BSD / MIT |
| [GLM](https://github.com/g-truc/glm) | 1.0.1 | vector / matrix math | MIT |
| [Dear ImGui](https://github.com/ocornut/imgui) | 1.92.8-docking | immediate-mode UI (docking) | MIT |

## What the system produces

The bundled CMU gait-graph specs (`cmu_gait_graph`, `cmu78_gait_graph`) build a
small walk/run graph and let the runtime traverse it. The figures below are the
kind of evidence those builds produce (the regeneration scripts live on the
`dev/core` branch, where the CLI ships).

**The transition metric separates good cuts from bad ones.** Same-gait
transitions (walk↔walk, run↔run) cost far less than cross-gait (walk↔run) — note
the log scale and the ~5× gap in medians:

![Transition cost within-gait vs cross-gait for subject 16: median 378 vs 1935
on a log scale](docs/figures/transition_distance_within_vs_cross.png)

**The build gate accepts good edges and rejects bad ones.** Within-gait and
walk→run pairs pass with mostly "good" samples; the awkward run→walk pair has
4 edges rejected:

![Edge acceptance by gait pair for subject 16: walk→walk, run→run and walk→run
pass; run→walk has 4 edges rejected](docs/figures/edge_acceptance_subj16.png)

**The runtime emits one continuous stream.** A random walk over the walk/run
graph produces a single connected root trajectory, switching between nodes
through short transition windows:

![Synthesized root trajectory for subject 78: walk (blue) and run (orange)
segments joined by 104 transition frames](docs/figures/root_trajectory_subj78.png)

## Demo spaces

`specs/demo_walk_2d.pmg_spec` defines one 2-D locomotion space (axis 0 =
turn_rate, axis 1 = travel_speed) over four authored walking anchors
(`walkMoreCurve`, `walkCurve`, `walkTightCurve`, `walkStraightTwiceAsFast`),
triangulated support, one self-edge. Cross-family jog is intentionally excluded.
Missing domain corners are projected into authored support, not synthesized.

Curated CMU mocap demos use clips tracked directly under `BVH/` (subjects 16 and
78, `NN_MM.bvh` = subject_trial; CMU Graphics Lab MoCap Database):
`specs/cmu_walk_1d.pmg_spec` (1-D walking-speed blend), `cmu_walkrun_1d`,
`cmu_gait_graph` (subj16 walk/run), `cmu78_gait_graph` (subj78 walk/run).

## Data provenance

Every BVH clip shipped on this branch comes from one of two sources:

**CMU Graphics Lab Motion Capture Database** (`mocap.cs.cmu.edu`) — the eight
`NN_MM.bvh` clips, where `NN` = subject and `MM` = trial. Subject 16: `16_21`,
`16_31`, `16_36`, `16_45`. Subject 78: `78_10`, `78_27`, `78_29`, `78_30`. The
CMU database is free to use for any purpose.

**"Center" locomotion corpus** — the four `walk_2d` anchors `walkCurve`,
`walkMoreCurve`, `walkTightCurve`, `walkStraightTwiceAsFast`. They share the
`Center`-rooted skeleton (BVH `ROOT Center`) and ship as the project's bundled
demonstration clips (present since the initial commit). These drive the
single-family `demo_walk_2d` parametric walk space.

## Known limitations

- Demo support is sparse; missing 2-D corners are projected, not synthesized.
- Runtime traversal renders a raw pose stream; foot-lock is applied to the
  parametric-blend preview clip, not the live graph-traversal stream.
- Five-frame blending is the demo default.
- Reader compatibility covers V2–V13; writer emits V13 only.
- Viewer requires the OpenGL dependencies pulled by `PMG_BUILD_VIEWER`.

## License

Released under the **MIT License** — see [LICENSE](LICENSE) (© 2026 Dongyeob
Han). The bundled dependencies listed above keep their own (permissive)
licenses, and the CMU Graphics Lab Motion Capture Database clips remain under the
CMU terms (free to use for any purpose).
