# ParametricMotionGraphs — Viewer-only distribution

Minimal viewer-only build of the BVH-based Parametric Motion Graph (PMG)
system: the `pmg_core` library plus the real-time OpenGL viewer. The CLI,
test suite, experiments, audit docs, and paper source are removed on this
branch to keep the tree small — they live on the `dev/core` development
branch.

This is a PMG-core **demo**, not a paper reproduction. It implements the core
PMG pipeline on a single-family, single-node walking motion space: sampled
transition-region construction, a Kovar-style transition metric, runtime
lookup, cyclic-window cleanup, and target-directed steering.

## Concept overview

![PMG concept scenes: pipeline, a node as a parameterized motion space, and edge
sampling](docs/figures/concept_scenes.gif)

Three explanatory scenes (intuition only — quantitative evidence lives on the
full-project branches): the PMG **pipeline** (motion examples → motion-space
node → quality-gated transition graph → generated stream), a **node** as a
parameterized motion space with a limited convex support region (`turn_rate`
primary, `travel_speed` a single-example secondary axis), and **edge sampling**
(a source × target candidate matrix scored by a Kovar-style distance, accepted
on enough good samples).

## Build and run

```powershell
cmake -S . -B build              # PMG_BUILD_VIEWER defaults ON here
cmake --build build --config Release --target pmg_viewer
.\build\Release\pmg_viewer.exe specs\demo_walk_2d.pmg_spec
```

First configure fetches GLFW / GLEW / GLM / ImGui via CMake `FetchContent`
(network required once; cached in `build/_deps` afterwards).

In the viewer: set `Mode = Parametric blend` and `Foot-lock (IK) = ON`, then
drive the curvature / speed sliders. `Param-sweep paths = ON` with a top-down
camera shows the parameter→trajectory fan.

## What's included

| Area | Modules | Role |
| --- | --- | --- |
| Domain data | `ParametricMotionSpace`, `ParametricMotionGraph` | motion spaces, nodes, edges, samples |
| Offline algorithm | `MotionSpacePreparation`, `PmgBuilder`, `PmgOfflinePipeline` | registration, transition search, artifact build |
| Runtime algorithm | `RuntimeController`, `GoalDirectedLocomotion`, `AlignmentStrategy` | phase-gated transitions, alignment, pose stream |
| Input/persistence | `GraphSpec`, `GraphIo` | spec adapter; V2–V13 artifact compatibility |
| Viewer | `ViewerHost`, `PmgViewerWorkspace`, `ViewerRuntimeModule` | render host, PMG UI, runtime adapter |

Headers in `include/pmg/`, implementations in `src/`, viewer in `apps/viewer/`.

## Demo space

`specs/demo_walk_2d.pmg_spec` defines one 2-D locomotion space (axis 0 =
turn_rate, axis 1 = travel_speed) over four authored walking anchors
(`walkMoreCurve`, `walkCurve`, `walkTightCurve`, `walkStraightTwiceAsFast`),
triangulated support, one self-edge. Cross-family jog is intentionally
excluded. Missing domain corners are projected into authored support, not
synthesized.

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
