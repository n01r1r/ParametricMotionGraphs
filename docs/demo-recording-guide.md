# PMG Faithful Demo Video — Recording Guide

Ref: *Parametric Motion Graphs* (Heck & Gleicher) + its result video.
Companion: `docs/audits/paper-faithfulness-20260622.md` (what is FAITHFUL vs
APPROXIMATION vs EXTENSION). Every shot below is labeled so captions stay honest.

Scope decided 2026-06-27: **both** a walk_2d spine and a CMU multi-node stretch.
Recording: **external capture (OBS)**, no in-app recorder.

## Prereqs

- Build (MSVC): `cmake --build build --config Release` → `build/Release/pmg_viewer.exe`, `pmg_cli.exe`.
- Launch viewer on an asset (first arg = spec / `.pmg` artifact / BVH):
  - `build/Release/pmg_viewer.exe specs/demo_walk_2d.pmg_spec`
  - `build/Release/pmg_viewer.exe experiments/cmu/out/cmu_gait_graph.pmg`
- OBS: 1080p60, **Window Capture** on the viewer window, hide desktop/taskbar.
  Add a lower-third text source per take for the honesty caption.

## Viewer controls (confirmed)

- Mode radios: **Clip playback** / **Parametric blend** / **Graph runtime**.
- **Foot-lock (IK)** checkbox — keep ON for every shot (post-process, honest).
- **Speed** + **Phase** sliders; per-axis parameter sliders in Parametric blend.
- **Param-sweep paths** checkbox — draws the trajectory fan.
- Graph runtime: click in the floor to place a walk target (goto).

---

## Take order

Record the two **READY** takes first (zero code needed). The two **BLOCKED**
takes wait on the viewer foot-lock bake (moving character skates — measured
−65%/−83%, see `docs/audits/runtime-foot-skate-20260627.md`).

### T1 — Parametric control (walk_2d) · READY

- **Shows:** continuous control of one parametric motion space — vary curvature
  and speed, motion changes smoothly with clean feet. Paper §4–§5.
- **Steps:** launch on `demo_walk_2d.pmg_spec` → **Parametric blend** → **Foot-lock (IK)** ON →
  slowly drag the curvature slider min→max, then speed slider. Hold ~2 s at extremes.
- **Label:** FAITHFUL (parametric space + blend synthesis); foot-lock is an
  honest EXTENSION (Kovar foot-skate cleanup on the output).

### T2 — Parameter→trajectory fan (walk_2d) · READY

- **Shows:** how the parameter maps to ground trajectory.
- **Steps:** same load → **Param-sweep paths** ON → orbit camera to top-down.
- **Label:** FAITHFUL (parameter-space representation, §4).

### T3 — Goal-directed target reach (walk_2d) · BLOCKED on viewer foot-lock

- **Shows:** click a target, character walks to it (semantic control → curvature
  parameter). Paper §4 (k-NN lookup + goal-directed controller).
- **Steps:** **Graph runtime** → **Foot-lock (IK)** ON → click target → **Play**.
- **CLI evidence (now):** `pmg_cli --goto specs/demo_walk_2d.pmg_spec 20 0 --seconds 30 --arrival-distance 18 --tolerance 2`
  → `reached=1`, `pop_ratio≈1.9`. Tune `--arrival-distance` per target.
- **Label:** lookup FAITHFUL; controller policy + arrival ease APPROXIMATION.
- **Blocker:** streamed feet skate until the viewer bake-and-lock lands.

### T4 — CMU multi-node traversal (subj 16) · BLOCKED on viewer foot-lock + scripting

- **Shows:** traversal across parametric nodes (walk ↔ run) — the graph itself.
  Paper §3.2 + §4.
- **Asset:** `experiments/cmu/out/cmu_gait_graph.pmg` (already built; gate passes).
  Contact joints `LeftFoot,RightFoot`.
- **Steps:** **Graph runtime** → script a deliberate walk→run→turn (do NOT random-walk;
  it churns ~1.8 transitions/s). **Foot-lock (IK)** ON.
- **CLI sanity (now):** `pmg_cli --random-walk experiments/cmu/out/cmu_gait_graph.pmg --seconds 30 --contact-joints LeftFoot,RightFoot`
  → 55 transitions, `pop_ratio≈1.78`.
- **Label:** edge build + traversal FAITHFUL; **honest caption: cross-gait pop is
  a known limitation** — CMU density graph won on coverage, not smoothness
  (`docs/audits/...`, memory `pmg-graph-build-findings`). Single-skeleton subj 16
  is locomotion only.
- **Blocker:** streamed feet skate; plus needs a scripted transition sequence.

---

## Remaining engineering before T3/T4

**Viewer bake-and-lock playback** (the one gap left for moving-character shots):
when a goto/run is set up, capture the controller's world-pose stream into a
`MotionClip`, apply `LockFootContacts`, and replay it (the viewer already renders
`Pose` and already wires the lock for the preview). Chosen over a `MotionClip`→BVH
export because `Pose` is quaternion and re-rendering avoids an euler round-trip —
render-faithful. True causal online lock is unnecessary for a recorded,
deterministic demo.
