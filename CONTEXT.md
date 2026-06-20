# Domain and code context

## Why

Repository implements offline construction and runtime evaluation of a
parametric motion graph. Paper terms are used where contracts match; optional
engineering extensions remain labeled.

## Domain model

- Node owns one `ParametricMotionSpace`.
- Directed `PmgEdge` stores `TransitionSample` values: source parameter,
  reachable target box, source/target normalized phases, distance.
- Registration synchronizes examples within a motion space.
- Alignment places target clip against source clip at runtime transition.

Canonical terms: [docs/domain-glossary.md](docs/domain-glossary.md).

## Code path

1. `src/GraphSpec.cpp`: parse spec, validate structure, resolve BVH paths.
2. `src/PmgOfflinePipeline.cpp`: prepare spaces, build nodes/edges, return artifact.
3. `src/MotionSpacePreparation.cpp`: load/register examples.
4. `src/PmgBuilder.cpp`: sample and classify transitions.
5. `src/GraphIo.cpp`: read V2-V12, write V12.
6. `src/RuntimeController.cpp`: project request, select edge, wait for phase gate,
   align transition, emit world-space poses.
7. `apps/viewer/ViewerRuntimeModule.cpp`: adapt controller lifecycle/status/trace
   for `PmgViewerWorkspace`; no separate transition algorithm.

Public contracts are matching headers under `include/pmg/`. CLI entry and
dispatch live in `apps/PmgCommands.cpp`; command families live beside it.

## Change paths

- Paper-compatible algorithm change: paper mapping -> domain header/source ->
  focused test -> CLI/viewer adapter only if contract changed.
- Optional extension: explicit enum/config/spec field; preserve current default;
  report choice in artifact metadata/diagnostics.
- File format change: `GraphIo` plus roundtrip/legacy tests; do not leak version
  branching into domain/runtime.
- Viewer-only change: `PmgViewerWorkspace` or `ViewerRuntimeModule`; keep
  `RuntimeController` UI-independent.

ADR: [docs/adr/adr-experimental-branch-status.md](docs/adr/adr-experimental-branch-status.md).
Paper symbol map:
[docs/paper-concept-to-code-symbol.md](docs/paper-concept-to-code-symbol.md).

## Current limits

Sparse demo support requires projection. Edge construction is sampling-based
and may reject edges. Offline preparation still knows BVH paths. Artifact writes
are V12-only. Viewer is optional and OpenGL-dependent.

Current animation-quality evidence is report-only: accepted transitions still
include visible-pop cases, and contact mismatch is common. The 5-frame blend
window remains selected over 8 frames because the longer window adds latency,
reduces accepted reachability, and does not remove the measured artifacts. See
`PHASE.md` and `build/blend_window_comparison.md`.

