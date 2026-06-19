# Viewer Graph Workspace Lifecycle Audit

## Definition

Phase 3.1 identifies lifecycle code in `apps/viewer/PmgViewerWorkspaceGraph.cpp`
that can move later into concrete `ViewerGraphWorkspace` code.

No behavior moves in this session.

## Reason

`PmgViewerWorkspaceGraph.cpp` mixes graph lifecycle state with ImGui drawing.
Future refactor should first move ownership and adoption paths, then leave
drawing as a caller of that concrete module.

## What

Move target means "belongs in a concrete graph-workspace module later", not
"move now".

| Responsibility | Current function/member | Current lines | Move target |
| --- | --- | ---: | --- |
| runtime install input | `DesiredParameterForNode(int)` | 219 | `ViewerGraphWorkspace` query/helper |
| save/reload reset | `ResetGraphRuntimeSession(bool)` | 231 | lifecycle reset/clear |
| selection reset | `ResetGraphRuntimeSelection()` | 244 | lifecycle selection/runtime-view reset |
| graph installation | `StartGraphRuntimeController(...)` | 254 | runtime install path |
| artifact-derived diagnostics | `RebuildRootCanonicalizationMarkers()` | 273 | artifact adoption side effect or later diagnostic hook |
| artifact adoption | `AdoptArtifact(...)` | 299 | atomic adoption path |
| reload | `LoadGraphArtifact(...)` | 371 | load then shared adoption |
| spec build | `BuildArtifactFromSpec(...)` | 377 | spec build then shared adoption |
| save/reload | `SaveArtifact(...)` | 390 | save retained source artifact |
| lifecycle status label | `GraphOriginLabel() const` | 413 | status query |
| lifecycle status label | `GraphPersistenceLabel() const` | 429 | status query |
| graph build | `BuildGraphRuntime()` | 443 | sandbox graph build/install entry |
| runtime install input | `InstallSandboxGraph(...)` | 487 | sandbox install path |
| graph build | `AddAuthoredNode()` | 514 | authoring model mutation remains caller-side unless later scope expands |
| graph build | `AddAuthoredEdge(...)` | 541 | authoring model mutation remains caller-side unless later scope expands |
| graph build | `BuildAuthoredGraph()` | 579 | authored graph build/install entry |

## How

Session 3.2 should introduce concrete files:

- `apps/viewer/ViewerGraphWorkspace.h`
- `apps/viewer/ViewerGraphWorkspace.cpp`

Suggested first move:

1. Move `source_artifact_`, live graph origin/status, `runtime_`, runtime
   config/input, graph skeleton/fps ownership only if callers can stay thin.
2. Route load/build/adopt/save through one adoption path.
3. Keep ImGui functions in `PmgViewerWorkspaceGraph.cpp`; they call concrete
   methods and render returned status/snapshots.
4. Keep PMG edge algorithm in `pmg::PmgBuilder`, serialization in `GraphIo`.

Do not introduce an abstract interface.

## Checklist

- [x] Artifact adoption move target identified.
- [x] Spec build move target identified.
- [x] Graph installation move target identified.
- [x] Save/reload move target identified.
- [x] Runtime install input move target identified.
- [x] Selection reset move target identified.
- [x] Failure/status message move target identified.
- [x] No behavior change.
