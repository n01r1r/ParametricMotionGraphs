# Viewer UI backend plan

## Current problem

`PmgViewerWorkspace` mixes runtime/workspace orchestration with Dear ImGui
widgets. Replacing ImGui therefore touches runtime-facing orchestration.

## New seam

- `ViewerUiState`: copied, UI-only snapshot; no mutable PMG internals.
- `ViewerUiCommand`: explicit requests applied by `PmgViewerWorkspace`.
- `ImGuiViewerUi`: reads state, owns migrated widgets, emits commands.

PMG mutations remain in `PmgViewerWorkspace`. Complex graph, canvas, input,
motion-space, and transition-grid drawing helpers remain there temporarily;
they are migration debt, not a second backend contract.

## Platform lifecycle rule

GLFW, OpenGL, ImGui context/backend initialization, frame lifecycle, render,
and shutdown remain in host code and `main.cpp`. UI adapters own widgets only.

## Remaining Dear ImGui references

| File | Classification | Why still there | Intended migration target |
|---|---|---|---|
| `apps/viewer/main.cpp` | platform lifecycle | Owns ImGui context/backend init, per-frame begin/end, GLFW callback chaining, input capture gates. | Keep in platform host unless backend lifecycle moves behind higher-level shell. |
| `apps/viewer/ImGuiViewerUi.cpp` | widget adapter | Owns migrated workflow/transport/display widgets, reads `ViewerUiState`, emits `ViewerUiCommand`. | Keep as current Dear ImGui adapter. |
| `apps/viewer/PmgViewerWorkspace.cpp` | temporary workspace helper | Legacy tabs, custom plots/canvas helpers, shortcut polling still call ImGui directly. | Incrementally move remaining non-graph widgets into `ImGuiViewerUi`; leave graph/canvas for later phase. |
| `apps/viewer/PmgViewerWorkspaceGraph.cpp` | temporary workspace helper | Graph authoring/runtime tabs and canvases still render directly with ImGui. | Next migration target after seam PR: graph/canvas/input tabs behind state/command seam. |
| `apps/viewer/ViewerHost.cpp` | camera/host UI | Draws camera-only host window (`Follow scene focus`, `Reset camera`). This is temporary UI leakage, not workspace mutation logic. | Keep for now; if host UI grows, move to small host state/command adapter separate from workspace UI seam. |

## Future backends

Manual minimal UI, Nuklear, or RmlUi can consume the same snapshot and emit the
same commands. Add no broad `IViewerUi` until a second backend exists.

## Non-claim

This change does not replace ImGui. It only makes replacement local and starts
moving widgets behind the state/command seam.
