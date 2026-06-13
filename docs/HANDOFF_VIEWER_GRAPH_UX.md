# Handoff: Viewer Graph Build/Manipulation UX

Last updated: 2026-06-13. Branch: `codex/reduce-viewer-text`.

Scope: `apps/viewer/` only. No `pmg_core` changes, so the 36/36 core test
result in [`STATUS.md`](STATUS.md) is unaffected.

## What changed

This branch carries two related passes over the viewer's ImGui control window.

### 1. Text reduction (original branch purpose)

Trimmed verbose `TextWrapped`/`TextDisabled` blurbs and duplicate diagnostics
across Workflow, Transport, Inputs, Display, Motion Space, Transition Grid, and
the graph/transition panels. Removed the unused `ModeName` helper (no remaining
caller) and the redundant camera-target readout in `ViewerHost`. No control flow
changed: every deleted line was explanatory text or a diagnostic still shown
elsewhere (mode via the Workflow radios; frame/fps in Inputs; keybinds in
`HandleShortcuts`).

### 2. Graph build/manipulation UX affordances

Implemented in council-priority order. Reconciled with the terseness pass by
expressing guidance as structural controls and one-line hints, not prose.

| Improvement | Location |
| --- | --- |
| TGOOD/TBAD sliders placed at the single-node build button (the build reads them; they were previously only on the Transition Grid tab). Shared `tgood_`/`tbad_` members, disambiguated widget IDs `##build`. | `PmgViewerWorkspaceGraph.cpp` `BuildGraphSection` |
| Build-path source tags: spec = "per-edge config, saveable"; single-node = "scratch, not saveable". | same |
| One-line next-step hint (clip -> samples -> graph), terse successor to the removed multi-line step strip. | `PmgViewerWorkspace.cpp` `BuildWorkflowSection` |
| Per-sample edit/delete list: inline parameter `InputFloat` (commit on Enter) + `x` remove. Erase deferred to after the loop; `RebuildPmgSpace` follows. | `PmgViewerWorkspace.cpp` `BuildMotionSpaceSection` |
| Pre-build validity hint: heatmap minimum cell vs TGOOD -> "transitions likely" / "likely empty; raise TGOOD". Hint only (heatmap source is the loaded clip, not the space). | `PmgViewerWorkspaceGraph.cpp` `BuildGraphSection` |
| Graph canvas: node hover-highlight and drag-to-reposition. View-only; the graph is never mutated. | `PmgViewerWorkspaceGraph.cpp` `DrawGraphCanvas` |

New header state (`PmgViewerWorkspace.h`): `std::vector<glm::vec2>
graph_node_offsets_` (per-node drag offsets on top of the auto-layout) and
`int graph_drag_node_`. Offsets are resized on demand to the node count inside
`DrawGraphCanvas` and cleared in `AdoptArtifact` and `BuildGraphRuntime` so a new
graph never inherits a prior graph's positions.

## Build and verify

Viewer requires network FetchContent on a cold configure (glfw/imgui/glew/glm).
If `build/` is already configured (it is, in this workspace), it builds offline:

```powershell
cmake --build build --target pmg_viewer --config Debug
```

Result: both edited translation units compile clean, `pmg_viewer.exe` relinks,
no warnings. There are no unit tests over the ImGui UI; compilation is the gate.

## Review record

Four independent reviewers (one `cavecrew-reviewer` + three finder angles via
`/code-review high`). One real bug surfaced and was fixed before this writeup:
stale drag offsets survived a graph swap with an equal node count, because the
on-demand resize only fires on a count change. Fixed by clearing
`graph_node_offsets_`/`graph_drag_node_` at both graph-(re)build entry points.
Confirmed safe: the `pmg_examples_` erase (deferred, single consumer per tab),
the shared TGOOD/TBAD members, and all ImGui IDs.

A minor, intentional interaction remains: starting a node drag with a
double-click also sets the runtime target (mouse delta is ~0, so no real
displacement). Left as-is.

## Deferred / next

In-GUI multi-node authoring (add node, add edge, edit edge thresholds) is
**Path B**, still deferred. The current canvas is inspection + view manipulation
+ runtime steering only; multi-node graphs are authored via `.pmg_spec` files
and built through the same core path as the CLI `--build-graph`.
