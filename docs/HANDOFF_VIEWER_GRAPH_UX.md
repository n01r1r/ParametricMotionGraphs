# Handoff: Viewer Graph Build/Manipulation UX

Last updated: 2026-06-13. Status: merged into `main` via PR #17 (`c32414f`);
source branch deleted. This file is now a landed-work record, not a live handoff.

Scope: viewer UI and viewer-specific tests. No `pmg_core` behavior changes.

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

## Path B — in-GUI multi-node authoring (landed)

Implemented as a viewer-side authoring model (no `pmg_core` graph API change;
`ParametricMotionGraph` stays append-only). `BuildGraphSection` gained an
"Author multi-node graph (sandbox)" block: **Add node from motion space**
snapshots the current 1-D motion space (+ its skeleton) as `AuthoredNode`;
**Add edge** records a source/target pair with per-edge TGOOD/TBAD seeded from
the sliders; the edge list edits thresholds inline and removes entries. **Build
authored graph** reconstructs the immutable `ParametricMotionGraph` from scratch
(`PmgBuilder::BuildEdge` per edge, per-edge thresholds) and installs it.

To get distinct nodes, re-author the motion space (load other clips in Inputs)
between **Add node** presses. Nodes must share one strictly compatible skeleton
(joint names, hierarchy, offsets, and channel conventions are checked at
build). Empty-box edges are skipped with a status note.

Both the single-node sandbox and the authored build now funnel through one
private `InstallSandboxGraph(built, blend_frames, status)` that owns the
controller/alignment/flag/offset wiring (removed the duplicated tail in
`BuildGraphRuntime`).

Like the single-node sandbox, an authored graph is **not saveable** (no backing
artifact). The saveable route for multi-node graphs remains `.pmg_spec` files
built through the same core path as the CLI `--build-graph`.

## Deferred / next

Saving authored sandbox graphs (synthesize a `BuiltPmgArtifact` from the
authoring model so "Save artifact" works) remains the next flow-level item.

## Follow-up: in-canvas edge authoring

Implemented on branch `codex/viewer-canvas-edge-authoring`:

- authored nodes render on a dedicated canvas before graph build;
- drag a node's gold connector onto a target node to add a directed edge;
- drag a node body to arrange the authoring canvas;
- combo-based edge creation remains as a fallback;
- both paths share `GraphAuthoringModel.h`, which rejects invalid endpoints,
  non-finite/negative thresholds, `TBAD < TGOOD`, and duplicate directed pairs;
- edited invalid thresholds fail explicitly before builder work instead of
  being silently clamped.

`test_viewer_graph_authoring` covers valid, reverse, self, duplicate, invalid
endpoint, and invalid-threshold insertion. Native UI automation was unavailable
in the implementation environment; viewer compilation is the interaction-code
gate.

### UX correction after visual review

The first canvas-authoring pass exposed every build path and both canvases in
one vertical stack. It was functionally complete but visually ambiguous.

The Graph panel now uses four task tabs:

- **Author**: numbered node snapshot -> edge connection -> build flow;
- **From spec**: saveable `.pmg_spec` build only;
- **Quick self-edge**: one-node diagnostic only;
- **Runtime**: live graph canvas and playback controls only.

The header always reports current graph provenance, node/edge count, and
saveability. Successful builds open Runtime automatically. Authoring shows
current clip names and parameter range, auto-suffixes duplicate node names,
hides source/target combo controls under Advanced, and keeps only the authored
canvas visible. Runtime layout reserves label space; self-edge loops bend
sideways instead of clipping above the canvas.

## Follow-up: idle self-edge playback

The graph viewer previously submitted its selected node and parameter on every
playback update. When both already matched the active runtime state, that still
requested the node's self-edge and repeatedly jumped to the edge's target phase.

`dev/ui` commit `b6b49a7` leaves the request empty while the viewer is idle and
the selected state already matches. Idle playback now uses
`RuntimeController`'s continuous cycle folding. Node changes, parameter
changes, and goal steering still submit explicit graph-transition requests.
Core same-node/same-parameter transitions remain intentional and unchanged.

Verification: Debug `pmg_viewer` rebuilt successfully;
`test_runtime_controller` and `test_goal_directed_locomotion` pass (2/2).
