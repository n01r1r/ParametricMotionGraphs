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

## UI cleanup pass (2026-06-24)

Trim and clarify the live UI surface without touching the seam. Code landed on
`dev/ui` (`8755805` structure, `c608f9f` polish); this plan doc on `dev/misc`.
Viewer ctest 5/5.

### Done

- Deleted dead builders `BuildWorkflowSection` / `BuildTransportSection` /
  `BuildDisplaySection` (~125 lines, zero call sites after the `ImGuiViewerUi`
  migration).
- Rewired path preview into the live transport. The ghost-skeleton renderer was
  alive but its only toggle lived in the dead transport builder, so the feature
  was unreachable; it now emits the existing `SetPathPreviewEnabled/Count`.
- Dropped `SelectClip` command: exact duplicate of `LoadClip`, never emitted.
- Renamed the `PMG Runtime` tab to `Graph`: fixes the parent/child `Runtime`
  name collision and matches the existing "see the Graph tab" help text.
- Merged the three Graph build subtabs (Author / From spec / Quick self-edge)
  into one `Build` tab with an Author/Spec/Quick source radio. 9 -> 7 tabs.
- Uniform native `SeparatorText` section headers (Mode / Transport / Display on
  the main window; Inputs; Motion Space; Build Source).

### Deliberately skipped (rationale)

| Item | Why skipped | Revisit when |
|---|---|---|
| Split `ViewerUiState` into per-panel sub-structs | It is a flat copy-snapshot; reads fine. Split = churn across `MakeUiState()` and every read site for no behavior gain. | A data set keeps moving together across several panels. |
| Split `ApplyUiCommand` flat switch into helpers | The ~110-line one-liner dispatch reads as a table; helpers add jump cost. | The switch outgrows a screen or a case gains real logic. |
| Reduce to one accent color | Heatmap, phase-timeline, and green/amber/red status colors are semantic encodings, not chrome. | Never for data-viz; only if a non-semantic chrome accent appears. |
| Normalize input-field / button widths | Layout-regression risk, no render tests to catch breakage, low readability gain. | A width inconsistency actually misreads in use. |
| Finish command-seam migration | ~14 `ViewerUiCommandType` values still have no emitter (legacy tabs call workspace methods directly). Intentional incremental-migration debt, not breakage. | Next seam-migration phase (see "Remaining Dear ImGui references"). |

### Possible extensions / potentials

- `SeparatorText` swap in the denser legacy Graph-build / Transition-Grid
  dividers -- same one-line change, left out to keep commits tight.
- Tab-merge follow-through: if Build sources still feel redundant in use,
  collapse Author/Spec/Quick further, or fold Coverage into Runtime.
- Migrate Transition-Grid + Graph tabs behind the state/command seam (the
  "next migration target" already named above).
- Restart-graph button and next-step hint were dropped with the dead transport
  builder; recoverable from history at `e4dc753`. Re-add to the live UI only if
  users ask.
- Stale repro step `docs/audits/baseline-20260618.md:168` ("PMG Runtime -> From
  spec") -- left as a dated historical snapshot; sync only if that audit is
  re-run.

## Rendering pass (2026-06-25)

Scene-rendering / readability features on `dev/ui`, no seam change. Viewer ctest
5/5; full ctest 50/50.

### Done

- **Translucent diagnostic-line infra** (`96f7703`). Added `DiagnosticLine::alpha`
  + a `uAlpha` uniform and a blended diagnostic-line pass (opaque geometry stays
  `alpha=1`); used by the ghost trail and sweep overlay below. Originally shipped
  with a `ColorEdit3` floor-color picker, **removed in the follow-up** (the
  per-motion origin marker covers the actual need; the floor stays a constant).
- **Origin marker** (follow-up). A distinct magenta disc + vertical pole at the
  motion's phase-0 root (`AppendOriginMarker`), so the start point is identifiable
  without recoloring the whole floor.
- **Diluted path-preview ghosts** (`8f2251d`, strengthened in follow-up). Path
  preview now defaults on (the toggle existed but off, so the trail looked
  "gone"). After the first faint grey-blue pass read as too washed-out, the trail
  is now a vivid saturated blue at higher per-ghost alpha (`clamp(3/count,
  0.45..0.85)`), lightness ramping dark->bright for the start->end cue.
- **Parameter-sweep root-path overlay** (`627f62f`). New blend-mode diagnostic:
  `RecomputeParamSweepPaths` blends the space at 5 values across the view axis
  and caches each integrated root trajectory; `AppendParamSweepPaths` draws them
  translucent, blue(slow)->red(fast). Toggle "Param-sweep paths", default on.
  Shows how the travel path changes with the blend parameter (the onion-skin
  only showed one parameter at a time).
- **Capsule mannequin skeleton** (`82da47f`). `BoneSegment::radius` gives each
  bone a girth by child-joint role (`MannequinBoneGirth`: trunk / head+proximal
  / extremity), with sphere caps both ends so a limb chain reads as one smooth
  capsule body instead of uniform wire. Girth scales with display + skeleton
  scale.

### Deliberately skipped (rationale)

| Item | Why skipped | Revisit when |
|---|---|---|
| Per-axis / runtime tunables for ghost alpha, sweep count, girth | Constants read fine and tune in one line; no demand for live sliders. | A user actually wants to dial these in-session. |
| N-D param-sweep refresh on sibling-axis drag | Cached on space rebuild only; correct for the 1-D spaces this targets. | An N-D space needs the overlay to follow a held axis. |
| Order-independent transparency for trails | Depth-write + back-to-front-ish cylinder draw reads fine for faint trails. | Translucent overlap artifacts actually misread. |
