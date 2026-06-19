# Shallow Module Deletion Audit

## Definition

Phase 5.1 identifies shallow modules, pass-through seams, and compatibility
wrappers that may be deleted or inlined in later small sessions.

No code changes are made in this session.

## Reason

The repository has several seams created during cleanup and viewer extraction.
Some are real boundaries with tests or dependency isolation. Others now add a
file, symbol, or friend declaration without enough behavior to justify the
extra surface area.

## What

Verdicts:

- `KEEP_DEEP_MODULE`: owns real behavior or a cohesive domain.
- `KEEP_REAL_ADAPTER_SEAM`: boundary is useful for isolation, tests, or deps.
- `DELETE_SHALLOW_PASS_THROUGH`: remove wrapper/module after callers move.
- `INLINE_FORWARDER`: inline one-line forwarding function at its only caller.
- `DEFER_UNTIL_CALLERS_STABLE`: wait because callers or public behavior are
  still moving.

| Area | Current surface | Verdict | Reason | Later session |
| --- | --- | --- | --- | --- |
| CLI dispatcher | `apps/PmgCommands.cpp`, `apps/PmgCommandModules.h` | `KEEP_REAL_ADAPTER_SEAM` | Dispatcher keeps usage/error handling in one place and prevents one giant CLI file. Module entry points have multiple command families, not one-line behavior. | None |
| CLI BVH commands | `apps/PmgBvhCommands.cpp` | `KEEP_DEEP_MODULE` | Owns BVH loading, joint lookup, contact inspection, recut export, and CLI parsing. | None |
| CLI transition commands | `apps/PmgTransitionCommands.cpp` | `KEEP_DEEP_MODULE` | Owns transition inspection, convention compare, distance grids, threshold calibration. | None |
| CLI graph commands | `apps/PmgGraphCommands.cpp` | `KEEP_DEEP_MODULE` | Owns graph spec validation, build, inspect, edge diagnosis. | None |
| CLI diagnostic commands | `apps/PmgDiagnosticCommands.cpp` | `KEEP_DEEP_MODULE` | Large mixed diagnostic file, but not shallow. Split only by behavior, not deletion. | None |
| CLI runtime commands | `apps/PmgRuntimeCommands.cpp` | `KEEP_DEEP_MODULE` | Owns random-walk and goto app commands with quality gates and output dumps. | None |
| CLI executable forwarder | deleted in 5.2 | `INLINE_FORWARDER` | Program entry now lives in `apps/PmgCommands.cpp`; no separate callable CLI wrapper remains. | Done |
| Legacy frame-count generation | deleted in 5.4 | `DELETE_SHALLOW_PASS_THROUGH` | Diagnostics/tests now use duration-derived `ParametricMotionSpace::GenerateClip`; no legacy fixed-frame public wrapper remains. | Done |
| `ParametricMotionSpace` legacy friend | deleted in 5.4 | `DELETE_SHALLOW_PASS_THROUGH` | Friend existed only for legacy wrapper access. | Done |
| Graph artifact compatibility load | `LoadPmgArtifactText(path).graph` in `GraphIo.cpp` | `DEFER_UNTIL_CALLERS_STABLE` | One-line convenience, but file format compatibility is a real public concern. Avoid shrinking until artifact callers settle. | Later, after GraphIo callers stable |
| Graph compatibility comments | V2-V6 readers, centered/default reinterpretation | `KEEP_DEEP_MODULE` | Not shallow; preserves old artifact semantics. | None |
| Viewer workspace interface | `apps/viewer/ViewerWorkspace.h` | `KEEP_REAL_ADAPTER_SEAM` | `ViewerHost` is algorithm-neutral and testable through `TestWorkspace`; interface isolates render/input host from PMG-heavy workspace. | None |
| Viewer host forwarding methods | `ViewerHost::Update`, `BuildUi`, `HandleGroundClick`, `Render` | `KEEP_REAL_ADAPTER_SEAM` | Forwarding is the host's contract: camera focus, renderer lifecycle, and input dispatch. Tests cover it. | None |
| Viewer workspace factory | deleted in 5.3 | `INLINE_FORWARDER` | `main.cpp` constructs `PmgViewerWorkspace` directly. | Done |
| Viewer runtime module | `apps/viewer/ViewerRuntimeModule.*` | `KEEP_DEEP_MODULE` | Owns runtime controller lifecycle, trace, request preview, and status snapshot. Not a pass-through. | None |
| Viewer graph workspace split target | `PmgViewerWorkspaceGraph.cpp` lifecycle functions | `DEFER_UNTIL_CALLERS_STABLE` | Previous audit identifies behavior to move. Do not delete while graph workspace boundary is still forming. | After Phase 3 graph-workspace refactor |
| One-line GLM/math local helpers | `ToGlm`, `Add`, `Subtract`, scalar canvas helpers | `KEEP_DEEP_MODULE` | Local pure helpers reduce repeated coordinate math in viewer drawing. File-level static helpers do not expand public API. | None |
| Test-only viewer host fake | `tests/test_viewer_host.cpp::TestWorkspace` | `KEEP_REAL_ADAPTER_SEAM` | Test fake is the proof that `ViewerWorkspace` seam works. Do not delete unless interface is deleted. | None |
| Test-only registered blending legacy calls | rewritten in 5.4 | `DELETE_SHALLOW_PASS_THROUGH` | Test now validates `GenerateClip` behavior directly. | Done |
| Domain convenience functions | `MinParameter`, `MaxParameter`, `ExampleParameters`, `Domain` | `DEFER_UNTIL_CALLERS_STABLE` | Some are one-line wrappers, but many app/core callers use them. Public surface shrink has broad caller churn. | Later, one function per session |

## How

Completed deletion sessions:

1. Session 5.2 deleted the CLI executable forwarder files.
2. Session 5.3 deleted the viewer workspace factory.
3. Session 5.4 deleted the legacy frame-count generation wrapper.
4. Later: consider public domain convenience shrink only after CLI/viewer graph
   callers stop moving.

## Checklist

- [x] CLI command modules audited.
- [x] Legacy clip generation audited.
- [x] Graph compatibility forwarding audited.
- [x] Viewer factory/workspace interfaces audited.
- [x] One-adapter seams audited.
- [x] One-line forwarding functions audited.
- [x] Test-only exposed domain functions audited.
- [x] Deletion candidates and keep reasons recorded.
- [x] Follow-up shallow deletions completed.
