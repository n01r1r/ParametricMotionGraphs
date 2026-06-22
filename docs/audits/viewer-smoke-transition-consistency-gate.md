# Viewer Smoke — Transition Consistency Gate

## Scope

- Branch: `transition-consistency-gate`
- Commit: `f2ec23a`
- Date: 2026-06-21
- Fixture: `specs/demo_walk_2d_triangulated.pmg_spec`
- Artifact: `build/viewer_out/walk_2d_triangulated.pmg` (`PMG_GRAPH_V12`)

This is the final manual merge gate. Automated checks below are preflight evidence only; they do not replace rendered-output inspection.

## Manual Checklist

| Item | Status | Required observation |
| --- | --- | --- |
| artifact load | NOT RUN | V12 artifact opens without viewer error |
| spec build | NOT RUN | spec builds from viewer and installs resulting graph |
| graph runtime playback | NOT RUN | runtime pose advances continuously |
| 2D parameter projection display | NOT RUN | raw and projected parameters are visible and coherent |
| transition diagnostics | NOT RUN | preview, active transition, status, and markers update coherently |
| reset / restart | NOT RUN | runtime restarts from a valid state and trace resets |
| trace clear | NOT RUN | path and transition markers clear immediately |
| goto locomotion | NOT RUN | ground target drives locomotion toward the selected point |
| save / reload | NOT RUN | saved graph reloads with matching runtime behavior |
| root canonicalization markers | NOT RUN | canonical root markers remain aligned with rendered motion |

## Automated Preflight

- `pmg_cli --validate-graph-spec specs/demo_walk_2d_triangulated.pmg_spec`: PASS; one 2D node, five examples, compatible 31-joint skeleton, one requested self-edge.
- `pmg_cli --build-graph specs/demo_walk_2d_triangulated.pmg_spec build/viewer_out/walk_2d_triangulated.pmg`: PASS; V12, one node, one edge, 48 generated frames at 30 fps.
- Focused `ctest` selection: PASS, 11/11. Covered builder, runtime controller, graph I/O/spec, offline pipeline, root canonicalization, goto locomotion, viewer host, viewer graph authoring/runtime, and viewer runtime module.
- Debug viewer executable exists: `build/Debug/pmg_viewer.exe`.

## Blocker

Manual GUI control and screenshot inspection were unavailable in this session. Windows automation connection failed twice with `sandboxCwd must use the file URI scheme`; no checklist item was inferred from automated tests.

## Merge Decision

**HOLD.** Branch becomes a merge candidate only after all ten manual items are changed to PASS with observed details. Any FAIL keeps the gate open.
