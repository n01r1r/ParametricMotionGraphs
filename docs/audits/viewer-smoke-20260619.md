# Viewer Smoke — 2026-06-19

## Scope

Manual viewer acceptance checklist. This non-interactive session did not perform GUI input or visual inspection; items remain pending. Automated evidence is noted but does not replace manual verification.

## Manual Checklist

- [ ] artifact load
- [ ] spec build
- [ ] graph runtime playback
- [ ] parameter projection display
- [ ] transition diagnostics
- [ ] reset/restart
- [ ] trace clear
- [ ] goto locomotion
- [ ] save/reload
- [ ] root canonicalization markers

## Automated Evidence

- Debug viewer executable built: `build/Debug/pmg_viewer.exe`.
- Full suite passed: 37/37.
- Focused suite passed: 8/8, including viewer graph authoring/runtime, viewer runtime module, runtime controller, graph I/O/spec, PMG builder, offline pipeline.
- Full suite also passed artifact build, goto locomotion, root canonicalization, and viewer host tests.

## Manual Run Record

- Status: NOT RUN
- Reason: GUI interaction and rendered-output inspection unavailable in current non-interactive verification session.
- Acceptance: run executable, exercise every checklist item, then replace each pending box with pass/fail plus observed details.
