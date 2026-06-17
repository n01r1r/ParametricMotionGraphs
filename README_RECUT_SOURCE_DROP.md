# PMG recut source drop v3

This source drop is a corrected follow-up to v2.

## What changed from v2

`--export-known-cyclic-recuts` now writes the recut BVH files correctly, but
`--audit-cyclic-continuity specs/demo_walk_jog_topology_recut.pmg_spec` still
failed because `PrepareMotionSpaces` was attempting to run `ExtractFirstCycle`
again on the already-cropped `*_recut_*` BVH files.

A short recut clip may legitimately contain only one detected contact of the
cycle joint. It should be treated as the authored one-cycle clip, not as raw
multi-cycle footage that needs another contact-based cycle extraction pass.

`src/MotionSpacePreparation.cpp` now skips `ExtractFirstCycle` for BVH files
whose filename contains `_recut_`, while preserving the node registration,
contact joints, contact registration, DTW refinement, parameter calibration,
artifact metadata, and viewer diagnostics path.

## Included files

- `apps/PmgBvhCommands.cpp`
- `apps/PmgCommands.cpp`
- `src/MotionSpacePreparation.cpp`
- `specs/demo_walk_jog_topology_recut.pmg_spec`
- `specs/demo_walk_jog_topology_recut_dynamics.pmg_spec`

Copy these files over the repository root with the same relative paths.

## Commands

```powershell
cmake --build build --target pmg_cli

.\build\Debug\pmg_cli.exe --export-known-cyclic-recuts `
  --bvh-dir .\BVH `
  --output-dir .\BVH\recut

.\build\Debug\pmg_cli.exe --validate-graph-spec `
  .\specs\demo_walk_jog_topology_recut.pmg_spec

.\build\Debug\pmg_cli.exe --audit-cyclic-continuity `
  .\specs\demo_walk_jog_topology_recut.pmg_spec `
  --output-csv .\build\cyclic_recut.csv `
  --output-md .\build\cyclic_recut.md
```

Use `demo_walk_jog_topology_recut.pmg_spec` first. The dynamics spec is for a
separate threshold-calibration run.
