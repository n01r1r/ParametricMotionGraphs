# ADR: Paper-first module boundaries

## Status

Accepted for current code. `experimental` remains `KEEP_AS_REFERENCE` and
`DO_NOT_MERGE`.

## Context

This repository needs the PMG paper algorithm to stay visible while CLI,
artifact compatibility, and viewer code support building and inspecting it.
The `experimental` branch is a useful minimal sketch, but lacks the current
GraphSpec, V12 artifact, offline build, runtime, and viewer contracts.

## Decision

### Paper-first implementation

`PMGFullPaper.tex` is the primary algorithm reference. Implement paper
contracts before optional extensions. Extensions must remain opt-in and must
not silently change paper-compatible defaults. Current example:
`kKovarDirectionalPointCloud` is default; `kDynamicsWindow` is an extension.

### Deep-module test

Keep a module when its small public surface hides cohesive policy, lifecycle,
validation, or algorithmic work. Delete or inline a module when it only
forwards one call and removing it does not duplicate policy. Current results:

- kept: `PmgOfflinePipeline`, `PmgBuilder`, `RuntimeController`,
  `ViewerRuntimeModule`, CLI command-family modules;
- deleted: CLI executable forwarder, viewer workspace factory, legacy
  frame-count clip-generation wrapper and its friend access.

See [shallow-module-deletion-audit](../audits/shallow-module-deletion-audit.md).

### Viewer/runtime seam

`ViewerRuntimeModule` owns viewer-side controller install/reset/update,
request preview, status, and trace state. `RuntimeController` owns transition
scheduling and pose-stream semantics. `PmgViewerWorkspace` owns UI, input,
filesystem discovery, and rendering. Viewer code may adapt runtime state; it
must not duplicate transition algorithms.

### GraphSpec adapter

`LoadGraphSpec` parses and structurally validates the text spec, resolving
relative BVH paths. `BuildPmgArtifactFromSpec` remains a compatibility entry
point and delegates orchestration to `BuildPmgOfflinePipeline`. GraphSpec is
an input adapter, not the PMG domain model.

### Infrastructure limit

`GraphIo` alone branches on artifact versions (reader V2-V12, writer V12).
CLI/viewer translate arguments and UI state into explicit configs/requests.
Current offline preparation still receives and opens BVH paths; this known
isolation violation is not described as resolved.

### Compatibility modules

Keep GraphIo legacy readers and `BuildPmgArtifactFromSpec`: persisted artifacts
and existing callers are real compatibility contracts. Remove them only with
an explicit format/caller migration, not as shallow-wrapper cleanup.

## Consequences

- Paper concepts remain traceable to code.
- Optional quality extensions stay distinguishable from paper-compatible flow.
- Adapter seams remain only where they isolate lifecycle, dependencies, or
  compatibility.
- No decision here claims future graph-workspace extraction or filesystem
  provider work is complete.

