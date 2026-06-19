# Infrastructure Isolation Audit

## Purpose

Check whether artifact I/O, filesystem handling, option parsing, build orchestration, and runtime control remain behind infrastructure adapters. This audit changes no runtime behavior.

## Scope

- `GraphIo`: serialized PMG artifact compatibility
- `GraphSpec`: spec parsing and relative BVH path resolution
- CLI: argument parsing, filesystem output, build/runtime commands
- viewer: artifact/spec discovery, load/save, build/runtime adapters
- domain: offline pipeline, graph, motion spaces, runtime controller

## Findings

| Boundary | Status | Evidence | Isolation violation |
|---|---|---|---|
| Artifact versions | Contained | `src/GraphIo.cpp` owns `PMG_GRAPH_V2` through `PMG_GRAPH_V12` parsing and writes V12. | None outside reporting text in `apps/PmgGraphCommands.cpp`; CLI does not branch on version. |
| Domain file paths | Violation | `GraphSpecExample::bvh_path`, `ArtifactMetadata::source_bvh_paths`, `PrepareMotionSpaces`, and `BuildPmgOfflinePipeline` carry/open BVH paths. | Domain build pipeline knows filesystem paths. Moving this requires a new content/provider contract and is deferred to avoid behavior change. |
| GraphSpec filesystem | Adapter-local | `LoadGraphSpec` opens spec and resolves relative BVH paths before build. | `GraphSpec` combines parsed domain description with file loading; accepted current adapter seam. |
| CLI parsing | Contained | `apps/Pmg*Commands.cpp` parses `argv`; `include/pmg` and `src` contain no CLI option parsing. | None. |
| Viewer filesystem | Contained | `PmgViewerWorkspace` discovers BVH/spec files and loads/saves artifacts using `std::filesystem`, `GraphIo`, and `GraphSpec`. | None in domain runtime. Diagnostic overlay reopens metadata BVH paths in viewer only. |
| Build path | Shared | CLI and viewer both call `LoadGraphSpec` then `BuildPmgArtifactFromSpec`, which delegates to `BuildPmgOfflinePipeline`. | None. |
| Runtime path | Shared contract | CLI uses `RuntimeController` directly; viewer `ViewerRuntimeModule` wraps the same `RuntimeController`, `RuntimeControlRequest`, and `RuntimeControllerConfigFromArtifact`. | Wrapper policy/status mapping exists only in viewer adapter; no duplicate transition algorithm found. |
| Options vs domain | Contained | CLI/viewer translate options/UI state into config/request values; domain objects do not read CLI/viewer state. | Domain config values are governed by adapters by design; domain invariants still validate inputs. |

## Direct Answers

- Does a domain algorithm know a file path? **Yes.** Offline preparation receives `GraphSpecExample::bvh_path` and loads BVH files. This is the remaining isolation violation.
- Does a runtime/build module know a serialized version? **No version branching found.** Compatibility branching is limited to `GraphIo`; CLI only reports the latest format label.
- Does a CLI/viewer option govern a domain object? **Only through explicit config/request contracts.** No domain module parses CLI/viewer options.

## Compatibility Contract

- Reader accepts V2-V12 inside `GraphIo`.
- Writer emits latest V12.
- Latest save/load roundtrip remains covered by `test_graph_io`.
- Malformed/unknown input fails with `LoadPmgArtifactText` exceptions.
- No artifact-version condition exists in graph, build-pipeline, or runtime modules.

## Checklist

- [x] Isolation violation list exists.
- [x] V2-V12 compatibility contained in `GraphIo`.
- [x] CLI/viewer build path converges on `BuildPmgOfflinePipeline`.
- [x] CLI/viewer runtime shares `RuntimeController` contract.
- [x] Option parsing stays in adapters.
- [x] No intentional behavior change beyond selecting already-required latest V12 serialization label.
