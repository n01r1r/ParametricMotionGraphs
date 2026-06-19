# Offline PMG Build Sequence Audit

## Purpose

Document the current paper offline construction path before moving build
orchestration out of GraphSpec.

## Sequence

1. `LoadGraphSpec(path)` parses the whitespace-delimited spec file.
   Relative BVH paths are resolved against the spec file directory.

2. `LoadGraphSpec()` performs structural validation:
   node existence, duplicate edge/config guards, parameter dimensions, expected
   support coverage, simplex support, triangulated 2D support, and active
   calibration metrics.

3. `BuildPmgArtifactFromSpec(spec, config)` is the compatibility entry point.
   It delegates to `BuildPmgOfflinePipeline(spec, config)`.

4. `BuildPmgOfflinePipeline()` maps `ArtifactBuildConfig` to
   `MotionSpacePreparationConfig`.

5. `PrepareMotionSpaces()` loads every example BVH once, enforces one compatible
   skeleton, optionally extracts the first cycle, canonicalizes root origin, and
   records source BVH paths.

6. `PrepareMotionSpaces()` prepares each node motion space in production order:
   authored examples, explicit parameter support, contact registration, optional
   DTW refinement, optional parameter calibration.

7. `BuildPmgOfflinePipeline()` creates graph nodes from prepared production
   motion spaces and records node registration metadata.

8. `BuildPmgOfflinePipeline()` copies runtime generation metadata:
   generated frame count and frames per second. Invalid runtime frame settings
   fail before edge construction.

9. `BuildPmgOfflinePipeline()` builds each declared edge with
   `PmgBuilder::BuildEdgeWithReport()`, using the edge-specific config when
   present and otherwise the default build config.

10. Rejected edge candidates are recorded in `artifact.metadata.edge_builds`.
    Created edges are added to the runtime graph. If every declared edge is
    rejected, build fails.

11. `GraphIo` is the persistence boundary. Callers save the returned
    `BuiltPmgArtifact` with `SavePmgArtifactText()` or load prior artifacts with
    `LoadPmgArtifactText()`.

## Checklist

- Spec parse: `LoadGraphSpec()`
- Structural validation: `LoadGraphSpec()` validators
- BVH loading: `PrepareMotionSpaces()`
- Motion preparation: `PrepareMotionSpaces()`
- Registration: `PrepareMotionSpaces()`
- Support setup: `PrepareMotionSpaces()`
- Sampled transition build: `BuildPmgOfflinePipeline()`
- Metadata population: `BuildPmgOfflinePipeline()`
- GraphIo persistence boundary: `SavePmgArtifactText()` / `LoadPmgArtifactText()`
