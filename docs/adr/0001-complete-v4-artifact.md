# ADR-0001: Complete V4 Artifact Is the Offline/Online Seam

## Status

Accepted (format since extended to PMG_GRAPH_V6; the seam decision
is unchanged)

## Context

Legacy graph files stored motion spaces and edges but omitted the Skeleton and
build configuration. CLI applications rebuilt registered graphs through a
separate path, so a saved graph was not the object validated online.

## Decision

`BuiltPmgArtifact` owns the Skeleton, registered graph, runtime frame settings,
source paths, node registration settings, edge sampling settings, seeds, and
edge reports. `PMG_GRAPH_V4` serializes this complete artifact. Runtime commands
and the viewer load it directly. V2/V3 remain readable as graph-only legacy
files but cannot perform standalone point-cloud runtime alignment.

## Consequences

- Offline and online behavior cross one Interface.
- Reproduction metadata travels with the graph.
- V4 files are larger because they store the Skeleton and diagnostic reports.
