# Design

## Purpose

Provide an inspectable paper-core PMG implementation whose offline output is
the exact artifact consumed online.

## Core Modules

```text
GraphSpec
  -> BuildPmgArtifactFromSpec
      -> cycle extraction
      -> contact registration
      -> optional DTW refinement
      -> PmgBuilder edge sampling
  -> BuiltPmgArtifact
      -> GraphIo V4
      -> RuntimeController
      -> GoalDirectedLocomotion
```

`BuiltPmgArtifact` is the offline/online seam. It owns the Skeleton,
ParametricMotionGraph, runtime frame configuration, registration metadata,
edge sampling configuration, seeds, and build reports.

## Contracts

- BVH offsets/root positions and distance thresholds use native BVH units.
- Phase is normalized to `[0, 1]`.
- Generated clips derive their frame count from the blended example durations;
  explicit frame counts are diagnostic overrides.
- Graph runtime requires a V4+ artifact with a non-empty Skeleton.
- Node registration either names contact joints or explicitly disables DTW.
- Edge creation fails if any sampled source parameter has no valid target box.
- Random runtime selection considers only outgoing edges.
- Goal-directed locomotion currently requires a one-dimensional steerable node.

## Failure Modes

The implementation throws close to the invalid input for missing files,
unknown joints/nodes, incompatible skeletons, mismatched contact structures,
empty edges, malformed artifacts, invalid runtime dimensions, and unsteerable
goal-directed nodes.

## Limitations

The full prioritized deviation list lives in
[PAPER_CONFORMANCE.md](PAPER_CONFORMANCE.md). Structural highlights:

- Parameter-accuracy calibration covers one-dimensional nodes with the
  turn-rate metric; other dimensions/metrics fall back to Shepard weights.
- Manual GraphSpec parameterization replaces Kovar-Gleicher automatic database
  extraction and parameterization.
- The included corpus validates walking/jogging behavior, not the original
  boxing/platform experiments.
- Transition regions are axis-aligned boxes and clips transition near their
  end; mid-clip transitions and partial source domains remain future work.
- Foot locking is optional and not serialized as runtime behavior.
