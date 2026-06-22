# Candidate-window provenance bridge

## Purpose

This branch connects offline candidate-window extraction to a human-inspectable,
spec-authorable provenance file. It reduces purely manual motion-example authoring.

It does not discover motion families, create PMG nodes, change GraphSpec, or change
runtime/controller/viewer behavior. Candidates remain proposals requiring human review.

## Command

```text
pmg_cli --extract-candidate-windows source.bvh --min-frames 20 --max-frames 120 --stride 5 --top-k 10 --output-md candidates.md --output-csv candidates.csv --output-candidates candidates.pmg_candidates.json
```

## JSON schema example

```json
{
  "schema": "pmg_candidate_windows_v1",
  "source_bvh_path": "source.bvh",
  "extraction_config": {"min_frames": 20, "max_frames": 120, "stride_frames": 5, "top_k": 10},
  "candidates": [{"source_bvh_path": "source.bvh", "start_frame": 10, "end_frame": 49, "score": 0.25, "reason": "aligned endpoint pose=...", "root_displacement": 12.0, "heading_delta_radians": 0.1}]
}
```

Frame ranges are inclusive. `root_displacement` uses source BVH native distance
units. Heading delta uses radians. Lower score ranks first; ties use start then end
frame. Empty clips produce a valid empty `candidates` array.

## Next step

GraphSpec examples currently reference whole BVH files. Human authors can inspect
this file, export selected ranges with `--export-bvh-recut`, then add those BVHs as
normal `example` lines. Direct frame-range references belong in a later GraphSpec
change if that workflow becomes necessary.
