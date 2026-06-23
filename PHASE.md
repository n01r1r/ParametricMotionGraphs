1. Implementation Specification
# Specification: Non-destructive MotionClipSegment support for PMG examples

## Objective

Add general non-destructive BVH segment support to the PMG motion-example layer.

A motion-space example must be able to reference:

```text
source_bvh + start_frame + end_frame + parameter + optional phase/contact metadata

instead of requiring a physically recut BVH file.

This is needed because diagnostics showed that walk_2d failures are caused by bad source anchor loop windows, not by skeleton/FK or transition gates.

Design Principle

Do not overwrite or silently replace original BVH files.

Canonical input:

BVH/jogCurve.bvh + start_frame/end_frame metadata

Optional derived/debug export:

BVH/jogCurve_recut_012_039.bvh

The derived recut BVH must not become the source of truth.

Paper-faithful interpretation

The PMG paper assumes compatible, phase-aligned example clips before building parametric motion spaces and graph edges.

Therefore this implementation should treat segment selection as an offline authoring/build-stage operation:

raw BVH
→ valid MotionClipSegment
→ motion-space example
→ graph build
→ runtime playback/control

Do not implement runtime insertion of arbitrary BVH motion. Arbitrary compatible motion can later use the same MotionClipSegment abstraction, but only after compatibility checks, candidate segment extraction, and graph rebuild.

Required abstraction

Introduce or extend an existing motion example representation to support:

struct MotionClipSegment {
    std::string source_bvh;
    int start_frame = 0;
    int end_frame = -1;   // -1 means full clip / last frame
    std::string phase_label;        // optional
    std::string contact_start;      // optional diagnostic metadata
    std::string contact_end;        // optional diagnostic metadata
};

A motion-space example should conceptually become:

struct MotionExampleSpec {
    MotionClipSegment segment;
    std::vector<float> parameter;
};

Do not hard-code file names such as jogCurve.bvh or walkTightCurve.bvh in PMG builder logic.

Backward compatibility

Existing specs without start_frame / end_frame must continue to work.

Default behavior:

start_frame = 0
end_frame = last frame

Old behavior should be equivalent to full-clip segment usage.

Spec schema

Update the existing pmg_spec parser/writer to support segment fields.

Preferred schema shape, adapted to the existing repo format:

examples:
  - source_bvh: BVH/walkCurve.bvh
    parameter: [0.0, 0.0]

  - source_bvh: BVH/walkTightCurve.bvh
    start_frame: 12
    end_frame: 43
    parameter: [1.0, 0.0]
    phase_label: left_contact_start

  - source_bvh: BVH/jogCurve.bvh
    start_frame: 8
    end_frame: 35
    parameter: [0.0, 1.0]
    phase_label: left_contact_start

If the actual spec is not YAML, preserve the existing format and add equivalent fields.

Builder behavior

The PMG builder must:

Load the original BVH.
Extract only [start_frame, end_frame].
Preserve frame order and frame time.
Use the extracted segment as the example clip.
Store segment provenance into the built artifact, if artifact metadata already supports provenance.
Continue all existing root normalization, resampling, parameter interpolation, and edge construction logic on the extracted segment.

Invalid segment behavior:

start_frame < 0                → error
end_frame >= frame_count       → error
end_frame <= start_frame       → error
segment too short for motion   → error or explicit diagnostic failure
Diagnostics integration

Update existing diagnostics so anchor reports include segment provenance:

source_bvh
start_frame
end_frame
parameter
cycle score
root velocity seam
yaw seam
contact mismatch
start contact state
end contact state

audit-motion-space-registration should show whether a bad anchor is a bad source file or a bad selected segment.

Candidate search

If feasible in this PR, add a small command or mode to find candidate cycle windows:

pmg_cli find-cycle-segments BVH/jogCurve.bvh --out outputs/diagnostics/jogCurve_segments

Candidate scoring should prioritize:

contact state match
low cycle score
low root velocity seam
low yaw seam
reasonable duration / frame count

If this is too large, do not implement full auto-authoring. At minimum, make segment metadata supported and auditable.

walk_2d update

Use the new segment support to update only the bad walk_2d anchors:

Bad anchors from diagnostics:

anchor 2: walkTightCurve.bvh, param [1, 0]
anchor 3: jogCurve.bvh, param [0, 1]

Do not create a walk_2d-specific code path. The walk_2d fix must be expressed in the spec/source data using segment fields.

Non-goals

Do not:

- overwrite original BVH files
- make recut BVH files canonical inputs
- add second PMG node
- add CMU corpus expansion
- add transition quality gates
- add foot locking / IK
- continue UI migration
- change runtime behavior to hide bad loops
- implement runtime arbitrary-motion insertion
- hard-code walk_2d file names in builder logic
Acceptance criteria

The implementation is acceptable if:

1. Existing full-clip specs still build.
2. Specs with start_frame / end_frame build.
3. Artifact diagnostics report segment provenance.
4. Bad walk_2d anchors can be replaced by segment-window references.
5. audit-motion-space-registration improves or clearly reports remaining failures.
6. audit-transition-pop is rerun after rebuild.
7. Tests pass without external CMU corpus.
### Acceptance consistency residual

After phase-aligned segment windows and strict-interior consistency auditing,
the walk_2d acceptance-consistency audit drops from 13 accepted-bad cases to 1
at the high-sampling diagnostic profile.

The remaining case is not a boundary artifact:

- strict interior: true
- d / T_BAD: 1.029
- distance inside shrunk face: 0.020
- quality gate: survives

This is currently treated as an expected limitation of sparse sampling plus
axis-aligned reachable boxes, not a runtime bug. Runtime box containment remains
unchanged. The audit intentionally preserves this case as an honest diagnostic
signal rather than hiding it with tolerance or a runtime gate.
