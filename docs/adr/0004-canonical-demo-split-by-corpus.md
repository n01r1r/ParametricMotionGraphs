# ADR-0004: The Canonical Demo Stays Split Because the Corpus Is Incomplete

## Status

Accepted

## Context

A spec/storage/runtime audit asked whether the shipped `.pmg_spec` files are
genuine Parametric Motion Graphs or convenient fixtures. The verdict was that
they are valid PMG instances but that **no single spec is simultaneously
multi-node, multi-dimensional, and fully sampled**:

- `specs/demo_walk_2d_triangle.pmg_spec` is a genuine 2-D space (turn ×
  gait) with measured-metric calibration, but its three examples — `walkCurve`
  @ (0,0), `walkTightCurve` @ (1,0), `jogCurve` @ (0,1) — form a triangle. The
  (1,1) **tight-turn jog** corner is unsampled, so that quadrant is
  extrapolation rather than interpolation.
- `specs/demo_walk_jog_topology.pmg_spec` is a genuine directed multi-node graph (walk ↔ jog
  with both self-loops), but each node is 1-D.

The natural "fix" is one demo with a fully-sampled 2-D space across several
nodes. That requires a tight-turn jog clip (for the (1,1) corner) and, ideally,
a third locomotion action — both in a skeleton compatible with Group B.

## Decision

**Do not synthesize the missing corner, and do not fabricate BVH data.** The
available corpus cannot support a 4-corner 2-D locomotion space:

- Per `docs/MOTION_CORPUS.md`, only **Group B** has systematic curvature and
  speed variation, and within Group B `jogCurve` is the **only** jog clip — a
  wide curve. There is no tight-turn jog, so the (1,1) corner has no source
  motion. The other Group-B clips (`walkSpiral`, `walkFastSpiral`,
  `walkStraightTwiceAsFast`, `walkToJog`) are walk-family or transition clips,
  not a second jog gait.
- Cross-group clips are rejected at build time (skeleton bone-offset mismatch),
  so the gap cannot be filled from Groups A/C/D or the singletons.

The canonical demo therefore **stays split** by necessity:
`demo_walk_2d_triangle` carries the dimensionality story and
`demo_walk_jog_topology` carries
the topology story.

To keep this honest rather than implicit, specs declare machine-readable
structural expectations that `LoadGraphSpec` enforces (the `expect` keyword,
added with the spec-expectation validator):

- `demo_walk_2d_triangle` declares `expect walk_2d spanned_axes 2` (true: both
  axes vary) and **deliberately omits** `expect walk_2d corner_coverage full`,
  with a comment recording why the (1,1) corner is absent.
- `demo_walk_self_edge_minimal` declares `expect walk corner_coverage full`
  because its 1-D
  axis genuinely samples both extremes.
- `demo_walk_jog_topology` declares full 1-D walk coverage and singleton jog
  dimensionality separately, so topology remains explicit without implying a
  parameterized jog family.

## Consequences

- A future architecture review should **not** re-suggest adding the (1,1)
  tight-jog corner or a third locomotion node without first acquiring new
  motion data. The blocker is the corpus, not the spec grammar or the builder.
- If a tight-turn jog clip (and ideally a distinct third Group-B-compatible
  locomotion action) is later captured or sourced, revisit: extend
  `demo_walk_2d_triangle` to the fourth corner and add the
  `expect ... corner_coverage full` claim, and consider promoting the split
  demo into a single multi-node multi-dim spec.
- Importing a foreign skeleton (e.g. CMU) to obtain a tight jog is out of scope
  here: it would form its own skeleton group and require re-verifying the
  loader, contact-joint heuristics, and units before any space could use it.
- The `expect` validator means the gap is now a visible, enforced property of
  the spec rather than a claim buried in a comment, so the demo's honesty does
  not depend on a reviewer noticing.
