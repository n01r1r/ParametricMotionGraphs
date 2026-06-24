# CMU motion-space experiment (branch `experiment/cmu-motion-space`)

Question: does CMU mocap give a **smoother parametric manifold** than the Credo
"Center" corpus? Tested **without retargeting**.

## Why no code changed
The pipeline is skeleton-agnostic: `BvhLoader` reads arbitrary hierarchy + per-clip
fps; contact detection / foot-lock take joint indices as config; the strict
`RequireSkeletonCompatible` gate (name + parent + offset@1e-4 + channels) only
fires when **mixing** skeletons. A CMU-only space never mixes, so it just runs.

## Hard constraint
CMU rigs differ **per subject** (bone offsets), and the gate is 1e-4 on offsets.
So every example must come from **one subject**. Used subject 16 (canonical
locomotion). All 10 downloaded clips share a byte-identical HIERARCHY -> gate passes.

## Data (gitignored, re-fetch via `fetch.sh`)
Subject 16 straight steady clips, pathV measured from root XZ (cm/s @120fps):

| clip | pathV cm/s | gait | cycle window |
|------|-----------|------|--------------|
| 16_31 | 17.1 | walk | 290-419 |
| 16_21 | 29.9 | walk | 10-129 |
| 16_45 | 69.9 | run  | 5-89 |

## Result (`--audit-foot-skate`, sweep 9)

| space | gait span | max_adjacent_step (pose pop) | foot-lock effect on pop |
|-------|-----------|------------------------------|-------------------------|
| `cmu_walk_1d`    | within-gait (17->30 walk) | **1.27** | none (output post-proc) |
| `cmu_walkrun_1d` | cross-gait (17->70 walk->run) | **6.48** (5x) | none |

Both spaces: foot-slide cut ~78-80% by foot-lock; mid-sweep contacts collapse to 1
(the two anchor strides aren't phase-locked -> blended mid-poses lose clean plants).

## Conclusion
CMU is **not intrinsically smoother**. Same structure as the Center corpus:
within-family blends are smooth, cross-family (gait-change) blends jolt. The pose
pop is 5x larger across the walk->run gait boundary, and foot-lock cannot fix it
(it's a manifold gap, not a slide artifact).

CMU's real advantage is **density**: subject 16 offers many same-gait walk speeds,
so you can pick close-speed in-family pairs the sparse Center corpus can't supply.
The lever is coverage, not per-clip smoothness -- consistent with the original
clip-limit diagnosis. Blending CMU *into* the Center hull still needs retargeting
(out of scope; a separate graph node remains the correct structure).
