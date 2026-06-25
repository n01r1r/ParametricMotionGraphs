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

---

# Multi-node gait graph (sweep -> faithful topology)

`cmu_gait_graph.pmg_spec`. Built data-driven from the full 10x10 transition-distance
sweep (`out/transition_matrix.txt`, `--inspect-transition` on recut cycles).

## Sweep finding
- **within-walk** D = 10-230 (tight, reliable)
- **within jog+run** D = 67-900; jog->run cheap (16_35->16_45 = 67)
- **walk <-> jog/run wall**: D >= 998 for EVERY crossing

So the faithful topology is **2 nodes, not 3** -- jog sits *with* run on the far
side of the wall (walk->jog 1472 is no better than walk->run 1274). Nodes:
`walk_cmu` (16_31@17.1 <-> 16_21@29.9), `run_cmu` (16_36@47.1 <-> 16_45@69.9).

## Build result (`--build-graph`, source-range restriction ON by default)
- Self-edges (walk->walk, run->run) build cleanly -> nodes are reliable.
- Cross-edges build via the §6 source-range restriction (now the DEFAULT; use
  `--no-restrict-source-range` for the legacy all-or-nothing drop). Even the
  **best** crossing in the build's kovar metric is D~2709 (walk->run) / D~1932
  (run->walk) -- ~10x the self-edge (~250) at *every* source sample. The graph
  CONNECTS but the cross edges are **teleport-grade**, not clean planted
  transitions. Root cause: subject 16 has **no walk<->run transition clip**.
  Faithful fix = a bridge clip that contains the gait change (cf. walkToJog in the
  Center corpus), not threshold tuning.

## CLI changes
`apps/PmgGraphCommands.cpp`: wired the `restrict_source_range` builder flag (PR #57,
was only on a runtime command) into `--build-graph`. The default was then flipped
ON (faithful to paper §6's method), with `--no-restrict-source-range` exposing the
legacy all-or-nothing ablation.

## KNOWN BUG discovered (pre-existing, unrelated to the above)
A **dim=1** graph builds (V13) but `LoadPmgArtifactText` fails on reload
("failed to read parameter vector") -> `--inspect-graph`/viewer/runtime cannot open
a 1-D `.pmg`. All `tests/test_graph_io.cpp` round-trips use dim=2, so the 1-D path
was never exercised. Independent of the `--restrict` change (reproduces on a full
self-edge-only 1-D build). The graph builds and audits fine; only the
text-artifact round-trip is broken. Needs a focused GraphIo fix + a dim=1
round-trip test.
