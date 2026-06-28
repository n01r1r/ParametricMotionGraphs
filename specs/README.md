# Supported PMG specs

This directory intentionally lists only specs that are currently usable with the available BVH corpus.

**Data note:** only the CMU clips under `BVH/` are bundled. The non-CMU clips
(the `demo_walk_2d` "Center" anchors, the jog/`*A` action corpus) have
unconfirmed source/license and are git-ignored — not redistributed. Specs that
reference them (`demo_walk_2d`, the jog fixtures, the deprecated topology specs)
need those BVH supplied locally. See Provenance below.

## Presentation / viewer demos

### `demo_walk_2d.pmg_spec`

Primary canonical demo (single-family, paper-faithful `walk_2d`).

Use this for the smooth sparse parametric locomotion viewer demo. It places four
logically-similar walking anchors (wide walk, walk, tight walk, fast walk)
inside one 2-D parametric motion space and uses a single self-edge:

```text
walk_2d -> walk_2d
```

The spec declares `parameter_support walk_2d triangulated_2d`, so loading
validates the authored examples and the explicitly provided triangle indices.
In the viewer, use Graph -> Coverage to see the triangulated mesh overlaid on the
node's domain; the missing corners are marked as unsampled. Use
Graph -> Runtime -> Transition to compare the requested target parameter with the
actual transition parameter projected into authored support and the current
edge's reachable box.

Because every anchor is the same gait family, every authored pair classifies
`PASS_PAIR_PHASE_ALIGNMENT` (no cross-family pose-seam blowup), so within-hull
blends stay logically similar -- the parametric-synthesis precondition the paper
assumes. The travel-speed axis tops out at the authored `walkStraightTwiceAsFast`
anchor (~0.75); the old `[0, 1]` jog corner is intentionally excluded. Per the
paper, jog belongs in a separate node connected by a transition edge, not blended
into the walk space. See `docs/audits/jolt-cross-family-diagnosis-20260623.md`.

### `demo_walk_2d_triangle.pmg_spec`

Legacy simplex fixture/demo.

Use only for three-anchor simplex regression. It is not the canonical
presentation or CTest demo.

### CMU mocap demos

Curated high-quality CMU clips, tracked directly under `BVH/` (self-contained,
no fetch step). Same method as `demo_walk_2d`, applied to standard mocap:

- `cmu_walk_1d.pmg_spec` — subject 16, 1-D walking-speed blend (`16_31` ~17.1
  cm/s ↔ `16_21` ~29.9 cm/s). Safe within-gait aux demo.
- `cmu_walkrun_1d.pmg_spec` — subject 16, walk↔run speed axis (`16_31`, `16_45`).
- `cmu_gait_graph.pmg_spec` — subject 16, 2-node walk/run graph (`16_21/31/36/45`).
- `cmu78_gait_graph.pmg_spec` — subject 78 (gate PASS), 2-node walk/run graph
  (`78_10/27/29/30`).

Provenance: CMU Graphics Lab Motion Capture Database (mocap.cs.cmu.edu), subjects
16 and 78; clip filename `NN_MM.bvh` = subject_trial. Subjects 127/86 were
evaluated but excluded (127 calibration-loose, 86 data-blocked). These eight CMU
clips are the only BVH redistributed with the repo; the non-CMU corpus is
git-ignored (unconfirmed provenance) and must be supplied locally.

## Regression / audit specs

## Unit fixtures

Fixtures live under `specs/fixtures/`.

- `fixtures/fixture_edge_selective_good_bad.pmg_spec`
- `fixtures/fixture_transition_box_shrink.pmg_spec`
- `fixtures/fixture_walk_2d_jog_crossfamily.pmg_spec`

These are not viewer demos. The first two intentionally stress edge
classification and conservative transition-domain shrink behavior. The third is
the old jog-in-walk-hull spec, retained to exercise the cross-family within-hull
jolt and edge-box acceptance overreach that motivated the single-family
canonical demo.

## Removed / deprecated specs

The old split walk/jog topology specs are not part of the supported set:

- `demo_walk_jog_topology.pmg_spec`
- `demo_walk_jog_topology_recut.pmg_spec`
- `demo_walk_jog_topology_recut_loose.pmg_spec`
- `demo_walk_jog_topology_recut_dynamics.pmg_spec`

They require direct cross-node gait transitions with insufficient jog coverage and no authored gait-change transition clip. The builder is expected to drop or reject some of those edges.
