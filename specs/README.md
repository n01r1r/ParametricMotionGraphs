# Supported PMG specs

This directory intentionally lists only specs that are currently usable with the available BVH corpus.

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
