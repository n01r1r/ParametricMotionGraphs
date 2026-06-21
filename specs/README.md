# Supported PMG specs

This directory intentionally lists only specs that are currently usable with the available BVH corpus.

## Presentation / viewer demos

### `demo_walk_2d_triangulated.pmg_spec`

Primary demo.

Use this for the smooth sparse parametric locomotion viewer demo. It places wide-walk, tight-walk, and jog anchors inside one 2-D parametric motion space and uses a single self-edge:

```text
walk_2d -> walk_2d
```

The primary demo uses a triangulated 2D scattered support over five authored locomotion anchors.
The spec declares `parameter_support walk_2d triangulated_2d`, so loading validates the authored examples and the explicitly provided triangle indices.
In the viewer, use Graph -> Coverage to see that triangulated mesh overlaid on the node's domain; the missing corners are marked as unsampled. Use Graph -> Runtime -> Transition to interact with the 2D canvas and compare the requested target parameter with the actual transition parameter projected into authored support and the current edge's reachable box.

### `demo_walk_2d_triangle.pmg_spec`

Legacy simplex fixture/demo.

Use only for three-anchor simplex regression. It is not the canonical
presentation or CTest demo.

## Regression / audit specs

## Unit fixtures

Fixtures live under `specs/fixtures/`.

- `fixtures/fixture_edge_selective_good_bad.pmg_spec`
- `fixtures/fixture_transition_box_shrink.pmg_spec`

These are not viewer demos. They intentionally stress edge classification and conservative transition-domain shrink behavior.

## Removed / deprecated specs

The old split walk/jog topology specs are not part of the supported set:

- `demo_walk_jog_topology.pmg_spec`
- `demo_walk_jog_topology_recut.pmg_spec`
- `demo_walk_jog_topology_recut_loose.pmg_spec`
- `demo_walk_jog_topology_recut_dynamics.pmg_spec`

They require direct cross-node gait transitions with insufficient jog coverage and no authored gait-change transition clip. The builder is expected to drop or reject some of those edges.
