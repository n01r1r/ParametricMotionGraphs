# Supported PMG specs

This directory intentionally lists only specs that are currently usable with the available BVH corpus.

## Presentation / viewer demos

### `demo_walk_2d_triangle.pmg_spec`

Primary demo.

Use this for the smooth sparse parametric locomotion viewer demo. It places wide-walk, tight-walk, and jog anchors inside one 2-D parametric motion space and uses a single self-edge:

```text
walk_2d -> walk_2d
```

This matches the current corpus. It does **not** claim a full rectangular 2-D locomotion family because the `(tight turn, jog)` corner is missing.

### `demo_walk_self_edge_minimal.pmg_spec`

Minimal regression/demo spec.

Use this when testing one 1-D walk motion space with a single self-edge. It is the smallest stable viewer sanity check.

## Regression / audit specs

### `legacy_walk_curvature.pmg_spec`

Retained for regression and audit commands only. Do not use it as the main presentation demo. The middle anchor is known to make the measured turn response non-monotone.

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
