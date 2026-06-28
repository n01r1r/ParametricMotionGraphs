# Supported PMG specs

This directory lists the specs shipped with the viewer-only distribution. Each
is usable with the BVH clips tracked under `BVH/`. Regression fixtures, audit
specs, and the full corpus live on the development branches (`dev/core` and
the other `dev/*` branches). Clip sources are documented under "Data
provenance" in the top-level `README.md`.

## Primary demo

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
into the walk space.

Source: the four anchors (`walkCurve`, `walkMoreCurve`, `walkTightCurve`,
`walkStraightTwiceAsFast`) are the bundled "Center"-skeleton locomotion clips
(BVH `ROOT Center`) shipped with the project. See `README.md` > Data provenance.

## CMU mocap demos

Curated single-subject demos over CMU Graphics Lab MoCap clips tracked under
`BVH/` (`NN_MM.bvh` = subject_trial). They exercise multi-node walk/run graphs
on a real corpus.

- `cmu_walk_1d.pmg_spec` -- 1-D walking-speed blend (single node).
- `cmu_walkrun_1d.pmg_spec` -- 1-D walk/run speed blend.
- `cmu_gait_graph.pmg_spec` -- subject 16 walk/run, two nodes + transition edge.
- `cmu78_gait_graph.pmg_spec` -- subject 78 walk/run, two nodes + transition edge.
