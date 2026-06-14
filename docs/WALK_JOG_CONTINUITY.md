# Walk/Jog Continuity and Control Audit

Last updated: 2026-06-13.

## Purpose

Explain the visible walk loop interruption in `specs/walk_jog.pmg_spec`,
quantify it, define the `jog` and desired-parameter contracts, and separate
paper-faithful behavior from useful but non-equivalent reference code.

## Finding

The remaining walk interruption was caused by viewer request policy, not
world-space root placement.

The viewer stopped requesting the active node's self-edge when node and
parameter were unchanged. `RuntimeController` then raw-wrapped the generated
cycle. The extracted walk cycles are not joint-space periodic at their first
and last samples, so cycle folding preserves root travel but exposes a pose and
velocity seam.

The PMG runtime described by Heck and Gleicher instead repeatedly traverses the
self-edge. Near the source transition phase it aligns the next generated clip
and blends into its early phase over a short centered window. The apparent
phase reset is intentional clip concatenation; the blend is what hides it.

## Numerical Check

Setup:

- graph: `specs/walk_jog.pmg_spec`;
- runtime rate: 30 frames/s;
- sample duration: 20 seconds;
- transition blend: 5 frames, cubic smoothstep;
- pose step: mean Euclidean world-position displacement over all joints;
- acceleration: mean norm of the per-joint second finite difference;
- reported ratio: maximum divided by sequence median.

| Walk parameter | Playback | Max step / median | Max acceleration / median | Transitions |
|---:|---|---:|---:|---:|
| 0.0 | raw cycle wrap | 2.60 | 21.18 | 0 |
| 0.0 | self-edge blend | 1.66 | 4.73 | 33 |
| 0.5 | raw cycle wrap | 2.73 | 21.99 | 0 |
| 0.5 | self-edge blend | 1.65 | 4.47 | 33 |
| 1.0 | raw cycle wrap | 3.56 | 6.78 | 0 |
| 1.0 | self-edge blend | 1.99 | 2.68 | 31 |

The self-edge path lowers the worst acceleration ratio by 60-80% and the worst
pose-step ratio by 40-50%. This supports restoring repeated self-edge requests
in the viewer. It does not prove perceptual smoothness.

### Source-cycle seams

The same diagnostic applied directly to contact-extracted cycles:

| BVH | Skeleton group | Seam step / median | Seam acceleration / median | Assessment |
|---|---|---:|---:|---|
| `walkCurve` | B | 3.04 | 17.06 | current wide-turn anchor; raw wrap poor |
| `walkMoreCurve` | B | 3.11 | 17.85 | current middle anchor; raw wrap poor |
| `walkTightCurve` | B | 2.55 | 9.91 | current tight-turn anchor |
| `walkSpiral` | B | 1.84 | 10.99 | possible added curvature sample |
| `walkFastSpiral` | B | 9.48 | 44.08 | reject as loop anchor without cleanup |
| `walkStraightTwiceAsFast` | B | 1.67 | 3.23 | useful speed sample, not curvature-equivalent |
| `jogCurve` | B | 1.25 | 4.33 | current jog anchor |
| `walkToJog` | B | 4.56 | 28.60 | transition clip, not cyclic node example |
| `smoothWalk` | C | 1.35 | 7.57 | smoother step seam but incompatible skeleton |

`smoothWalk` cannot be mixed into the Group-B walk/jog space without explicit
retargeting. Adding an incompatible BVH would change bone offsets and violate
the blend contract. `walkSpiral` is the only immediate Group-B curvature
candidate worth a separate calibrated experiment; it should not replace an
anchor based on seam score alone.

### Blend-window experiment

Increasing the transition window was tested with permissive diagnostic
thresholds so edge rejection did not confound the comparison.

| Parameter | 5-frame acceleration ratio | 7-frame | 9-frame |
|---:|---:|---:|---:|
| walk 0.0 | 4.73 | 3.59 | 2.52 |
| walk 0.5 | 4.47 | 3.55 | 1.87 |
| walk 1.0 | 2.68 | 3.19 | 3.19 |
| jog 0.0 | 2.72 | 2.20 | 1.66 |

Nine frames help wide and middle turns and jog, but slightly worsen the tight
turn. Distance magnitude also changes with window size, so all edge thresholds
would need reproducible recalibration. The production window remains five
frames until that full edge-quality sweep exists.

## Paper Comparison

The implementation choice is based on primary sources:

- [Heck thesis, Chapter 6: Parametric Motion Graphs](https://pages.cs.wisc.edu/~heckr/Thesis/6-PMGs.pdf)
  defines a locomotion node as short, two-step clips and repeatedly selects an
  outgoing edge, target parameter, alignment, and centered linear-blend
  transition. It also states that PMG requires smooth parametric motion spaces.
- [Kovar and Gleicher 2004](https://graphics.cs.wisc.edu/Papers/2004/KG04/)
  builds those smooth spaces through automatic related-motion extraction,
  constrained time alignment, registration, blending, and sampled inverse
  parameterization. Its walk experiment reduced 96 examples to 46 after
  redundancy removal.
- This repository uses three manually selected walk examples, contact anchors,
  optional slope-constrained DTW, and root-delta reconstruction. It does not
  reproduce the KG04 match web, match graph/reference selection, monotone cubic
  B-spline registration, or original walk corpus. Therefore smoothness is an
  input assumption only approximated here, not a reproduced paper result.

PMG does not continuously change the parameter inside an already generated
clip. It chooses the next parameter at a graph transition. Short clips provide
responsiveness.

## `maxxgx/motion-graphs` Comparison

Reference: [maxxgx/motion-graphs](https://github.com/maxxgx/motion-graphs),
inspected at commit `45372d896ea84ca1b7f14b69f76d74f60106b8e3`.

Useful agreement:

- root translation uses linear interpolation;
- joint rotations use quaternion slerp;
- transition weight is cubic with zero endpoint slopes;
- both source and target windows participate in the blend.

Material differences from the original Motion Graphs and PMG papers:

- it implements discrete Motion Graphs, not parametric motion spaces or PMG
  reachable target boxes;
- its graph construction excludes transitions within the same source
  animation, so it cannot model this PMG self-edge case;
- its point-cloud distance sums the square of an already squared point
  distance, producing fourth-power distance;
- it does not perform the paper's optimal floor-plane alignment in that
  distance function;
- local-minimum and graph-validity handling is not equivalent to the original
  SCC/path machinery.

Only its cubic pose blend is corroborating implementation evidence. Copying its
graph or distance code would reduce paper fidelity.

## Why Jog Cannot Be Adjusted

`walk_jog.pmg_spec` declares:

```text
node jog 1
example jog 0.0 ../BVH/jogCurve.bvh
```

One example produces parameter domain `[0.0, 0.0]`. The viewer slider has equal
minimum and maximum, generated requests clamp to 0, and the runtime has no
second jog example to blend toward. Declaring dimension 1 does not create a
controllable degree of freedom. The viewer now reports this as a fixed
parameter instead of displaying an inert slider.

A controllable jog node needs at least two compatible, semantically ordered jog
examples, preferably several curvature samples, followed by registration,
metric calibration, and edge rebuild. No such jog family exists in the current
Group-B corpus.

## Desired Parameter Contract

`desired_parameter` is the requested coordinate in the target node's
parametric motion space:

1. viewer chooses `desired_node` and target-space coordinate;
2. outgoing edge lookup computes the reachable target box;
3. request is clamped to that box;
4. target node generates the next short clip at the clamped coordinate;
5. runtime aligns and blends into it at the stored transition phases.

For `walk_jog`:

- `walk`: authored scalar 0..1, calibrated against measured `turn_rate`;
- `jog`: singleton scalar fixed at 0.

For `walk_curvature_speed.pmg_spec`:

- axis 0: turn/curvature;
- axis 1: gait/speed, calibrated with `travel_speed`;
- goal-directed goto now drives both axes (turn_rate heading + travel_speed
  pace); the manual blend slider still exposes axis 0 and holds axis 1 at its
  midpoint.

`walk_curvature.pmg_spec` remains the clean one-slider steering demo.
`walk_curvature_speed.pmg_spec` is the 2-D data contract and is now fully
steerable under goto; only the manual per-axis slider UI is still 1-D.
