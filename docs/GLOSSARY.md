# Glossary

## Purpose

Map paper language to repository symbols and prevent one concept from acquiring
multiple names.

## Motion Data

| Term | Meaning | Code | Unit / shape |
|---|---|---|---|
| Motion clip | Regular pose samples at fixed rate | `MotionClip` | frames, frames/s |
| Pose | Root position plus local joint rotations | `Pose` | root `[3]`, rotations `[J]` |
| Frame | Integer index into a clip | `int frame_index` | `[0, N-1]` |
| Phase | Normalized clip coordinate | `float phase` | `[0, 1]` |
| Skeleton | Joint hierarchy, offsets, BVH channels | `Skeleton` | `J` joints |
| World joint position | Forward-kinematic point | `Vec3` | native BVH units |

Use **motion clip** for the object and **frame** only for an index/sample.

## Parameterized Motion

| Term | Meaning | Code | Unit / shape |
|---|---|---|---|
| Parametric motion space | One action generated from parameterized examples | `ParametricMotionSpace` | parameter dimension `d` |
| Parameter vector | Semantic control coordinate | `ParameterVector` | `[d]` |
| Example motion | Authored clip and parameter | `ExampleMotion` | one per sample |
| Parameter domain | Axis-aligned authored bounds | `ParameterDomain` | min/max `[d]` |
| Blend weights | Non-negative example contributions | `ComputeLocalBlendWeights` | sum to 1 |
| Parameter calibration | Requested value to measured blend inversion | `ParameterCalibration` | currently 1-D turn rate |
| Blended duration | Weighted example clip duration | `BlendedDurationSeconds` | seconds |

An authored parameter is not automatically a physical measurement. The
`turn_rate` metric makes the generated behavior follow the
anchor-interpolated measured turn-rate curve; the parameter coordinate itself
can remain normalized.

## Registration

| Term | Meaning | Code | Unit / range |
|---|---|---|---|
| Contact interval | Consecutive planted frames for one joint | `ContactInterval` | frame range |
| Strike phase | Contact interval start | `StrikePhase` | `[0, 1]` |
| Lift phase | Contact interval end | `LiftPhase` | `[0, 1]` |
| Canonical phase | Shared phase domain across examples | `TimeWarp` input | `[0, 1]` |
| Example phase | Clip-specific phase after warp | `TimeWarp::Evaluate` output | `[0, 1]` |
| Registration | Installation of one phase warp per example | `RegisterSpaceByContacts` | one warp/example |
| DTW refinement | Interior timing correspondences between contact anchors | `RefineRegistrationByDtw` | slope in `[1/2, 2]` |
| Root-delta blending | Integrate blended heading-local root steps | `GenerateClipFromWeights` | native units/frame |

Use **registration** for time correspondence. Use **alignment** only for the
rigid floor transform at a transition.

## Transition Distance

| Term | Meaning | Code | Unit / shape |
|---|---|---|---|
| Point cloud | Windowed corresponding body points | `PointCloud` | `[window * J]` points |
| Point-cloud metric | Aligned weighted mean squared distance | `AlignedPointCloudDistance` | native units squared |
| Distance grid | Distance for sampled source/target frame pairs | `DistanceGrid` | `[Ns, Nt]` |
| Optimal transition | Minimum grid cell | `OptimalTransition` | phases + distance |
| Alignment | Target-to-source floor transform | `RigidTransform2D` | radians, native units |
| Transition window | Frames used by metric and runtime blend | `window_size` | frames |

The repository's metric uses joint points and centered windows. It is
Kovar-derived, not a byte-for-byte reproduction of the 2002 sampling layout.

## Parametric Motion Graph

| Term | Meaning | Code | Unit / shape |
|---|---|---|---|
| PMG node | Named parametric motion space | `PmgNode` | graph node |
| PMG edge | Directed sampled transition mapping | `PmgEdge` | graph edge |
| Source sample | One sampled source parameter and its transition data | `TransitionSample` | source `[d_s]` |
| Reachable target box | Axis-aligned valid target region | `ParameterAabb` | min/max `[d_t]` |
| GOOD sample | Distance at or below `TGOOD` | `good_count` | classification |
| NEUTRAL sample | Distance between thresholds | `neutral_count` | classification |
| BAD sample | Distance at or above `TBAD` | `bad_count` | classification |
| Target phase sample | Retained GOOD target parameter and phase pair | `TargetTransitionPhaseSample` | target `[d_t]` |
| Interpolated transition | Runtime box and phase query result | `InterpolatedTransition` | box + two phases |

`TGOOD` and `TBAD` are corpus- and metric-specific squared-distance thresholds.
They have no universal physical interpretation.

## Artifact and Runtime

| Term | Meaning | Code | Contract |
|---|---|---|---|
| Built PMG artifact | Complete offline output | `BuiltPmgArtifact` | skeleton + graph + metadata |
| Artifact format | Text serialization version | `PMG_GRAPH_V6` | V2-V5 read compatibility |
| Runtime request | Desired node and parameter | `RuntimeControlRequest` | target coordinates |
| Runtime controller | Streams, aligns, blends, places motion | `RuntimeController` | one active clip/transition |
| World placement | Accumulated clip-local to world transform | `WorldTransform` | `RigidTransform2D` |
| Goal-directed locomotion | World target to 1-D parameter controller | `GoalDirectedLocomotion` | greedy local control |
| Random graph walk | Random outgoing edge and target parameter | `ChooseRandomOutgoingTransition` | seeded RNG |

## Naming Rules

Prefer:

```text
motion clip
pose
phase
parametric motion space
parameter vector
registration
alignment
reachable target box
transition window
world placement
```

Avoid ambiguous alternatives:

```text
animation / take / sequence       -> motion clip
time / progress                   -> phase, unless seconds are intended
blend tree / blendspace           -> parametric motion space
feature / input vector            -> parameter vector
retiming / alignment curve        -> registration or TimeWarp
root transform                    -> alignment or world placement, specify which
valid range                       -> parameter domain or reachable target box
```

## Symbols Used in the Papers

| Paper symbol | Meaning | Repository equivalent |
|---|---|---|
| `M(t)` | motion evaluated at time | `MotionClip::SampleNormalizedPhase` |
| `P(lambda)` | parameterized motion generator | `ParametricMotionSpace::GenerateClip` |
| `D(M1(t1), M2(t2))` | aligned window distance | `AlignedPointCloudDistance` |
| `Ls`, `Lt` | sampled source/target parameters | builder sample vectors |
| `Lt_GOOD`, `Lt_BAD` | classified target samples | `good_hits`, `bad_target_parameters` |
| `B(Ns, Nt)` | interpolated target transition region | `target_parameter_box` |
| `w_i` | PMG source-neighbor weight | `weights[n]` in edge lookup |
| `TGOOD`, `TBAD` | transition thresholds | `good_transition_threshold`, `bad_transition_threshold` |
