# Paper-to-Code Map

## Status Labels

- **Direct**: same responsibility and core equation.
- **Adapted**: paper behavior with a documented representation change.
- **Extension**: repository behavior beyond the paper.
- **Out of scope**: intentionally not implemented.

## Motion Graphs (2002)

| Paper concept | Status | Implementation | Primary tests |
|---|---|---|---|
| Motion clip as root positions plus joint rotations | Direct | `MotionClip`, `Pose`, `BvhLoader` | `test_motion_clip`, `test_bvh_loader` |
| Windowed point cloud | Adapted | `MotionDistance::BuildPointCloud` | `test_motion_distance`, `test_distance_grid` |
| Closed-form floor alignment, Eqs. 1-4 | Direct | `MotionDistance::AlignedPointCloudDistance`, `RigidTransform2D` | `test_motion_distance`, `test_rigid_transform` |
| Frame-pair distance function | Direct | `MotionDistance::BuildDistanceGrid` | `test_distance_grid` |
| Candidate transition threshold | Adapted | `FindOptimalTransition`, PMG double thresholds in `PmgBuilder` | `test_distance_grid`, `test_pmg_builder` |
| Root interpolation and joint slerp, Eqs. 5-6 | Direct | `BlendPose` | `test_pose_blend` |
| Cubic C1 blend, Eq. 7 | Direct with reversed weight convention | `RuntimeController::CurrentPose` | `test_runtime_controller` |
| Accumulated floor placement | Direct | `RuntimeController`, `RigidTransform2D::Compose` | `test_runtime_controller` |
| Constraint-aware footplant cleanup | Adapted, optional | `ContactDetection`, `FootLocking` | `test_contact_detection`, `test_foot_locking` |
| Local-minimum harvesting over database | Out of scope | None | None |
| Largest-SCC pruning | Out of scope | None | None |
| Branch-and-bound walk search, Eq. 8 | Out of scope | None | None |
| Path-fitting objective, Eq. 9 | Out of scope | None | None |

### Point-cloud differences

`MotionDistance` follows the paper's floor-alignment mathematics, with these
representation differences:

- points are joint world positions, not downsampled mesh vertices;
- directional windows are endpoint-clamped when a configured phase range
  reaches a clip boundary;
- optional velocity weighting is a repository extension and defaults off.

The source-start/target-end placement and raw weighted squared sum now match
Kovar Section 3.1 exactly. Joint sampling and endpoint handling still make
thresholds corpus/configuration specific.

### Alignment formula map

| Mathematical quantity | Code |
|---|---|
| total weight | `total_weight` |
| weighted source floor sums | `sum_ax`, `sum_az` |
| weighted target floor sums | `sum_bx`, `sum_bz` |
| floor-plane dot term | `dot_term` |
| floor-plane cross term | `cross_term` |
| optimal yaw | `atan2(numerator, denominator)` |
| optimal translation | `alignment.dx`, `alignment.dz` |
| aligned weighted error | `weighted_squared` |

## Parametric Motion Graphs (2007)

| Paper section | Status | Implementation | Primary tests |
|---|---|---|---|
| Parametric motion-space node | Adapted | `ParametricMotionSpace` | `test_parametric_motion_space` |
| Smooth registered examples | Adapted | `RegisterSpaceByContacts`, `RefineRegistrationByDtw`, `TimeWarp` | `test_motion_registration`, `test_dtw_refine`, `test_registered_blending` |
| Accurate motion parameterization | Adapted | `CalibrateParameterMetrics` for vector metrics | `test_parametric_motion_space`, `test_graph_io` |
| Sec. 3.1 clip-pair transition test | Direct with point-cloud differences | `FindOptimalTransition` | `test_distance_grid` |
| Sec. 3.2 source/target random sampling | Direct | `PmgBuilder::BuildEdgeWithReport` | `test_pmg_builder` |
| GOOD / NEUTRAL / BAD thresholds | Direct | `PmgBuilderConfig` classification | `test_pmg_builder`, `cli_validate_graph_walk` |
| GOOD bounding box | Direct | `ParameterAabb::ExpandToInclude` | `test_pmg_builder` |
| BAD exclusion by minimal box adjustment | Direct | `ParameterAabb::ShrinkToExclude` | `test_pmg_builder`, `fixture_transition_box_shrink.pmg_spec` |
| Reject edge when a source sample has no target region | Direct for sampled sources | early return from `BuildEdgeWithReport` | `test_pmg_builder` |
| Average normalized transition point | Direct compatibility field | scalar fields in `TransitionSample` | `test_pmg_builder`, `test_graph_io` |
| Target-dependent phase field | Extension | `TargetTransitionPhaseSample`, `ResolveTargetPhases` | `test_edge_lookup`, `test_graph_io` |
| Runtime nearest-neighbor lookup, Eqs. 1-3 | Direct | `PmgEdge::LookupInterpolated` | `test_edge_lookup` |
| Runtime target clamping | Direct | `ParameterAabb::Clamp` | `test_edge_lookup`, `test_runtime_controller` |
| Runtime alignment recomputation | Direct | `PointCloudAlignment::Resolve` | `test_alignment_strategy` |
| Runtime blend | Direct | `RuntimeController` | `test_runtime_controller` |
| Random graph walk | Direct application | `ChooseRandomOutgoingTransition`, CLI `--random-walk` | `test_goal_directed_locomotion`, CLI tests |
| Target-directed control | Adapted application | `GoalDirectedLocomotion`, CLI `--goto` | `test_goal_directed_locomotion`, CLI tests |
| Interactive control | Adapted application | viewer runtime controls | build/manual viewer inspection |

### PMG interpolation formula map

The paper selects `k = dimension + 1` nearest source samples.

```text
raw_weight_i = 1 / distance(query, sample_i)
             - 1 / distance(query, sample_k)

weight_i = raw_weight_i / sum_j raw_weight_j
```

Implementation:

- exact matches are handled before reciprocal-distance evaluation;
- the cutoff is the selected `k`th neighbor, matching Equation 2 (verified
  against Heck PhD thesis Ch. 6 Eqs. 6.1-6.2 and Allen et al. [ACP02]);
- the farthest selected neighbor receives zero raw weight;
- zero/invalid weight sums fall back to the nearest sample;
- min and max target-box corners are weighted independently;
- target phase samples use a separate local inverse-distance interpolation.

See [`adr/0002-pmg-knn-cutoff.md`](adr/0002-pmg-knn-cutoff.md) for the cutoff
decision.

## Offline Build Map

```text
GraphSpec
  -> LoadGraphSpec
  -> BvhLoader::Load
  -> RequireSkeletonCompatible
  -> ExtractFirstCycle
  -> RegisterSpaceByContacts
  -> RefineRegistrationByDtw
  -> CalibrateParameterMetrics
  -> PmgBuilder::BuildEdgeWithReport
  -> BuiltPmgArtifact
  -> SavePmgArtifactText
  -> config.json / metrics.json / report.md / edge_samples.csv
```

Key files:

- `include/pmg/GraphSpec.h`, `src/GraphSpec.cpp`
- `include/pmg/ParametricMotionSpace.h`, `src/ParametricMotionSpace.cpp`
- `include/pmg/MotionDistance.h`, `src/MotionDistance.cpp`
- `include/pmg/PmgBuilder.h`, `src/PmgBuilder.cpp`
- `include/pmg/PmgArtifact.h`, `src/PmgArtifact.cpp`
- `include/pmg/GraphIo.h`, `src/GraphIo.cpp`

## Online Runtime Map

```text
RuntimeControlRequest
  -> find matching outgoing edge
  -> PmgEdge::LookupInterpolated
  -> clamp desired target parameter
  -> ParametricMotionSpace::GenerateClip
  -> PointCloudAlignment::Resolve
  -> RuntimeController blend
  -> accumulate RigidTransform2D world placement
```

Key files:

- `include/pmg/RuntimeController.h`, `src/RuntimeController.cpp`
- `include/pmg/AlignmentStrategy.h`, `src/AlignmentStrategy.cpp`
- `include/pmg/GoalDirectedLocomotion.h`,
  `src/GoalDirectedLocomotion.cpp`

## Test-to-Claim Matrix

| Claim | Test evidence |
|---|---|
| Alignment removes floor yaw/translation | `test_motion_distance` |
| Distance grid uses source-start/target-end windows and returns its true minimum | `test_distance_grid` |
| Edge build is deterministic for a fixed seed | `test_pmg_builder` |
| BAD points are excluded from retained boxes | `test_pmg_builder` |
| PMG lookup handles exact and interpolated queries | `test_edge_lookup` |
| Registration preserves shared contact structure | `test_motion_registration`, `test_registered_blending` |
| DTW refinement preserves contact anchors | `test_dtw_refine` |
| V9 round-trip preserves calibration, transition distance D, and runtime behavior | `test_graph_io` |
| Runtime world placement remains continuous | `test_runtime_controller` |
| Goal control maps world goals to achievable turn rates | `test_goal_directed_locomotion` |
| Included BVH graph builds and streams | CTest CLI cases in `CMakeLists.txt` |

Tests establish these software contracts only. They do not establish perceptual
equivalence to the paper videos or unavailable datasets.
