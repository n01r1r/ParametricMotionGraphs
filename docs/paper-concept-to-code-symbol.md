# Paper concept to code symbol

## Mapping

| Paper/domain concept | Primary code symbol | Location | Implementation note |
|---|---|---|---|
| parametric motion space | `pmg::ParametricMotionSpace` | `include/pmg/ParametricMotionSpace.h` | Evaluates/blends examples at `[parameter_dimension]` parameter and normalized phase |
| node | `pmg::PmgNode` | `include/pmg/ParametricMotionGraph.h` | Owns node name and motion space |
| edge | `pmg::PmgEdge` | `include/pmg/ParametricMotionGraph.h` | Directed source/target node indices plus transition samples |
| sampled transition | `pmg::TransitionSample` | `include/pmg/TransitionTypes.h` | Offline source sample, reachable target box, phases, distance |
| source parameter | `TransitionSample::source_parameter` | `include/pmg/TransitionTypes.h` | Interpolation query input to `PmgEdge::LookupInterpolated` |
| target bounding box | `TransitionSample::target_parameter_box` / `pmg::ParameterAabb` | `include/pmg/TransitionTypes.h` | Interpolated then clamps requested runtime target |
| transition point | `source_transition_phase`, `target_transition_phase` | `include/pmg/TransitionTypes.h` | Directional normalized phase pair; frame windows derived by `TransitionWindow` |
| runtime stream | `pmg::RuntimeController::Update`, `CurrentPose` | `include/pmg/RuntimeController.h` | Advances seconds and returns world-space pose |
| phase gate | `pmg::RuntimeController::TryScheduleTransition` | `src/RuntimeController.cpp` | Detects crossing of interpolated source transition phase |
| registration | `pmg::RegisterSpaceByContacts`, `BuildRegistrationWarps` | `include/pmg/MotionRegistration.h` | Offline example-to-canonical phase mapping; optional DTW refinement |
| alignment | `pmg::AlignmentStrategy`, `PointCloudAlignment` | `include/pmg/AlignmentStrategy.h` | Runtime target-to-source `RigidTransform2D` resolution |

## Boundary notes

- `InterpolatedTransition` is runtime interpolation of stored edge samples, not a
  new sampled transition.
- `RuntimeTransitionDiagnostics::reachable_target_box` exposes selected edge
  feasibility; it does not describe full target motion-space support.
- `PointCloudAlignment` implements renderer/runtime-conditioned local placement;
  its existence does not prove globally optimal motion alignment.

