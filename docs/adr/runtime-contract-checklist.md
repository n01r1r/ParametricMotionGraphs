# Runtime Contract Checklist

## Purpose

Phase 12 refactor guardrail. Preserve current runtime semantics while code moves.

This document records observable contracts only. It is not permission to change behavior.

## Contract Items

### 1. Same node + same effective parameter = no-op

Definition:

- If request stays on current node
- and requested parameter projects/clamps to current effective parameter
- runtime must not schedule self-edge transition

Current source:

- `src/RuntimeController.cpp:333-349`
- `tests/test_runtime_controller.cpp:480-516`
- viewer status text path: `apps/viewer/ViewerRuntimeModule.cpp:64-68`
- viewer test: `tests/test_viewer_graph_runtime.cpp:76-78`

Expected observable behavior:

- `CompletedTransitions() == 0`
- `IsTransitioning() == false`
- viewer scheduling text becomes `no-op (parameter unchanged)` when request equals current parameter exactly

Refactor check:

- Preserve effective-parameter comparison after support projection / domain clamp, not only raw request equality.

### 2. Same node + different parameter = self-edge transition eligible

Definition:

- Same-node request with different effective parameter may schedule self-edge
- only if edge exists and phase gate is crossed

Current source:

- gate entry: `src/RuntimeController.cpp:352-397`
- transition schedule body: `src/RuntimeController.cpp:399-505`
- test: `tests/test_runtime_controller.cpp:165-261`

Expected observable behavior:

- no immediate transition before phase gate
- after gate crossing, active transition diagnostics appear
- after blend completion, `CompletedTransitions() >= 1`

### 3. Raw request / projected preview / runtime actual stay distinct

Definition:

- `raw request`: user-requested node parameter before support/domain correction
- `projected preview`: node-local support/domain projection shown before scheduling
- `runtime actual`: parameter actually used by scheduled edge after reachable-box clamp and, if present, `ProjectInside`

Current source:

- raw/projected storage: `apps/viewer/ViewerRuntimeModule.cpp:86-90`
- snapshot fields: `apps/viewer/ViewerRuntimeModule.cpp:127-133`
- runtime tab labels: `apps/viewer/PmgViewerWorkspaceGraph.cpp:1905-1963`
- transition scheduling:
  - self preview projection: `apps/viewer/PmgViewerWorkspace.cpp:656-664`
  - runtime actual selection: `src/RuntimeController.cpp:399-405`

Expected observable behavior:

1. `REQUESTED RAW` can lie outside support
2. `REQUESTED PROJECTED` is support/domain projection
3. `RUNTIME ACTUAL` stays `pending` before scheduling
4. during active transition, `RUNTIME ACTUAL` equals scheduled target parameter, not raw request

Refactor check:

- Do not collapse these three into one field/value.

### 4. Phase gate semantics stay PMG-centered by default

Definition:

- Runtime uses centered gate by default
- transition schedules when source clip phase crosses:
  - `source_transition_phase - half_window_phase`

Current source:

- config default: `include/pmg/RuntimeController.h:37-47`
- gate math: `src/RuntimeController.cpp:376-397`
- regression test: `tests/test_runtime_controller.cpp:252-261`

Expected observable behavior:

- request can remain pending for many updates
- scheduling begins near centered gate, not at arbitrary request time
- viewer status while preview exists but gate not crossed: `waiting for phase gate`

### 5. support-target-box projection uses both support and reachable box

Definition:

- Runtime target must satisfy transition reachable box
- If target node has explicit support, runtime target must also satisfy support
- For explicit support, use `ProjectInside(parameter, box)`, not support-only projection

Current source:

- runtime target selection: `src/RuntimeController.cpp:399-405`
- `ProjectInside`: `src/ParameterSupport.cpp:478-573`
- tests:
  - simplex projection: `tests/test_runtime_controller.cpp:428-478`
  - support+box intersection: `tests/test_runtime_controller.cpp:518-567`
  - triangulated 2D inside/outside: `tests/test_runtime_controller.cpp:569-649`

Expected observable behavior:

- `reachable_target_box.Contains(actual_target_parameter)` during active transition
- actual target can differ from projected preview if reachable box shrinks target further

### 6. completed transition count semantics

Definition:

- Counter increments only when active blend finishes and runtime commits `next_*` state into current state
- Counter resets to zero on `Start()`

Current source:

- reset on start: `src/RuntimeController.cpp:98-109`
- increment on finalize: `src/RuntimeController.cpp:144-159`
- viewer snapshot exposure: `apps/viewer/ViewerRuntimeModule.cpp:123-126`

Expected observable behavior:

- pending request does not increment count
- active transition start does not increment count
- count increments after blend completion only
- viewer `completed transitions` matches controller count

### 7. Trace clear / reset behavior

Definition:

- `Clear trace` clears path points + transition markers, then seeds path with current pose
- `Restart graph` restarts controller at current node/current parameter, then resets trace
- full runtime session reset clears runtime object, requested params, trace, markers, goto/steering overlays

Current source:

- clear trace: `apps/viewer/ViewerRuntimeModule.cpp:92-98`
- restart graph: `apps/viewer/ViewerRuntimeModule.cpp:43-52`
- runtime session clear: `apps/viewer/PmgViewerWorkspaceGraph.cpp:211-222`
- UI buttons:
  - `Restart graph`: `apps/viewer/PmgViewerWorkspace.cpp:1117-1122`
  - `Clear trace`: `apps/viewer/PmgViewerWorkspaceGraph.cpp:1833-1839`

Expected observable behavior:

1. `Clear trace` leaves trail count at one seed point or near-one after immediate next update
2. `Restart graph` clears completed transition count because controller `Start()` resets state
3. `ResetGraphRuntimeSession()` leaves runtime not ready

### 8. 2D runtime control contract

Definition:

- 2D target canvas updates requested vector immediately
- viewer may show request before transition active
- active transition overlays both requested point and actual scheduled point

Current source:

- control canvas + drag behavior: `apps/viewer/PmgViewerWorkspaceGraph.cpp:1725-1806`
- vector fallback logic: `apps/viewer/PmgViewerWorkspaceGraph.cpp:165-197`
- test for full vector preservation: `tests/test_viewer_graph_runtime.cpp:13-41`

Expected observable behavior:

- full 2D vector survives request path
- runtime does not flatten 2D request back to scalar axis 0 when vector is valid

## Refactor Acceptance Checklist

- [ ] same effective parameter still yields no-op
- [ ] same node + changed effective parameter still eligible for self-edge
- [ ] raw / projected / actual remain separate in UI + runtime snapshot
- [ ] default scheduling still waits for PMG-centered phase gate
- [ ] runtime target still respects support and reachable box together
- [ ] completed transition count increments only on completed blend
- [ ] clear/reset semantics unchanged
- [ ] 2D request path preserves full vector

## Out of Scope

- No algorithm change
- No UI redesign
- No artifact schema change
- No metric contract change
