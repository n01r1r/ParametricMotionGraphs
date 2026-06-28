# Goto auto-navigation: cycle-latency + projection diagnosis (2026-06-28)

Analysis only — no code change. Hand-off for a later session that decides whether
to fix goto steering. Builds on `docs/audits/goto-arrival-ease-20260623.md`
(speed-ease P0, already landed: `arrival_speed_distance` 3.0 → 18.0).

## Symptom

In the viewer GraphRuntime "walk to target" mode (tested on `demo_walk_2d`, which
has a `turn_rate` axis), the character orbits the target and never enters
tolerance — it gets "stuck" circling.

## Root cause: BOTH effects, compounding

### 1. Projection (out-of-hull) — real, secondary

Each frame `UpdateGotoSteering(pose)` asks the core steering for a
`(turn_rate, travel_speed)` parameter. In `RuntimeController::TryScheduleTransition`
(`src/RuntimeController.cpp:340-345`) that request is snapped onto the node's
authored support:

```
effective = HasExplicitParameterSupport()
              ? ProjectToSupport(request.desired_parameter)
              : ClampToDomain(request.desired_parameter);
```

A turn sharper than the node spans is projected to the achievable max → the
character under-turns → it cannot make a corner tighter than its minimum turning
radius. Additionally `:346-349` drops the request entirely when the projected
parameter ≈ the current one, so sub-quantization corrections never apply.

### 2. Cycle-latency / stale request — real, the main "stuck" culprit

`current_parameter_` changes in only two places: `:89` (Start) and `:146`
(transition complete). There is **no continuous parameter update**. Even a
same-node turn nudge routes through the node's **self-edge** transition, which is:

- phase-gated by `edge_phase_range` (e.g. `0.70 0.95 …`), and
- requires `edge.LookupInterpolated(...)` to succeed (`:367-374`), then
- blends over `blend_seconds` before `next_parameter_` becomes `current_parameter_`.

So the steering request is recomputed every frame but is only **applied at the
self-edge phase window, ~once per cycle**. Between windows the character is
committed to the previous `turn_rate` and follows that arc. By the time the next
window opens it may have already passed the target, so the correction is computed
from a stale-relative pose and turns the other way → oscillation → orbit → never
within tolerance.

The arrival speed-ease (`distance / arrival_speed_distance`,
`GoalDirectedLocomotion.cpp:342-349`) helps — slower near the goal means a tighter
radius — but it does not remove the per-cycle granularity. Close to the target the
heading error changes fast (a near target subtends a large angle), so a
one-cycle-old commitment overshoots.

## Weighting

- Cause 2 (cycle-latency) dominates the "stuck/orbit" behaviour.
- Cause 1 (projection) adds the "can't physically turn tight enough" floor.
- Both originate in the same phase-gated self-edge machinery, so they compound.

## Levers (not implemented; for the fix session)

- **P1 (real lever for cause 2):** decouple a same-node parameter nudge from the
  gated self-edge — apply it as a continuous re-blend — or widen the self-edge
  phase window so corrections land more often than once per cycle.
- **Lookahead:** aim at a point ahead of the target to compensate the latency
  (paper future-work, line ~530).
- **Cause 1:** keep steering requests inside the node hull, or author
  sharper-turn / slower-walk examples to shrink the min radius.
- **Geometry fallback:** when the target is inside the min turning circle,
  stop-and-reorient or widen arrival tolerance.

## Note on Style-A

`specs/styleA_walk_strut.pmg_spec` nodes declare `parameter_metrics travel_speed`
only — **no `turn_rate` axis** (`spanned_axes 0`). With no heading axis the
steering has nothing to command; every request projects to a single point, so
goto cannot steer there at all. Style-A is a straight-line style-switch demo;
goto demos need a `turn_rate` node like `demo_walk_2d`.

## Resolution (2026-06-28, implemented + measured)

Both candidate levers for cause 2 were built behind opt-in config (default 0 =
original behavior, all 50 tests still green) and measured on `demo_walk_2d` via
`pmg_cli --goto`:

- **Tried first — widen the gate (option 3):** new same-node gate phase
  tolerance. Reach improved a lot on behind targets (min_dist 22.3 → ~6) but
  `pop_ratio` ~tripled (1.5 → 3–5, 100–320 transitions of off-gate blend churn)
  and the behind targets still missed `tol 4`. Off-gate firing pops because the
  blend starts away from the stored source reference frame. **Reverted** — bad
  tradeoff, not faithful.
- **Kept — continuous in-node tracking (option 1, the fix):**
  `RuntimeControllerConfig::same_node_tracking_rate` (CLI `--same-node-track R`).
  A same-node request lerps `current_parameter_` toward the projected target by
  `R` each `Update` and regenerates the clip preserving phase
  (`RuntimeController::TrackSameNodeParameter`), bypassing the gated self-edge
  entirely. No splice, no alignment seam → no transition pop.

Measured (rate 0.1, `--tolerance 4`):

| target | baseline | tracking 0.1 |
| --- | --- | --- |
| (0,-40) | reach @80s, pop 1.87 | reach, pop 2.31, **0 transitions** |
| (-30,-30) | **FAIL** (min 11.7) | **reach** 3.95, **pop 1.33** |
| (-35,-10) | FAIL (min 11.7) | min 5.8 — still missed |
| (0,40) | reach, pop 1.79 | reach, **pop 1.49** |
| (30,0) | reach, pop 1.81 | reach, pop 2.04 |

Tracking reaches behind targets baseline misses, with `transitions=0` (no
self-edge churn) and pop comparable-or-better. The `(-35,-10)` residual is the
min-radius geometry near the target (cause 1 / the Dubins floor), not latency —
that one needs the **lookahead** lever, not tracking. Test:
`tests/test_runtime_controller.cpp` asserts a same-node request converges inside
one cycle with zero completed transitions and no pose teleport.

**Remaining (viewer wiring, needs GUI verify):** the viewer still installs the
runtime with `same_node_tracking_rate = 0`, so the goto demo there still orbits.
Wire it scoped to goto only — add a setter so tracking turns on when
`goto_active_` flips (lines ~708/744, `PmgViewerWorkspace.cpp`) and off
otherwise — rather than baking it into `StartGraphRuntimeController` for all
graph playback (that would change the verified slider-drag transition UX).
