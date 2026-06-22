# Threshold Default Decision (2026-06-19)

Tracked mirror of `build/threshold_default_decision.md`.

## Purpose

Phase A4 closes threshold selection for the current single-node triangulated
`walk_2d` PMG. Scope is decision only: either commit `120/234` as the new
default or keep it experimental. No runtime, metric, interpolation, or graph
topology change is made here.

## Inputs

- `build/transition_threshold_sweep.md`
- `build/threshold_visual_acceptance_t120_b234.md`
- `build/parameter_response_audit.md`
- `build/reachable_region_audit.md`

## Decision

`COMMIT_120_234_AS_DEFAULT`

## Why

- Threshold sweep recommends `120/234` as the loosest non-baseline pair that
  still rejects the high-distance jog/walk cases accepted by `300/400`.
- Worst accepted transition under `120/234` stays at `D=47.851` with
  `root jump=2.233`, `heading jump=0.131`, `velocity jump=10.768`.
- Loose `300/400` accepts `D=275.146` on `(0.0, 0.0) -> (0.0, 1.0)` with
  `heading jump=0.426` and `velocity jump=12.374`.
- `build/threshold_visual_acceptance_t120_b234.md` confirms build success and
  CLI reachability smoke.
- Manual viewer inspection was later completed and accepted. No new pop,
  delayed transition, heading discontinuity, or root-jump regression was
  reported versus the loose `300/400` baseline, while the stricter pair avoids
  the bad jog/walk cases identified in the threshold sweep.

## Current status

- Committed viewer/demo spec is `specs/demo_walk_2d_triangulated.pmg_spec`
  with `edge_config walk_2d walk_2d 120 234 3 6 41`.
- `build/walk_2d_triangulated_t120_b234.pmg` remains the matching built
  artifact for the accepted default.

## Next step

Proceed to Phase B1 transition montage audit.
