# Goto Arrival-Ease Fix — 2026-06-23

## Phase

Discontinuity-fix track, phase 6 (cause #6: goal-directed `--goto` overshoot).
Application/controller-level fix; no graph, blend, or transition-metric change.

## Problem

`GoalDirectedLocomotionConfig::arrival_speed_distance` defaulted to `3.0` native
units. A travel-speed axis cruises at its fastest pace until within that distance
of the goal, then eases to its slowest so the controller settles instead of
overshooting. But one walk cycle covers ~10-22 native units, and the cruise
turning radius (cruise speed / max turn rate) is ~16 units, so the character was
never within 3 units except in passing. It therefore kept full cruise speed and
**orbited the goal at its turning radius** instead of spiralling in.

Reproduction (`--goto demo_walk_2d 10 10 --seconds 40 --tolerance 2.0 --trace`,
old default): the commanded parameter is pinned at `(1, 0.75)` (max turn, max
speed) and the distance-to-goal oscillates 3 -> 26 -> 10 -> ... The run only
grazes tolerance at t=24.7 s on a lucky close orbit pass; it never settles.

The CTest `cli_goto_walk_2d` did not catch this: it passed no `--tolerance`, so
the reached-assertion was skipped and only popping was checked.

## Fix

1. `arrival_speed_distance` default `3.0 -> 18.0` (just above the ~16-unit cruise
   turning radius, so easing begins ~one turning radius out and the path spirals
   in). Documented as a per-corpus calibration knob in native units.
2. New `pmg_cli --goto --arrival-distance D` override so the knob is tunable per
   corpus / per run without a rebuild (config field was already present).
3. `cli_goto_walk_2d` now passes `--tolerance 2.0`, asserting the goal is reached
   within `--seconds 20`. The old default reached only at ~24.7 s (> 20 s), so
   this test now fails on the old behavior and passes on the new.

## Evidence

Arrival-distance sweep, target (10,10), 40 s, tolerance 2.0:

| arrival_speed_distance | reached | reached_at_s |
|---|---|---|
| 3 (old) | yes (orbit graze) | 24.7 |
| 12 | no | - |
| 16 | yes | 5.3 |
| 18 | yes | 5.3 |
| 20 | yes | 8.4 |

Reach below the ~16-unit turning radius is unreliable; at/above it the path
spirals in. Forward/side targets improve strictly, e.g. target (20,0):
48.7 s (d=3) -> 19.5 s (d=18) -> 5.3 s (d=45).

## Scope boundary

This fixes the arrival/overshoot orbit only. Targets *behind* the start that
require a tight turn-around (e.g. (-12,8), (-15,-15)) are not reached at any
arrival distance (3-60): that is the separate min-turning-radius geometry limit
(cruise turning radius ~16 units vs the maneuver needed), not the arrival ease.
It is out of scope here and unchanged.

## Verification

- `ctest -C Release`: 49/49 (cli_goto_walk_2d now asserts reaching).
- No change to graph build, transition metric, blend weights, or the canonical
  spec; accepted-BAD unaffected.
