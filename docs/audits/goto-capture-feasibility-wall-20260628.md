# Goto capture feasibility wall: turning-radius vs tolerance (2026-06-28)

Analysis only — no code change. Hand-off documenting why `--goto` capture is not
robustly convergent on `demo_walk_2d`. Builds on
`docs/audits/goto-arrival-ease-20260623.md` (speed-ease P0, landed:
`arrival_speed_distance` 3.0 → 18.0) and
`docs/audits/goto-cycle-latency-20260628.md` (continuous in-node tracking, landed
`fa468ef`). Those removed the per-cycle latency; the residual miss documented here
is **kinematic, not a control bug**.

## Symptom

`pmg_cli --goto demo_walk_2d.pmg_spec x z --tolerance 4` reaches some targets and
misses others, and a sweep of `--arrival-distance` flips PASS/FAIL
**non-monotonically** (e.g. 18 PASS, 20 FAIL, 24 PASS), every pass landing within
~0.05–0.15 of the tolerance boundary. Looks like a tuning lottery.

## The wall: minimum turning radius exceeds the tolerance

The steering calibration is reported at the top of every `--goto` run
(`achieved[...]` lines). For `demo_walk_2d`:

```
omega_max = max achievable |turn rate|   = 1.835 rad/s   (turn_rate axis, param=1)
v_min     = slowest achievable speed     = 10.12 units/s (travel_speed axis, param=0)
r_min     = v_min / omega_max            ~= 5.5 native units
```

`r_min ≈ 5.5` is the tightest circle the character can physically walk: the
slowest example in the speed axis still travels 10.12 units/s, and the sharpest
turn is 1.835 rad/s. The success tolerance is **4 < 5.5**.

A point-target controller that has overshot the goal must circle back; the
smallest circle it can hold has radius `r_min`. So the closest it can stay to the
goal is bounded below by ~`r_min`, and a 4-unit disk sitting inside that circle is
**unreachable by any steering law on this corpus**. Measured orbit closest
approach (capture off, several targets) clusters at / above the wall:

| target      | min_distance |
| ----------- | -----------: |
| (14, -10)   |         4.99 |
| (20, -14)   |         4.59 |
| (16, 12)    |         2.94 |
| (-14, 11)   |        10.43 |

`min_distance ≈ 4.99` *is* the limit-cycle closest approach, not a near-miss.
(2.94 for (16,12) is a favourable spiral grazing inside; 10.43 for (-14,11) is a
separate swing-around failure, see below.) The `arrival_speed_distance` knob works
only by tightening the inward spiral (slower speed → smaller instantaneous
radius); near the wall whether the spiral grazes inside 4 depends on gait phase,
which is the non-monotonic lottery.

Note this is the **floor** turning radius (the radius the speed-ease settles to),
distinct from the ~16-unit **cruise** radius in
`goto-cycle-latency-20260628.md` / the min-radius memory: cruise radius bounds
wide-corner tracking, floor radius bounds final capture.

## Current goto success: phase-sensitive grazing, not convergence

Because the achievable closest approach (~`r_min` ≈ 5.5) exceeds the tolerance
(4), a PASS means the limit cycle's closest point happened to fall inside the disk
on a given gait phase. It is **phase-sensitive grazing, not robust convergence**.
Restating the prior conclusion sharpened by the numbers: goto "works, but not
robustly convergent," and the binding limit is kinematic (`r_min > tolerance`),
not the heading field.

## Tested and reverted: capture-region heading taper

Hypothesis (from the goto experiment write-up): make the heading law
capture-region-aware — taper turn authority to zero inside a capture disk so the
character coasts straight through the goal instead of orbiting. Implemented as an
opt-in `--capture-radius` (smoothstep taper on the commanded turn rate) and swept
on the orbiting target (14, -10), `--arrival-distance 16`, `--tolerance 4`:

| capture-radius | min_distance |
| -------------: | -----------: |
|              0 |         4.99 |
|            4–8 |         4.99 (never engages above r_min) |
|             10 |         5.00 |
|             12 |         5.17 |
|             16 |         5.28 |
|             20 |         5.58 |

Tapering heading **worsens** capture monotonically: with no turn authority near
the goal the character coasts straight *out* of the disk, increasing min_distance.
A principled variant (turning-radius speed cap `v ≤ k·omega_max·distance`, run the
tightest feasible spiral) compressed the `arrival_speed_distance` sensitivity but
still could not cross the `v_min` wall and grew its own `k` lottery (k=0.6 grazes
in at 3.99; k=0.7–0.9 miss). **Both prototypes were reverted; nothing shipped.**
A local heading-law tweak cannot move the wall.

## Robust sub-4 capture is a data problem, not a steering tweak

The wall is set by `v_min`. The only way a 4-unit disk becomes physically
reachable is to lower `r_min`, and with `omega_max` fixed by the corpus that means
lowering `v_min`:

```
add slower-walk / idle / stopping example
  -> speed axis reaches below ~10 units/s
  -> r_min = v_min / omega_max drops below the tolerance
  -> the 4-unit disk becomes physically reachable
```

This is a model-space / corpus change (new locomotion example to widen the speed
axis downward), not a code change. Until `v_min` drops, robust small-disk capture
is structurally out of reach regardless of steering.

## Demo tolerance (separate from the above)

If the goal is only a passing demo, setting the demo tolerance above the measured
`r_min` (≈ 6) makes capture robust for forward/side targets. This is **not an
algorithm improvement** — it is matching the success predicate to the dynamics
(`tolerance ≳ r_min`), not paper-faithfulness. Documented here so a later tolerance
bump is not mistaken for a steering fix.

### Wired for recording (2026-06-28)

Viewer `goto_tolerance_` was `2.0` — *below* `r_min`, so viewer goto could never
fire "Target reached" and orbited forever on camera. Bumped to `6.0`
(`PmgViewerWorkspace.h`, comment points back here). CLI default
`arrival_speed_distance = 18` is inherited by the viewer, so capture matches the
CLI sweep below.

CLI verification at `--tolerance 6 --arrival-distance 16` (all PASS, `min_distance`
clusters 5.6–5.95 = the `r_min` floor):

| target    | result | min_distance | reached_at |
| --------- | :----: | -----------: | ---------: |
| (12, 8)   |  PASS  |         5.83 |     4.9 s  |
| (14, 10)  |  PASS  |         5.62 |     5.2 s  |
| (14, -10) |  PASS  |         5.93 |     5.3 s  |
| (20, -14) |  PASS  |         5.84 |     5.7 s  |
| (16, 12)  |  PASS  |         5.91 |    14.5 s  |
| (18, 0)   |  PASS  |         5.83 |    28.4 s  |
| (22, -8)  |  PASS  |         5.83 |    61.1 s  |

For video use the fast forward/side set — `(12,8)`, `(14,10)`, `(14,-10)`,
`(20,-14)` (all ≤ 5.7 s, direct approach). Skip `(18,0)` / `(22,-8)`: they capture
but orbit 25–60 s first, which reads as failure on camera. `(-14,11)` still fails
(swing-around, §"Separate, larger failure") — do not target it.

## Separate, larger failure: swing-around targets

`target(-14, 11)` misses at `min_distance = 10.43` — far above the `r_min` wall, so
*not* explained by this feasibility limit. It is a distinct swing-around geometry
(the character loops wide and never re-approaches), likely in the
`swinging_long_way_` re-acquisition path. Recommended as the next goto
investigation, independent of capture feasibility.

## Repro

```
pmg_cli --goto specs/demo_walk_2d.pmg_spec 14 -10 --tolerance 4 \
        --arrival-distance 16 --seconds 80
# read omega_max / v_min from the achieved[axis=0 turn_rate ...] /
# achieved[axis=1 travel_speed ...] lines; min_distance / reached at the end.
```

## Status

- No code shipped this session (both heading-taper and speed-cap prototypes
  reverted; working tree clean; existing tests green).
- Deliverable = the falsification + the quantified wall (`r_min ≈ 5.5 > tol 4`).
- Open: (1) data-level slower/idle example to lower `v_min`; (2) `(-14, 11)`
  swing-around bug; (3) optional demo tolerance ≳ 6, labelled as predicate-matching
  not an algorithm change.
