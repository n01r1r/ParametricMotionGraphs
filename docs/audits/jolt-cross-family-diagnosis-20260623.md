# Within-Hull Jolt Cross-Family Diagnosis — 2026-06-23

## Purpose

Isolate the root cause of non-smooth / jolting motion when `walk_2d` is
evaluated at arbitrary parameters inside its support hull, and decide whether it
is a PMG-structural limitation, a blend-code defect, or a data/authoring gap.
Then close the one code-side gap the diagnosis surfaced.

## Method

Two existing audits, no new metric code:

- `--audit-parameter-response` — per grid sample, root-trajectory descriptors
  (path length, average speed, signed curvature) over the support hull.
- `--audit-registration-phase-alignment` — pairwise pose-seam distance between
  authored examples, classified GOOD/NEUTRAL/BAD against the build thresholds.

Controlled experiment: `specs/demo_walk_2d_singlefamily.pmg_spec`, identical to
`demo_walk_2d_triangulated` with exactly one variable changed — the
cross-family `jogCurve` anchor removed, travel-speed axis kept via the
same-family `walkStraightTwiceAsFast` clip.

## Findings

Trajectory-level parameter response is smooth: speed climbs monotonically
(~9.9 -> ~21.9 units/s) along the travel axis, signed curvature monotone
(+0.099 -> -0.205) along the turn axis, all samples finite
(`PASS_PARAMETER_RESPONSE`). The jolt is therefore not a parameter-interpolation
discontinuity at the root/trajectory level.

Pairwise pose-seam distances:

| pair type | pose seam | class | safe |
|---|---|---|---|
| walk x walk | 2.6 - 6.5 | GOOD | yes |
| walk x fast-walk (2x speed) | 37 - 50 | GOOD | yes |
| anything x jogCurve | 217 - 246 | BAD | no |

The `jogCurve` anchor is 40-90x more pose-distant from the walk anchors than any
walk-family pair. Speed alone is innocent (the 2x-speed walk blends GOOD); gait
family is the discriminator. The blend path is sound (Slerp + sign-aligned
quaternion mean + per-example local-frame root deltas + time-warp engaged for
generated clips), and walk x walk / walk x fast-walk blends are GOOD.

Control group confirmation: with the jog anchor removed, all six authored pairs
classify `PASS_PAIR_PHASE_ALIGNMENT`. The `WARN_PHASE_MISMATCH_FOR_JOG` cases
disappear; only a separate, lesser `WARN_WEAK_CYCLE_SEAM` remains (present in
both specs, unrelated to the cross-family blend).

## Verdict

The within-hull jolt is a data/authoring gap: a single non-logically-similar
example (`jogCurve`) placed inside the walk motion space, violating the
parametric-synthesis precondition that nearby motions in a space look similar.
It is not PMG-structural and not a blend-code defect. The paper-faithful fix is
to keep jog in a separate node connected by a transition edge, not blended into
the walk space (`demo_walk_2d_singlefamily.pmg_spec`).

## Code gap closed

`--audit-registration-phase-alignment` classified pairs by metric per row
(`WARN_PAIR_NOT_SAFE` for any BAD pair), but the top-level rollup only escalated
`WARN_PHASE_MISMATCH_FOR_JOG` — i.e. BAD **and** a path whose filename contains
the substring "jog". An identically-bad cross-family clip under any other name
was recorded per row yet masked at the summary level.

Fix (`apps/PmgDiagnosticCommands.cpp`, `BuildRegistrationPhaseAlignmentAudit`
rollup): also escalate `WARN_PAIR_NOT_SAFE`, making the summary signal
metric-driven instead of filename-coupled. The jog-specific label is retained.

A/B verification (same clip, filename only changed):

| spec | before | after |
|---|---|---|
| cross-family, neutral filename | `WARN_WEAK_CYCLE_SEAM` (masked) | `WARN_PAIR_NOT_SAFE` (surfaced) |
| cross-family, filename has "jog" | `WARN_PHASE_MISMATCH_FOR_JOG` | `WARN_PHASE_MISMATCH_FOR_JOG` |
| single-family control | `WARN_WEAK_CYCLE_SEAM` | `WARN_WEAK_CYCLE_SEAM` (no false positive) |

## Out of scope / residual

- Removing the jolt itself (drop the jog anchor, or promote jog to its own node)
  is a data/authoring decision, not made here.
- `cli_audit_transition_acceptance_consistency_walk_2d` fails
  (`FAIL_OVERREACH_REMAINS`, accepted-bad-after-gate 3). Confirmed pre-existing
  on clean `main` (stash + rebuild reproduces it); a separate edge-box overreach
  concern, independent of this change.
- The phase-alignment audit has no CMake test; the metric-decoupling was
  verified by the A/B above rather than a committed cross-family fixture BVH.
- `walk_2d` declares `registration ... -` (no cycle joint), so
  `--audit-cyclic-continuity` returns zero rows; loop-seam continuity across the
  hull is currently unmeasured for this node.
