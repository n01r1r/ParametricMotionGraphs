# Transition Montage B1 Evidence (2026-06-19)

Tracked summary of `build/transition_montage_report.md`.

## Purpose

Version failing Phase B1 evidence before any root-cause-only audit work.

## Result

- Conclusion: `FAIL_VISIBLE_TRANSITION_POP`
- Evaluations: `2756`
- Accepted / rejected requests: `2512 / 244`
- Rejected jog/walk requests: `159`

## Top failing accepted case

- Source: `(0.1875, 0.375)`
- Requested target: `(0.025, 0.875)`
- Effective target: `(0.025, 0.875)`
- `D`: `245.926`
- Class: `BAD`
- Root jump: `2.37989`
- Heading jump: `0.385308`
- Velocity jump: `4.50446`

## Top accepted near-BAD cluster

- `(0.075, 0.875) -> (0.5, 0.0)`: `D=226.800`, jog/walk pair
- `(0.075, 0.875) -> (0.5125, 0.0)`: `D=226.358`, jog/walk pair
- `(0.075, 0.875) -> (0.5125, 0.125)`: `D=225.636`, jog/walk pair
- `(0.075, 0.875) -> (0.35, 0.0)`: `D=219.246`, jog/walk pair

## Why this file exists

Phase B1 proved visible failure exists under locked `TGOOD/TBAD = 120/234`.
Phase B1.5 now audits why accepted region logic still admits at least one
`BAD` transition without changing thresholds or runtime semantics.
