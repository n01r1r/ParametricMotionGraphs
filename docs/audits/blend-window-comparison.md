# Blend Window Comparison

## Purpose

Compare fixed 5- and 8-frame transition windows under identical `120/234`
thresholds. Reports use 30 fps artifacts. No runtime policy changed.

## Results

| Metric | 5 frames | 8 frames |
|---|---:|---:|
| Blend duration | 0.167 s | 0.267 s |
| Montage accepted requests | 2512 / 2756 | 1842 / 2756 |
| Montage rejected requests | 244 | 914 |
| Rejected jog/walk requests | 159 | 221 |
| Worst accepted D | 245.926 | 356.268 |
| Worst root jump | 2.380 | 4.592 |
| Worst heading jump | 0.385 rad | 0.315 rad |
| Worst velocity jump | 10.999 units/s | 8.811 units/s |
| Contact rows with mismatch | 1752 / 2495 | 1554 / 1842 |
| Max contact-foot velocity | 25.490 units/s | 24.800 units/s |
| Max skate distance | 1.956 units | 2.285 units |
| Reachable-region conclusion | `WARN_SOURCE_DEPENDENT_REGION_SHRINKAGE` | `FAIL_THRESHOLD_TOO_LOOSE` |
| Montage conclusion | `FAIL_VISIBLE_TRANSITION_POP` | `FAIL_VISIBLE_TRANSITION_POP` |

## Decision

`KEEP_BLEND_5`

Eight frames adds 0.1 s response latency, rejects 670 more sampled requests,
retains worse accepted transitions, and increases worst skate distance. It does
not resolve visible-pop or contact-mismatch findings. Adaptive blending remains
unimplemented because current evidence does not justify a policy change.

## Artifacts

- `build/blend_window_5/{walk_2d.pmg,montage.md,contact.csv,contact.md,reachable.csv,reachable.md}`
- `build/blend_window_8/{walk_2d.pmg,montage.md,contact.csv,contact.md,reachable.csv,reachable.md}`
