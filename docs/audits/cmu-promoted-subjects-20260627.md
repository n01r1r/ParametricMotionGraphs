# CMU promoted within-subject gait graphs (2026-06-27, session 2)

Follow-on to `cmu-corpus-within-subject-sweep-20260627.md`. Promotes additional
within-subject 2-node walk/run graphs and records the extractor utility that made
window curation fast. All data stays out of git (`experiments/cmu/bvh*/` gitignored);
only specs + this doc are committed.

A2 (stored transition-clip bridge edge) stays **closed NO-GO**. High build-time Kovar
D on a cross-gait edge is a topology/coverage **warning**, not a runtime verdict; runtime
quality is audited separately via `--random-walk` (pop_ratio / max_step).

## New extractor flag: `--min-window-speed S` (opt-in)

`--extract-candidate-windows` ranks by loop seam, which favors near-standing segments
(they loop perfectly at ~0 displacement). For locomotion you want forward travel.
`--min-window-speed S` drops every window whose travel speed
(`root_displacement / duration`, BVH native units/sec) is below `S`, then ranks the
survivors by loop seam as before. `S=0` (default) = legacy loop-seam-only ranking,
unchanged.

```
pmg_cli --extract-candidate-windows file.bvh --min-frames 90 --max-frames 180 \
    --stride 10 --top-k 8 --min-window-speed 30 \
    --output-md out.md --output-csv out.csv
```

Effect (subject 78, clip 78_03): without the flag the top 3 windows are 0.3-0.5 cm/s
standing segments; with `--min-window-speed 30` the top 3 are 45-48 cm/s locomotion at
the best available seam (3.8-5.5). Test: `tests/test_candidate_window_extractor.cpp`
(every surviving window clears the floor; an impossible floor empties the result).

## Subject 78 — PASS (run-heavy, walk band thin)

`experiments/cmu/cmu78_gait_graph.pmg_spec`, clips in `experiments/cmu/bvh78/`
(`78_27 78_30 78_29 78_10`, from `Sequence-076-080/78/Data`).

35 clips; many clean 40-76 cm/s run windows, fewer clean walk windows. Windows picked
with `--min-window-speed`, then best loop seam in each band:

| node | example | window | speed | extractor seam |
|------|---------|--------|-------|----------------|
| walk_78 | 78_27 | 195-284 | 21 | 5.6 |
| walk_78 | 78_30 | 495-644 | 34 | 3.4 |
| run_78  | 78_29 | 60-164  | 46 | 1.7 |
| run_78  | 78_10 | 15-104  | 66 | 8.4 |

Walk top (34) → run bottom (46) gap is clean (clearer gait separation than subj 127's
38/42 near-merge).

Calibrated thresholds from `--diagnose-graph-edge` (build point-cloud kovar D):

| edge | interior min-D | tgood/tbad | accepted sources | note |
|------|----------------|------------|------------------|------|
| walk→walk | 346-1394 | 1600/3200 | 6/6 | both ends GOOD; clean node |
| run→run | 422-887 | 1500/3500 | 6/6 | slow anchor 46 loose (min 422, median ~3808) |
| walk→run | ~1900 | 2200/4500 | 6/6 | moderate crossing |
| run→walk | 2126(slow)/4434(fast) | 2800/4800 | 2/6 | slow-run only; fast-run rejects |

`run→walk` keeps only the slow-run sources (46/48) — fast-run (66/62/64/65) is too far
from any walk pose (min-D 4434), so it drops out. This is the faithful paper §6
source-range restriction (same shape as subj16/subj127), not a tuning failure.

Build: `PMG_GRAPH_V13`, 2 nodes, 4 edges (samples 6/6/6/2). Cyclic-continuity warning on
`walk_78 [21]` (78_27, seam_ratio 16.3) — a build-time topology warning.
`--random-walk --seconds 20 --min-transitions 3`: **PASS, 24 transitions**, median_step
1.39, max_step 2.10, **pop_ratio 1.51** (runtime cleaner than subj127's 2.53 despite the
build warning — confirming build-D ≠ runtime pop).

## Not promoted this session (next candidates, recipe ready)

Subjects 35 and 102 (each ~30 loco clips per the sweep) remain. The recipe is now fast:

1. `cp /c/data/cmubvh/Sequence-*/<subj>/Data/<subj>_NN.bvh experiments/cmu/bvh<subj>/`
2. Classify per clip:
   `for f in bvh<subj>/*.bvh; do pmg_cli --extract-candidate-windows "$f" --min-frames 90
   --max-frames 150 --stride 15 --top-k 150 --min-window-speed 15 --output-md .. --output-csv ..;
   done`, read max travel speed + best-seam window per band from the CSVs.
3. Pick 2 walk (band 15-35) + 2 run (band 44-72) clean-seam spanning windows.
4. Write `cmu<subj>_gait_graph.pmg_spec` (copy subj78 as template), add `bvh<subj>/` to
   `.gitignore`.
5. `--validate-graph-spec` → `--diagnose-graph-edge` ×4 → set edge_config tgood/tbad from
   interior min-D → `--build-graph` → `--inspect-graph` → `--random-walk` (≥3 transitions).

Do NOT promote a subject whose run band lacks ≥2 clean spanning windows, whose windows
are mostly standing, or that needs cross-subject retargeting (out of scope).
