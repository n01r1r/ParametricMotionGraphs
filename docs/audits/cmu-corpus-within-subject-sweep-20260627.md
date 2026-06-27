# CMU full-corpus within-subject all-pairs transition sweep (2026-06-27)

Answers: "test every possible clip-pair combination across all cmubvh data, excluding
cross-subject." Cross-subject excluded by construction (per-subject directory). No
new code — existing `--calibrate-thresholds <dir>` does the N×N sweep per directory.

## Scope / method

- Corpus: `github.com/Shriinivas/cmubvh` full clone, `/c/data/cmubvh`, 2.1 G, 2505 BVH.
- `audit-corpus` over all 2505 → **one** skeleton group (`b7bb8a3e6ac6c74f`, 38 joints,
  120 fps). Signature = names/parents/channels; per-subject bone *offsets* still differ,
  so the strict 1e-4 compat gate still blocks cross-subject (excluded here anyway).
  Path-based category heuristic found **0** action labels (CMU filenames are numeric).
- Within-subject sweep: for each `Sequence-XXX/<subj>/Data`, `--calibrate-thresholds`
  loads every clip once and computes `FindOptimalTransition` for all ordered pairs
  (phase-restricted grid, source 70–95% → target 5–30%, window 5, stride 2). Native
  weighted squared-sum units.
- Aggregated to `outputs/cmu_within_subject_sweep/all_pairs.csv`
  (`subject,source,target,type,distance`) + per-subject logs in `per_subject/`.

## Result

- **103 subjects** swept (8 had <2 clips), **94,417 ordered pairs** (91,920 cross-clip
  "diff" + self). Wall time ~81 min (long multi-thousand-frame clips dominate the O(F²) grid).
- diff-pair distance distribution: min 1.47, p10 34, p25 87.5, **median 250**, p75 885, max 100677.
- **59/103 subjects** have ≥1 GOOD (<20) cross-clip pair.

### Richest subjects (cross-clip diff pairs; better multi-node candidates than subj 16)

| subj | diff pairs | min | n<20 | n<100 |
|------|-----------|-----|------|-------|
| 69 | 5550 | 2.73 | 636 | 3539 |
| 80 | 5256 | 4.65 | 193 | 2692 |
| 79 | 9120 | 1.79 | 846 | 2675 |
| 91 / 105 | 3782 | 4.60 | 144 | 2067 |
| 36 | 1332 | 4.10 | 287 | 1293 |
| 83 / 122 | 4556 | 3.01 | 328 | 1108 |
| 16 | 3306 | 4.65 | 158 | 693 |

subj 16 (the built 2-node graph) is mid-pack — a valid but not optimal pick.

### Duplicate subjects (identical distance signatures = same data, renamed)

- **91 ≡ 105**, **83 ≡ 122**. Treat as one subject each; don't double-count.

## Caveats (faithful)

- Distances are **raw full-clip** transitions over the late→early phase window, **not
  cyclic-window recuts**. Coarse for multi-action clips: the metric measures end-of-A →
  start-of-B best pose pair, assuming clip = one cycle.
- Sub-4 distances are mostly **near-duplicate takes** (same action repeated), not
  interesting gait-change transitions. Genuine value = clips that *differ* yet transition
  cheaply — needs action labels to confirm (CMU labels live outside the BVH paths).
- This is a combination *screen*, not a graph. Building a real multi-node graph from a
  rich subject still needs the subject-16 manual pipeline: pick cyclic windows
  (`start_frame`/`end_frame`), set parameter axis, calibrate edge thresholds, `--build-graph`.
- A2 stays closed NO-GO; no bridge/transition_edge code touched.

## What was NOT done

- Cross-subject pairs (excluded per request; structurally blocked without retargeting).
- Cyclic-window recut sweep (no recut/bvh-export tool exists; windows usable only via
  spec `start_frame`/`end_frame` + `--build-graph`).
- Action labeling (no CMU label file ingested).

## CORRECTION: cheap-transition richness ≠ good graph subject

The cheap-transition ranking (69/80/79) is misleading. Stationary subjects (79/80,
mean_speed ≈0.2–1.1 cm/s, idle/in-place) and slow ones (69, max 16.9, no run) win it
because similar poses transition cheaply — but they make useless one-blob graphs. The
correct selector is **clip count in the locomotion speed band (mean_speed 15–90 cm/s)**:

| subj | loco-band | walk(15–35) | run(40–120) | verdict |
|------|-----------|-------------|-------------|---------|
| 16 | 47 | many | 47–70 | built ✓ (reference) |
| 127 | 36 | 14 | 22 (max 79) | **built ✓ this audit** |
| 78 | 33 | 9 | 18 (max 78) | usable 2-node |
| 35 | 33 | 23 | 10 (max 63) | usable 2-node |
| 138 | 32 | 32 | 0 | walk-only → 1-node |
| 102 | 31 | 9 | 17 (max 78) | usable 2-node |
| 104 | 27 | 23 | 4 | walk-heavy, weak run |
| 69/79/80 | ~0–11 | — | ~0 | NOT locomotion |

## Promotion result: subject 127 (2-node walk/run)

`experiments/cmu/cmu127_gait_graph.pmg_spec`, clips in `experiments/cmu/bvh127/`.
walk_127 (127_31 ~12, 127_22 ~38 cm/s) ↔ run_127 (127_27 ~42, 127_07 ~79 cm/s).
Built `PMG_GRAPH_V13`, 2 nodes, 4 edges; `--random-walk` PASS, 28 transitions.
Run node solid (D 850–2800); **walk node loose** — slow window 127_31@12 is weak-seam
(extractor favored a near-standing loop). Thresholds are placeholders, not calibrated.

## Window-curation finding (the real bottleneck)

`--extract-candidate-windows` ranks by loop-seam, which **favors stationary standing
segments** (they loop perfectly with ~0 displacement). For a locomotion example you must
pick by *displacement/duration* (window speed), accepting a higher seam. Several subj-127
"walk" clips returned 0.1–3 cm/s windows. A "best-travel window" extractor mode would
remove the manual step.

## Session 2 (2026-06-27 cont.): subj-127 calibrated, extractor fix, subj-78 promoted

1. **Subject 127 CALIBRATED (walk node stays loose by data limit).** Edge thresholds
   replaced placeholders → derived from `--diagnose-graph-edge` (build point-cloud kovar
   D): walk self 1900/3800, run self 1600/3200, walk→run 2700/4500, run→walk 2900/4500.
   Build/inspect/random-walk PASS (26 transitions, pop_ratio 2.53). The weak 127_31@12
   window was **not replaceable**: it is the *cleanest* slow-walk window in the subject
   (seam 0.81); the looseness is the node's 3× speed span (12→38), and (a) no clean
   intermediate-speed walk window exists in subject 127 (travel-rank: mid-band only at
   seam ~10 vs 0.8/5.6 at the endpoints), (b) a 1-D `simplex` node is capped at 2 examples
   so the axis can't be densified without a 2-D reparameterization. Subject 127 walk node
   remains **experimental**, not reference-quality. Not promoted to a separate cmu127 doc
   (subj16 stays the lone reference PASS).
2. **Extractor travel filter shipped.** `--extract-candidate-windows --min-window-speed S`
   drops near-standing windows (the real bottleneck this audit named) before seam ranking;
   `S=0` keeps legacy behavior. Tested. See `cmu-promoted-subjects-20260627.md`.
3. **Subject 78 promoted (PASS).** 2-node walk/run, calibrated, random-walk PASS (24
   transitions, pop_ratio 1.51 — cleaner runtime than subj127). Details in
   `cmu-promoted-subjects-20260627.md`.

## What remains / what to do more

1. ~~Calibrate subj-127~~ DONE (session 2). Walk node loose by data limit, documented.
2. Promote 35 / 102 (recipe now fast via `--min-window-speed`; see promoted-subjects doc).
   78 done.
3. Denser speed axis (3–4 examples/node) is **blocked**: 1-D `simplex` support = exactly
   2 vertices. Would need a new multi-vertex 1-D support or a 2-D reparameterization
   (out of scope; not worth a new support type for this).
4. Non-locomotion multi-action graphs (subject-86 style) need external CMU action labels
   — BVH paths are numeric, so no in-corpus labeling.
5. Cross-subject blending stays out of scope (needs retargeting).
6. A2 bridge stays closed NO-GO. High build-time Kovar D = topology/coverage warning;
   runtime quality audited separately (subj78 build-warns yet runtime pop_ratio is the
   best of the three graphs).
