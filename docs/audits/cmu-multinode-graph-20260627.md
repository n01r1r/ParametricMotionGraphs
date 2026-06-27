# CMU multi-node PMG-core graph — reproduction audit (2026-06-27)

Closes the repo's **multi-node PMG-core demo gap**: a reproducible CMU-BVH graph
with ≥2 nodes, self-edges, bidirectional inter-node edges, and runtime traversal.
This is *not* a Section 5 full-corpus reproduction (corpus size, graph size, and
visual result set are out of scope).

A2 (stored transition-clip bridge) stays **closed NO-GO**. No bridge / transition_edge
/ SetEdgeBridge / `--bridge-ab` / `--manifold-probe` code was introduced. The lever
here is graph density / multi-node topology from compatible same-subject clips.

## Target

`experiments/cmu/cmu_gait_graph.pmg_spec` — subject-16 locomotion, 2 nodes, each
dim=1 `travel_speed`:

- `walk_cmu`: 16_31 @17.1, 16_21 @29.9
- `run_cmu` : 16_36 @47.1, 16_45 @69.9

Edges: walk→walk, run→run (self), walk→run, run→walk (cross, §6 source-range
restriction ON by default).

## Commands

```bash
# data (gitignored, regenerable)
cd experiments/cmu && bash fetch.sh 16 && cd ../..

CLI=build/Release/pmg_cli.exe
$CLI --validate-graph-spec experiments/cmu/cmu_gait_graph.pmg_spec
$CLI --diagnose-graph-edge experiments/cmu/cmu_gait_graph.pmg_spec walk_cmu walk_cmu
$CLI --diagnose-graph-edge experiments/cmu/cmu_gait_graph.pmg_spec run_cmu  run_cmu
$CLI --diagnose-graph-edge experiments/cmu/cmu_gait_graph.pmg_spec walk_cmu run_cmu
$CLI --diagnose-graph-edge experiments/cmu/cmu_gait_graph.pmg_spec run_cmu  walk_cmu
$CLI --build-graph   experiments/cmu/cmu_gait_graph.pmg_spec outputs/cmu_gait_graph/cmu_gait_graph.pmg
$CLI --inspect-graph outputs/cmu_gait_graph/cmu_gait_graph.pmg
$CLI --random-walk   outputs/cmu_gait_graph/cmu_gait_graph.pmg --seconds 20 --min-transitions 3 \
     --dump-motion-csv       outputs/cmu_gait_graph/random_walk_motion.csv \
     --dump-transitions-csv  outputs/cmu_gait_graph/random_walk_transitions.csv
```

## Results — PASS

| Gate | Result |
|------|--------|
| validate-graph-spec | PASS — 2 nodes, skeleton joints=38 compatible, 4 edges requested |
| diagnose (all 4 edges) | `edge_created=yes` on all 4 |
| build-graph | `PMG_GRAPH_V13`, nodes=2 edges=4, generated_frame_count=48 @30fps |
| inspect-graph (reload) | nodes=2; edges 0→0(6), 1→1(6), 0→1(6), 1→0(2) |
| random-walk | RESULT=PASS, frames=601, **transitions=35** (≥3), median_step=1.04 |
| deliverables | report.md, config.json, metrics.json, tables/edge_samples.csv, tables/cyclic_samples.csv |

### Edge detail (diagnose D, native cm²)

- **walk→walk**: D median 116–206 across 6 sources, all accepted, GOOD-dominated. Clean.
- **run→run**: D median 293–868; fast-run sources (≥65) carry 1 BAD each (16_45 weak
  yaw seam) but all accepted. Reliable.
- **walk→run**: all 6 sources accepted; best D=2709 (slow-walk source) — cross-gait wall.
  Source-range box restricts to the transition-compatible (fast-walk) end.
- **run→walk**: 2/6 sources accepted (slow-run 47.1 / 50.1); fast-run sources rejected
  (`no GOOD target samples`). This is §6 restriction working — keeps only the
  transition-compatible source end. Hence edge `1→0 samples=2` in inspect.

## Known limitations (faithful, not hidden)

- **Cross-gait edges are teleport-grade**, not clean planted transitions. Best
  walk→run D≈2709 / run→walk D≈1932 is ~10× the self-edge (~250). Root cause:
  subject 16 has no walk↔run transition clip. High build-time Kovar D is a
  topology/coverage **warning**, not automatically a runtime pop. A2 bridge was
  spiked and closed NO-GO (did not beat runtime cross-fade). Fix is corpus
  density/coverage (a real transition clip), not a bridge edge or threshold tuning.
- **Cyclic continuity**: 4/4 cyclic samples flagged weak (worst: run_cmu 16_45,
  yaw_ratio 7.15). Anchor strides aren't phase-locked; quality note, not a topology
  failure. random-walk pop_ratio=2.46 (cross-gait edges dominate).
- Blending CMU *into* the Credo "Center" hull still needs retargeting (out of scope).
- Subject 86 (multi-family) and full-corpus `audit-corpus` mining are deferred 2nd-stage.

## Fixed en route

- `experiments/cmu/cmu_gait_graph.pmg_spec` + `README.md`: removed stale "faithful
  fix = bridge clip" wording; replaced with A2 NO-GO verdict + density-is-the-lever.
- README "KNOWN BUG: dim=1 reload" marked **FIXED** — root cause was non-finite
  float parse (`ReadSerializedFloat`, `src/GraphIo.cpp`), regression covered by
  `CheckDim1RoundTrip` (`tests/test_graph_io.cpp`). Confirmed in practice:
  `--inspect-graph` reloads the 1-D artifact cleanly.
