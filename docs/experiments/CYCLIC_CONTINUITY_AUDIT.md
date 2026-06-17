# Cyclic Continuity Audit

Last updated: 2026-06-17.

## Purpose

Measure whether current cyclic clips and generated cyclic motion spaces are
locally continuous enough to support PMG cyclic self-edge streaming. This is a
corpus-quality diagnostic only. It does not change PMG edge construction,
runtime scheduling, blending, thresholds, or specs, and it does not claim
perceptual smoothness.

## Inputs

- `specs/demo_walk_self_edge_minimal.pmg_spec`
- `specs/demo_walk_jog_topology.pmg_spec`
- Command outputs:
  - `outputs/cyclic_continuity/walk_self_edge_cyclic.csv`
  - `outputs/cyclic_continuity/walk_self_edge_cyclic.md`
  - `outputs/cyclic_continuity/walk_jog_cyclic.csv`
  - `outputs/cyclic_continuity/walk_jog_cyclic.md`

The output directory is ignored by git, so this document preserves the tracked
conclusion.

## What Was Measured

The audit treats the cyclic seam as the transition from the last frame of a
clip to the first frame of the next cycle after applying the same
`ComputeCycleDelta` used by `RuntimeController` cycle folding.

For each cyclic node, it measures authored cycle clips and generated production
clips at endpoint/midpoint samples:

- mean joint seam distance and ratio to median in-clip adjacent step;
- pre, seam, and post root speed in native units per second;
- pre, seam, and post root yaw rate in radians per second;
- foot drift across the seam and first/last contact-state agreement when
  contact metadata exists.

Root/yaw seam rates are raw diagnostic fields only. `root_speed_ratio` compares
pre-seam and post-seam in-clip root speeds; `yaw_rate_ratio` compares pre-seam
and post-seam signed in-clip yaw rates with the configured deadband. They do
not divide by the seam rates, which are near zero under `ComputeCycleDelta`.

## Results

### `demo_walk_self_edge_minimal`

| Node | Samples | Strong | Weak pose | Weak root speed | Weak yaw rate | Weak contact |
|---|---:|---:|---:|---:|---:|---:|
| walk | 5 | 0 | 4 | 0 | 1 | 0 |

The tight authored endpoint (`parameter=1`) is closest by pose seam ratio
(`1.96991`) but still classifies weak by yaw-rate ratio (`4.74796`). The wide
authored endpoint (`parameter=0`) classifies as `weak_pose_seam`: seam ratio
`2.67087`, root-speed ratio `1.34537`, yaw-rate ratio `3.30683`, max contact
drift `1.68869`.

Generated walk samples are not substantially worse than authored anchors. Their
pose seam ratios (`2.07276` to `2.88604`) stay in the same range as authored
walk (`1.96991` to `2.67087`).

### `demo_walk_jog_topology`

| Node | Samples | Strong | Weak pose | Weak root speed | Weak yaw rate | Weak contact |
|---|---:|---:|---:|---:|---:|---:|
| walk | 6 | 0 | 5 | 0 | 1 | 0 |
| jog | 2 | 0 | 0 | 0 | 2 | 0 |

Walk remains weak across authored and generated samples. The middle authored
walk anchor (`parameter=0.5`, `walkMoreCurve`) had the largest walk seam ratio:
`2.73931`, with root-speed ratio `1.34553`.

Jog has a smaller pose seam (`0.433885`) and finite root-speed ratio
(`1.01675`). It classifies as `weak_yaw_rate` because its pre/post in-clip yaw
rates differ under the current yaw-rate diagnostic, not because seam yaw is
near zero.

## Interpretation

Current cyclic anchors are not cyclic-strong under this local seam diagnostic.
Generated production clips inherit the same seam weaknesses; registration and
parameter-space generation do not remove them.

Current self-edge residuals are primarily a cyclic seam and corpus-quality
problem:

- **Cyclic seam problem:** yes. Seam pose/contact drift is above threshold on
  walk anchors. The raw seam root/yaw rates collapse to near zero under the
  endpoint-to-endpoint cycle-delta contract, but those seam rates are diagnostic
  fields and no longer drive root/yaw ratios.
- **Transition window problem:** still relevant to runtime residuals, but a
  window sweep should not be the next fix before seam/corpus quality is improved.
- **Registration problem:** not primary in this audit. Generated registered
  samples remain weak in the same pattern as authored anchors.
- **Corpus coverage problem:** yes. Walk has only sparse curvature anchors, and
  jog is a singleton. There is no cyclic-strong replacement set in the current
  tracked specs.

## Recommendations

Do not change specs in this PR.

Anchor replacement or recutting is recommended before further PMG complexity:

- replace or recut walk cyclic anchors so first/last pose, contact state, and
  local velocities agree under `ComputeCycleDelta`;
- add more compatible jog anchors before treating jog as a production cyclic
  controller node;
- keep transition-quality gates diagnostic-only.

Further PMG core, window, or threshold changes are not justified by this audit.
The next improvement should be walk clip/corpus curation. Tuning around input
seam defects could hide corpus issues without making cyclic anchors strong.

## Feedback Loop Added 2026-06-17

The cyclic audit is now surfaced at artifact consumption points, not only as a
separate offline command:

- `pmg_cli --build-graph` prints a cyclic continuity warning when a built
  artifact contains weak cyclic samples;
- generated `metrics.json` records cyclic strong/weak sample counts;
- generated `report.md` records the same warning at build time;
- the viewer reports the warning when a `.pmg_spec` build or artifact load
  installs a graph.

This is an invertible diagnostic change. It does not modify BVH files, graph
specs, registration, edge thresholds, runtime scheduling, or transition
blending. Reverting the warning/report API and viewer/CLI call sites restores
the prior behavior without artifact migration.

## Checklist

- [x] Diagnostic-only module added.
- [x] Runtime cycle folding shares the same cycle-delta helper.
- [x] CSV and markdown outputs generated for both demo specs.
- [x] Debug build passed.
- [x] CTest passed.
- [x] No specs, edge thresholds, runtime scheduling, edge lookup, or blend logic
      changed.
- [x] Artifact build/load feedback loop added without changing runtime behavior.
