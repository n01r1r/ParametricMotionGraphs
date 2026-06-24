# §6 Source-Range Restriction (opt-in) — 2026-06-24

## Phase

Paper-faithfulness track. Closes the one genuinely-unimplemented paper item:
Heck & Gleicher 2007 §6's remedy for partially-compatible edges. Everything else
in the graph-build/lookup scope was already faithful (§3.2 target box-shrink,
§3.1 metric, §4 lookup); see the edge/lookup audit.

## Gap

`PmgBuilder::BuildEdgeWithReport` builds an edge by sampling source parameters
and, for each, finding the GOOD-transition target box. The rule was
**all-or-nothing**: the moment one sampled source parameter could not reach the
target (no GOOD hit, a BAD target left inside the reachable box, or an empty
box), the whole edge was rejected via early return. A wide source range against
a narrow target therefore yielded *no edge at all*, even when a sub-range of the
source was perfectly transition-compatible.

The paper names this exact limitation in §6 and its own remedy is to **restrict
the source range to the transition-compatible sub-range** rather than drop the
edge. The prior code comment mislabeled the all-or-nothing rule as itself
"§6-faithful"; the all-or-nothing rule is the *unrestricted* baseline, and the
restriction is the paper's fix on top of it.

## Decision

Implemented as an **opt-in ablation flag**, not a behavior change — matching the
existing `self_edge_cyclic_metric` pattern, because the shipped specs, their
calibrated thresholds, and the cross-family rejection guard all currently rely
on the all-or-nothing default.

- `PmgBuilderConfig::restrict_source_range` (default `false`).
- CLI: `--restrict-source-range` (build-graph), like `--self-edge-cyclic-metric`.
- `BuildEdgeWithReport`: the three source-sample reject points (no-GOOD,
  BAD-in-box, empty-box) become **skip-this-source-sample-and-continue** when the
  flag is set; legacy early-return is byte-for-byte unchanged when it is off.
- The edge is still rejected only if *no* compatible source sample remains
  (`edge_created = !samples.empty()` already enforces this), so a
  fully-incompatible pair is not rescued.
- No per-edge `.pmg_spec` syntax (YAGNI — the global flag enables the tests and
  the CLI; per-edge authoring can come later if a real spec needs it).

The restriction keeps the transition-compatible *subset* of sampled source
parameters; it does **not** guarantee a contiguous interval (sampling is
random + example-anchored), and the code/comments do not claim otherwise.

## Verification

- 49/49 ctest green; default-off path leaves every existing spec/edge build
  identical (existing graph-spec / builder / io tests unchanged).
- New builder tests (`tests/test_pmg_builder.cpp`, D1 block):
  - **flag off** — the wide source (|p|~1 extremes unreachable) still rejects the
    whole edge (preserved D1 all-or-nothing contract).
  - **flag on** — the same wide source builds an edge over the compatible
    samples: some source reports `accepted`, some restricted-out, and
    `max |p|` over accepted `< min |p|` over dropped (clean small-|p| subset,
    since distance grows monotonically with |p|). Kept-sample count equals the
    edge's transition-sample count.
  - **flag on, fully incompatible** — the disconnected pair still yields
    `edge_created == false` when every source sample is skipped.

## Residual / not in scope

- Per-edge spec syntax for the restriction (deferred; global flag + CLI suffice).
- Contiguous-interval recovery (current restriction is a compatible *subset*,
  faithful to the random+anchored sampling the builder already uses).
