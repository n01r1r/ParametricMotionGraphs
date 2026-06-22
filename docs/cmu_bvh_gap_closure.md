# CMU BVH corpus gap-closure audit

## Purpose

This corpus audit is a data-gating step for PMG gap closure. It does not reproduce the papers Section 5 results by itself. It identifies compatible BVH subsets that may support second-node, inter-node-edge, and graph-walk experiments.

## What this audit does

Parses external BVH files; records skeleton, channels, timing, root trajectory; groups compatible files; emits conservative heuristic candidates. Unreliable metrics remain `null` or `unknown`.

## What this audit does not claim

No motion-space suitability, transition quality, or Section 5 reproduction claim. No download, copy, trimming, normalization, or PMG construction.

## Relation to PMG gaps

Current demo is single-node `walk_2d`. Audit gates compatible run/action selection before second-node and inter-node work.

## External corpus and command

Obtain `cmubvh` separately. Keep corpus outside this repository.

```sh
pmg_cli audit-corpus --corpus-root /path/to/cmubvh --out outputs/cmu_audit
```

Optional: `--max-files N`, `--include "**/*.bvh"`, `--category-hints`, `--verbose`.

## Output files

`manifest.json`, `manifest.csv`, `skeleton_groups.json`, `candidate_groups.json`, `gap_closure_report.md`.

## Pass/fail gates

PASS requires reviewed structural, motion, contact, and window evidence. Parsed path-hint matches are SOFT PASS. Parse failures are FAIL. Missing evidence is UNKNOWN.

## Recommended next steps

Review shared-skeleton candidates; annotate windows; compare with `walk_2d`; then generate `run_1d` spec and bidirectional edge experiments. Build multi-node PMG only after evidence passes.
