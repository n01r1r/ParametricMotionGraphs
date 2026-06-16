# Documentation Map

## Purpose

This directory explains the two source papers and the implementation as one
system. It separates:

- behavior stated by the papers;
- behavior implemented in this repository;
- local engineering extensions;
- known deviations and unimplemented paper machinery.

The local paper files are:

- [`../mograph.pdf`](../mograph.pdf): Kovar, Gleicher, and Pighin, *Motion
  Graphs* (2002).
- [`../PMGFullPaper.pdf`](../PMGFullPaper.pdf): Heck and Gleicher, *Parametric
  Motion Graphs* (2007).

The PDFs are source material, not build inputs.

## Recommended Reading Order

1. [`PAPER_GUIDE.md`](PAPER_GUIDE.md) - what each paper contributes and how
   they fit together.
2. [`PAPER_CODE_MAP.md`](PAPER_CODE_MAP.md) - paper sections and equations
   mapped to code and tests.
3. [`PAPER_CONFORMANCE.md`](PAPER_CONFORMANCE.md) - line-by-line conformance
   audit, the deviation list (D1-D8), what's left, and the claim limit.
4. [`SPEC_AUDIT.md`](SPEC_AUDIT.md) - semantic audit of every `.pmg_spec`,
   including node/edge validity, intended role, and visualization expectations.
5. [`DESIGN.md`](DESIGN.md) - module structure, offline/online pipeline,
   contracts, and failure boundaries.
6. [`REPRODUCTION.md`](REPRODUCTION.md) - build, validation, artifact, and
   runtime commands.

## Supporting References

- [`../CONTEXT.md`](../CONTEXT.md) - canonical project vocabulary and the
  paper-symbol map; code and conversation use these terms exactly.
- [`MOTION_CORPUS.md`](MOTION_CORPUS.md) - BVH skeleton-compatibility groups,
  which clips can blend together, and the minimal 2-D calibration fixture.
- [`WALK_JOG_CONTINUITY.md`](WALK_JOG_CONTINUITY.md) - measured loop-seam
  analysis, paper comparison, jog-domain limits, and desired-parameter
  semantics.
- [`experiments/CYCLIC_CONTINUITY_AUDIT.md`](experiments/CYCLIC_CONTINUITY_AUDIT.md)
  - tracked conclusion from the cyclic seam audit; generated CSV/markdown lives
  under ignored `outputs/cyclic_continuity/`.
- [`adr/`](adr/) - accepted implementation decisions.

## Evidence Hierarchy

When documents disagree, use this order:

1. Current source code and tests for actual repository behavior.
2. Local PDFs for paper claims and equations.
3. Generated artifact metadata for one concrete build.
4. Design documents for intended architecture.

Documentation does not prove motion quality or scientific equivalence. The
tests validate explicit software contracts on synthetic cases and the included
BVH corpus.
