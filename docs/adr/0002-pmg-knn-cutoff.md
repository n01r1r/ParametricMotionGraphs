# ADR-0002: PMG Equation 2 Cutoff — k-th Nearest Neighbor

## Status

Accepted — verified against the primary sources (2026-06-13). Supersedes the
earlier "accepted with documented ambiguity" state.

## Context

PMG parametric synthesis interpolates an edge's stored transition samples with a
k-nearest-neighbor weighting; the unnormalized weight uses a reciprocal-distance
cutoff. The local conference PDF (Heck & Gleicher 2007) prints the cutoff as the
distance to the k-th nearest sample, under which the k-th (farthest selected)
neighbor receives zero weight. Figure 5's caption — "weighted average of the
2-nearest neighbors" — reads as if both neighbors contribute, which appeared to
conflict with the printed equation. At the original audit an authoritative copy
of the cited primary source was unavailable, so the convention stayed unverified.

## Verification (2026-06-13)

Resolved against the authoritative expanded source: Rachel Heck, *Automated
Authoring of Quality Human Motion for Interactive Environments*, PhD thesis,
University of Wisconsin–Madison, Chapter 6, pp. 104–105
(`https://pages.cs.wisc.edu/~heckr/Thesis/6-PMGs.pdf`). The thesis prints the
same scheme explicitly:

- k-nearest neighbors with **k = (parameter dimensions) + 1**, ordered closest
  to farthest as l₁ … l_k (p. 104).
- Eq. 6.1: `wᵢ = w′ᵢ / Σⱼ w′ⱼ`.
- Eq. 6.2: `w′ᵢ = 1/ε(l̃, lᵢ) − 1/ε(l̃, l_k)`, where ε is Euclidean distance and
  **l_k is the k-th (farthest selected) neighbor**.

So the cutoff is the k-th neighbor (not k+1), and `w′_k = 1/ε_k − 1/ε_k = 0`:
the farthest selected neighbor always receives zero weight. In general `dim`
neighbors carry positive weight and the `(dim+1)`-th sets the cutoff radius. The
thesis attributes the scheme to Allen et al. [ACP02] (k-NN interpolation for
skinned body deformation) and Buehler et al. [BBM+01] (unstructured lumigraph
rendering), and chooses it over a least-squares linear map specifically because
it avoids large negative weights and needs no global optimization (p. 105).

Figure 5 / thesis Fig. 6.3 are reconciled: "weighted average of the 2-nearest
neighbors" is literally a weighted average over the `k = 2` selected neighbors in
which the farther one's weight vanishes at the cutoff. It is schematic, not a
claim that both carry nonzero weight. The printed equation is authoritative.

## Decision

Keep the literal scheme: `k = dimension + 1`, cutoff = k-th neighbor distance,
with explicit handling of exact/equidistant degeneracies (which the source does
not specify). `PmgEdge::LookupInterpolated` in `src/ParametricMotionGraph.cpp`
implements exactly this (`weight = 1/distance − 1/cutoff_distance`, cutoff =
distance to the `(dim+1)`-th nearest sample, clamped at zero, then normalized)
and is confirmed faithful; no code change is required.

## Consequences

- The implementation is verified against the primary sources, not just the
  printed glyph — the earlier ambiguity is closed.
- In one dimension the second selected neighbor receives zero weight, so a 1-D
  lookup reduces to the nearest sample. This is the source scheme's own behavior,
  not an implementation artifact.
- Any future move to a `(k+1)` support radius would deviate from the source and
  still requires a new ADR plus comparative evidence and threshold
  re-calibration.

## References

- Heck, *PhD thesis*, Ch. 6, pp. 104–105 (Eqs. 6.1–6.2).
- Heck & Gleicher, "Parametric Motion Graphs," ACM SIGGRAPH Symposium on
  Interactive 3D Graphics and Games (I3D), 2007.
- [ACP02] Allen, Curless, Popović, "Articulated Body Deformation from Range Scan
  Data," SIGGRAPH 2002.
- [BBM+01] Buehler, Bosse, McMillan, Gortler, Cohen, "Unstructured Lumigraph
  Rendering," SIGGRAPH 2001.
