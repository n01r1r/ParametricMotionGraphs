# ADR-0002: Preserve the Printed PMG Equation 2 Cutoff

## Status

Accepted with documented ambiguity

## Context

PMG Section 4 defines the unnormalized neighbor weight using the distance to
the k-th nearest sample as the reciprocal cutoff. Under the printed equation,
the k-th neighbor receives zero weight. Figure 5 describes a weighted average
of two nearest neighbors, which visually suggests both contribute. The local
PDF and text extraction agree on the printed equation. A public authoritative
copy of the Allen et al. source was not available during this audit, so its
original interpolation convention remains independently unverified.

## Decision

Implement the literal PMG equation: use `k = dimension + 1`, use the k-th
neighbor distance as cutoff, and explicitly handle exact/equidistant
degeneracies. Keep a focused regression test for this interpretation.

## Consequences

- The implementation is reproducible against the printed formula.
- In one dimension, the second selected neighbor can receive zero weight.
- Changing to a k+1 support radius requires a new ADR and comparative evidence.
