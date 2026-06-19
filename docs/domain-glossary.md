# Domain glossary

## Purpose

Canonical vocabulary for code, tests, CLI text, and docs. Parameter vectors have
shape `[parameter_dimension]`; axis units come from each motion-space definition.
Normalized phase is dimensionless in `[0, 1]`.

| Term | Definition | Invariant / boundary |
|---|---|---|
| parametric motion space | Structurally compatible example motions indexed by a continuous parameter | One graph node owns one space; all examples share parameter dimension and skeleton/contact structure |
| node | Graph state containing one parametric motion space | Node index is graph-local; cyclicity is metadata, not implied by topology |
| edge | Directed connection from source node to target node | Stores zero or more sampled transitions; direction matters |
| sampled transition | Offline record at one source parameter | Contains reachable target box, normalized source/target phases, distance, and optional target-dependent phase samples |
| source parameter | Parameter vector where edge feasibility was sampled or queried | Shape matches source space parameter dimension |
| target bounding box | Axis-aligned reachable region in target parameter coordinates | Min/max shapes match target dimension; empty box is not reachable |
| transition point | Directional phase pair identifying first source support frame and last target support frame | Both phases normalized; alignment transform is not stored with it |
| runtime stream | Time-ordered world-space poses emitted by runtime controller | Clip-local motion plus accumulated floor-plane rigid transform remains continuous across accepted transitions |
| phase gate | Runtime condition that schedules transition when active phase crosses stored source transition phase | Handles normalized-phase wrap; placement follows configured transition-window convention |
| registration | Offline temporal correspondence among examples inside one motion space | Canonical phase maps to each example phase; endpoints remain pinned at 0 and 1 |
| alignment | Runtime rigid floor-plane placement of target clip against source clip | Maps target clip-local coordinates into source/world placement; recomputed per scheduled transition |

## Usage checklist

- Prefer `parameter`, not coordinate/value, when referring to a motion-space input.
- Prefer `target bounding box` in prose; `ParameterAabb` is code type.
- Prefer `transition point` for phase pair; reserve `transition window` for sampled frame span.
- Never use registration and alignment as synonyms.

