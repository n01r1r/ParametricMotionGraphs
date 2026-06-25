# Research: the two roots of PMG quality limits, and two faithful tweaks

Date: 2026-06-25. Status: B in progress, A designed. Branch: experiment/cmu-motion-space (research continues on a follow-up branch).

## Frame

Every motion-quality ceiling hit this session collapses to two roots:

- **R1 — no transition clips.** Edges are *synthesized* between cyclic node spaces; nothing uses a clip that *contains* a transition. So cross-gait edges are teleport-grade (best D ~10x self-edge). `specs/README.md` already admits it.
- **R2 — registration demands identical contact structure.** `RequireBlendableMotionFamily` / `BuildRegistrationWarps` (src/MotionRegistration.cpp:277,321) throw if examples differ in contact-anchor count. Registration anchors are positional: `anchor[k]` must correspond across examples. So clips with different footfall counts cannot co-register.

R2 is the deeper root. Evidence (measured 2026-06-25): the CMU *within-gait* walk space `16_31 (4 contact intervals)` ↔ `16_21 (3)` cannot contact-register, so its spec falls back to `registration ... LeftFoot - 3 0` (cycle-phase only, NO contact anchors). Result: blended mid-poses lose foot contacts (audit: mid-sweep contact coverage collapses to 1, `baseline_max_foot_slide=0.53`). R2 is why the density lever is *code-blocked* (can't add a walk variant with a different strike count), why foot-lock had to be a post-hoc band-aid, and a large part of why blends drift off-manifold.

## Tweak B (recommended, first): contact-structure-tolerant registration

**Hypothesis.** Aligning differing-structure examples so corresponding contacts co-occur in the blend reduces foot-slide AT SOURCE (pre-foot-lock) and raises mid-blend contact coverage.

**Algorithm (role-tagged anchor matching, deterministic — no fragile DTW).** `ContactInterval` carries `joint_index`. For each example build role-tagged anchors `(joint_index, {STRIKE,LIFT}, phase)`. Group by `(joint, event)`; within a group anchors are ordered by phase. The common structure keeps, per key, `min_over_examples(count)` anchors (first k by phase), matched positionally. Concatenate matched anchors per example -> equal-length lists -> existing `BuildRegistrationWarps`. Unmatched anchors (the extra strike in the 4-interval clip) are dropped from the *registration* set; the clip still blends, just isn't pinned at that extra contact.

**Faithfulness.** Improves the paper's own §3 registration stage; no new paradigm. Opt-in flag (e.g. registration `tolerant`/a `0|1|2` mode column, or a builder `tolerant_contact_registration`), defaulting OFF so existing behavior is byte-identical; the ablation pattern of `self_edge_cyclic_metric` / `restrict_source_range`.

**Baseline (already measured).** `experiments/cmu/cmu_walk_1d.pmg_spec` (16_31↔16_21, 4-vs-3): no contact registration; `--audit-foot-skate` pre-foot-lock `max_foot_slide=0.53`, mid-sweep contacts ->1.

**Evaluator / significance bar.** `--audit-foot-skate` *baseline* (pre-foot-lock) `max_foot_slide` and min mid-sweep contact coverage on the 16_31↔16_21 space, with tolerant contact registration ON vs the current no-contact-reg. Significant = a clear foot-slide drop AND contacts no longer collapsing to 1 mid-blend. (Foot-lock then stacks on top, but the point is to fix it at source.)

## Tweak A (second): transition-clip-seeded edges

**Idea.** Let an edge be seeded/validated by a clip that *contains* the transition (`walkToJog` is in the corpus, unused as a seed; CMU 16_08 is run->stop). Turns gait-graph teleports into real planted transitions. This is Kovar motion-graph territory bolted onto PMG nodes -> less "pure PMG tweak" than B, so it goes second.

**Sketch.** A spec `transition_edge <src> <tgt> <clip> [window]`: register the clip's start against the source node space and its end against the target node space; emit an edge whose transition sample is anchored on the real clip rather than a synthesized blend point. Evaluator: random-walk `pop_ratio` across the seeded edge vs the synthesized/teleport edge.

## Not worth it (per the faithful-first + significance bar)
C1 calibration interpolant (gain < noise floor), linear->geodesic blend (departs from faithful PMG), 1-D polyline support (YAGNI), full motion-matching (different system, production-only).
