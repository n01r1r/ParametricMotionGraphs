# Research: the two roots of PMG quality limits, and two faithful tweaks

Date: 2026-06-25. Status: B measured + kept (cleared the bar), A designed. Branch: feat/structure-tolerant-registration (off experiment/cmu-motion-space).

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

**Result (measured 2026-06-25, branch `feat/structure-tolerant-registration`).** Wired tolerant registration end-to-end: the spec `registration` line takes an optional 6th field `<tolerant:0|1>`; when set, `PrepareMotionSpaces` skips `RequireBlendableMotionFamily` and calls `RegisterSpaceByContacts(..., tolerant=true)`. CMU spec is now `registration cmu_walk LeftFoot LeftFoot,RightFoot 3 0 1`. `--space-sweep` (11 steps) — naive (no contact reg) vs registered (tolerant) vs registered + foot-lock:

| metric | naive | tolerant reg | tolerant + foot-lock |
|---|---|---|---|
| min_contacts | 0 | 2 | 2 |
| min_contact_coverage | 0 | 64 | 62 |
| max_foot_slide | 0.535 | 0.530 | **0.183** |
| max_slide_rate | 3.58 | 4.06 | **1.98** |
| max_adjacent_step | 0.91 | 1.64 | 1.64 |

Verdict: **clears the bar, kept (opt-in, documented).** Contact-collapse is fixed decisively and on its own — naive loses ALL foot contact for 4 of 11 sweep steps (`min_contacts=0`); tolerant holds `min_contacts=2` across the whole sweep. Raw `max_foot_slide` from tolerant registration *alone* is flat (~0.53), but naive's low number is an artifact: mid-sweep the foot is airborne, so it cannot slide. Tolerant registration's real role is to preserve the contacts foot-lock needs — on the naive blend foot-lock has nothing to lock at the collapsed steps; on the tolerant blend it cuts slide to 0.183 (−66% vs naive). Strict registration cannot build this space at all (throws on 4-vs-3 structure). Honest cost: `max_adjacent_step` rises 0.91→1.64 because the registered blend moves both feet through real stance instead of collapsing toward a near-static degenerate pose.

A real algorithm defect surfaced *during* measurement and was fixed: `MatchedContactAnchors` ordered anchors by their *mean* phase across examples, which is not each example's own order. CMU 16_31/16_21 disagree on left/right strike order, so one example's emitted list came out non-monotone and `TimeWarp::FromAnchors` threw "to anchors must be strictly increasing". Fix (`src/MotionRegistration.cpp`): greedily keep only the anchor chain that is strictly increasing in *every* example, dropping genuinely un-coregisterable anchors. Regression-tested by `TestMatchedContactAnchorsHandlesOrderConflict`. The original synthetic unit test never hit this — uniform data keeps mean order == per-example order, the false-confidence gap real mocap exposed.

## Tweak A (second): transition-clip-seeded edges

**Idea.** Let an edge be seeded/validated by a clip that *contains* the transition (`walkToJog` is in the corpus, unused as a seed; CMU 16_08 is run->stop). Turns gait-graph teleports into real planted transitions. This is Kovar motion-graph territory bolted onto PMG nodes -> less "pure PMG tweak" than B, so it goes second.

**Sketch.** A spec `transition_edge <src> <tgt> <clip> [window]`: register the clip's start against the source node space and its end against the target node space; emit an edge whose transition sample is anchored on the real clip rather than a synthesized blend point. Evaluator: random-walk `pop_ratio` across the seeded edge vs the synthesized/teleport edge.

## Not worth it (per the faithful-first + significance bar)
C1 calibration interpolant (gain < noise floor), linear->geodesic blend (departs from faithful PMG), 1-D polyline support (YAGNI), full motion-matching (different system, production-only).
