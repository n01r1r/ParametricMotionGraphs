# Style-A walk↔strut: clean same-rig multi-node graph (2026-06-28)

## Result

A faithful same-skeleton **two-node** PMG-style graph,
`specs/styleA_walk_strut.pmg_spec`, builds cleanly and traverses without
explosion. This is the first multi-node result that is simultaneously same-rig,
multi-node, clean transition distance, local data (no fetch), and a visually
meaningful style change.

## Data

All clips are the "A" mocap set, one skeleton (full-OFFSET md5 **4343a921**,
31 joints). No retargeting.

| node | clip | window | loop-seam score |
|------|------|--------|-----------------|
| walk_A  | WalkLoopA.bvh  | 0–30 | 0.214 (ok) |
| strut_A | StrutLoopA.bvh | 0–41 | 0.048 (clean) |

Cycle joint `LeftAnkle`. Each node uses two same-clip windows to satisfy the
`>=2`-example simplex rule; `spanned_axes 0` (this is an edge/topology demo, not
a within-node parametric span).

## Commands

```
pmg_cli --validate-graph-spec specs/styleA_walk_strut.pmg_spec
pmg_cli --build-graph specs/styleA_walk_strut.pmg_spec out/styleA_walk_strut.pmg
pmg_cli --diagnose-graph-edge specs/styleA_walk_strut.pmg_spec walk_A strut_A
pmg_cli --diagnose-graph-edge specs/styleA_walk_strut.pmg_spec strut_A walk_A
pmg_cli --random-walk specs/styleA_walk_strut.pmg_spec --seconds 10 --min-transitions 3 --walk-seed 7
```

## Edge acceptance

Build: `nodes=2 edges=4`, skeleton `compatible=yes`, no offset mismatch.

| edge | accepted | counts |
|------|----------|--------|
| walk_A → walk_A | self (built) | — |
| strut_A → strut_A | self (built) | — |
| walk_A → strut_A | **6/6** | GOOD=8 every sample |
| strut_A → walk_A | **6/6** | GOOD=8 every sample |

No §6 source-range drops (contrast the N=3 sneak attempt below).

## Transition distance

Raw min Kovar D (`--dump-distance-grid`), all far below the CMU subj16
cross-gait wall (D~2709), near the walk self-edge (~39):

| edge | diagnose D (min … median) |
|------|---------------------------|
| walk_A → strut_A | 103.8 … 104.0 |
| strut_A → walk_A | 29.7 … 39.5 |

## Runtime smoke (`--random-walk`, CLI proxy)

`frames=301 transitions=8 pop_ratio=2.60 RESULT=PASS`, zero quality-gate
skips. Traversal does not explode. pop_ratio 2.6 is moderate — a real same-rig
style change with no recorded bridge clip — acceptable for a short demo, not
zero. (GUI viewer visual smoke test still TODO: load spec, confirm the walk↔strut
style switch is legible and foot-skate is tolerable for a 5–8 s recording.)

## Why this over Center stand→walk→jog

The Center ("Credo") corpus splits stand and jog across different skeletons —
`specs/credo_stand_walk_jog.pmg_spec` fails the strict joint-offset gate at
joint[4]. The Style-A set keeps multiple cyclic gait styles on ONE rig, so the
cross-style transition is near-free and the graph builds without retargeting.

## Why N=3 (walk/strut/sneak) stays appendix

`specs/styleA_walk_strut_sneak.pmg_spec` builds `nodes=3 edges=6` but is
degraded: SneakLoopA's loop seam is weak (seam_ratio ~10, extractor score
0.856) and three sneak cross-edges drop under the §6 source-range restriction.
This rig has only three cyclic loop clips and no stronger third style, so a
clean N=3 is data-limited. N=3 remains future work / audit evidence, not a
release path.

## Report claim upgrade

From: "Graph infrastructure supports transitions; clean multi-node visual demo
is limited." To: "A same-rig Style-A walk↔strut graph builds cleanly and
demonstrates a real multi-node PMG-style transition between two cyclic motion
spaces." A stronger three-node graph remains data-limited (weaker third-style
loop quality).
