# Runtime Foot-Skate Measurement — 2026-06-27

## Phase

Faithful-demo-video track, step 1 (evidence gate): does the *streamed* navigating
character skate enough to need a foot lock before recording? Report-only;
no graph, blend, transition-metric, controller, or viewer change.

## Problem

`LockFootContacts` was already applied to the viewer's static parametric *preview*
clip (commit `c1b4708`), but the runtime traversal path streams controller poses
raw — `RuntimeController::CurrentPose()` per frame, no clip buffer, no lock. So
any recorded *moving* character (goto target-reach, graph traversal) would show
stance foot-slide. Magnitude was unmeasured.

## Measurement

New report-only CLI flag `--report-foot-skate` on `--goto` and `--random-walk`
(`apps/PmgRuntimeCommands.cpp`). It builds a `MotionClip` from the captured
world-pose stream, detects contact intervals once, measures mean stance slide
(max world drift of each contact foot during its own contact interval), applies
`LockFootContacts`, and re-measures on the same intervals.

Contact joints come from the artifact registration; when that carries none
(cycle_joint only, as in the current walk_2d / CMU artifacts) it falls back to
the CLI `--contact-joints` names resolved against the skeleton.

## Evidence

| Run | intervals | slide before | slide after | cut | median step |
|---|---|---|---|---|---|
| `--goto demo_walk_2d 20 0 --arrival-distance 18` | 23 | 0.191 | 0.067 | −65% | 0.476 |
| `--random-walk cmu_gait_graph (subj 16) --contact-joints LeftFoot,RightFoot` | 36 | 0.630 | 0.109 | −83% | 1.431 |

Raw stance slide is 40–44% of the median per-frame joint motion — visible skate.
The post-process lock cuts it 65–83%, consistent with the −62% measured earlier
on the parametric preview. Both runs render `RESULT=PASS` (pop unaffected).

## Conclusion

A faithful moving-character video requires foot-lock on the streamed motion, not
only the static preview. Because `Pose` stores quaternions and there is no
`MotionClip`→BVH serializer, and the viewer already renders `Pose` directly and
already wires `LockFootContacts`, the cheapest render-faithful path is to lock
the navigation where it is rendered (viewer bake-and-lock playback) rather than
serialize a clip out. Approach decision tracked for the next step.

## Scope boundary

Measurement + a fallback contact-joint resolver only. The runtime controller and
viewer scheduling are unchanged; nothing is rendered or saved by this flag. The
regression anchor is the table above (deterministic; re-run the two commands).

## Verification

- `ctest -C Release`: 50/50.
- Both commands run green and print the slide table above.
