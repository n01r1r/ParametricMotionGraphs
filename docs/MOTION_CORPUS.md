# Motion Corpus & Skeleton Compatibility

The `BVH/` corpus is not one homogeneous skeleton. Clips can only be blended
into the same parametric motion space when their skeletons are compatible;
mixing incompatible clips is rejected at build time.

## What "compatible" means

`pmg::CheckSkeletonCompatibility` (`src/SkeletonCompatibility.cpp`) requires, for
every joint: equal **name**, equal **parent index**, equal **channel
convention**, and **bone offset** within `1e-4`. Adding a clip to a space (viewer
"Add motion sample", or a multi-`example` spec node) runs this check against the
first clip and rejects on the first mismatch.

Every clip in `BVH/` shares the same joint *names* (24 named joints; the loader
also counts 7 `End Site` leaves, so artifacts report 31 joints). They split into
groups only by **bone offset** — i.e. different actor proportions.

## Skeleton groups (by bone offset)

| Group | Clips |
|-------|-------|
| **A** | `WalkLoopA`, `StaggerA`, `WalkStartA`, `WalkStopA`, `SneakLoopA`, `StrutLoopA`, `TightRopeA`, `AboutFaceA`, and the action set (`BackKickA`, `BigKick`*, `SlapA`, `SledgeHammerA`, `SneezeA`, `SpinBackKickA`, `SpinKickHiA`, `StubToeA`, `SwingAxeA`, `TantrumA`, `TripA`, `UpAndAwayA`, `VaultA`, `WaveA`, `WhipA`, `YawnStretchA`) |
| **B** | `walkCurve`, `walkMoreCurve`, `walkTightCurve`, `walkSpiral`, `walkFastSpiral`, `walkStraightTwiceAsFast`, `jogCurve`, `walkToJog` |
| **C** | `smoothWalk`, `walkReallySmooth` |
| **D** | `standStill`, `standReallyStill`, `walkCurvedLong` |
| singletons | `DNCMODRNA`+`DNCMODRNAA`, `BigKick`, `faked_sneak1`, `j_Uber_054_SMK_CHANG1_01` |

(*`BigKick` carries its own offset; it does not join Group A.)

Membership was derived by hashing each file's `OFFSET` block. Re-derive after any
corpus change:

```bash
for f in BVH/*.bvh; do
  sig=$(awk 'tolower($1)=="offset"{print $2,$3,$4} /MOTION/{exit}' "$f" | md5sum | cut -c1-8)
  echo "$sig $(basename "$f")"
done | sort
```

## Picking clips for a parametric space

- **Locomotion (turn × speed) spaces → Group B.** It is the only group with
  systematic curvature and speed variation (gentle→tight curves, walk→jog), so a
  2-D steering space is buildable entirely within it. `WalkLoopA` is **not** in
  Group B and cannot be added to such a space.
- **Walk + style/actions → Group A.** `WalkLoopA` only blends with Group A. That
  group has no clean steady turn/speed loops (`AboutFaceA` is a discrete turn
  maneuver, not a continuous parameter), so a Group-A motion space is a *gait
  style* space, not a metric-parameterized steering space.
- **Do not import CMU/cmubvh clips into an existing space.** They are yet another
  skeleton; they would form their own group and be rejected by every group above.
  They are only usable as a self-consistent standalone corpus, and only after the
  loader, contact-joint heuristics, and units are verified for that skeleton.

## Minimal 2-D fixture: `specs/walk_curvature_speed.pmg_spec`

A calibrated, reproducible 2-D Group-B fixture:

- axis 0 = turn (wide → tight), axis 1 = gait (walk → jog);
- examples `walkCurve` @ (0,0), `walkTightCurve` @ (1,0), `jogCurve` @ (0,1) —
  three non-collinear samples, enough to exercise 2-D interpolation and
  calibration;
- `parameter_metrics turn_rate travel_speed` + `parameter_calibration` give the
  axes measured physical meaning (unlike an uncalibrated viewer-authored space).

This is a triangular minimal sample set, not a robust rectangular motion
family. The jog anchor changes both turn rate and speed, and no `(1,1)`
tight-jog example exists. Calibration measures and inverts the generated sparse
blend; it does not create missing motion support or prove axis independence.
See [`SPEC_AUDIT.md`](SPEC_AUDIT.md).

Build and inspect:

```bash
pmg_cli --build-graph specs/walk_curvature_speed.pmg_spec outputs/walk_2d.pmg
pmg_cli --inspect-graph outputs/walk_2d.pmg
```

Then load `outputs/walk_2d.pmg` from the viewer's PMG Runtime tab (or pass it as
the launch argument). Example paths inside a `.pmg_spec` are resolved **relative
to the spec file's own directory**, which is why the specs reference `../BVH/...`.
