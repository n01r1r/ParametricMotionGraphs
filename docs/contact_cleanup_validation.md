# Runtime contact cleanup and refactor validation

## Problem and implementation

The live viewer previously sent the raw `RuntimeController` pose straight to
rendering. The Foot-lock checkbox affected only the generated preview clip.
Root motion, pose interpolation, and cross-fades therefore had no final
world-space footplant constraint. The offline IK also preserved each frame's
foot orientation, allowing rotation during a supposed plant.

The revised path is:

1. Infer per-ankle contacts on each source example. Transfer labels through the
   same time warps and blend weights used to synthesize the clip (weighted
   contact vote >= 0.5). This avoids declaring a skidding synthesized foot
   airborne merely because the synthesis introduced velocity.
2. Advance and align the graph as before. During a transition, the dominant
   clip supplies contact labels; an uninterrupted contact retains its anchor.
3. Apply `StreamingFootLock` to the final world-space pose. Retain position and
   orientation from contact acquisition, adjust the pelvis for leg reach,
   and solve two-bone IK without changing bone lengths.
4. Ease the correction out over 0.1 seconds after lift-off. Reset anchors on
   restart/disable and after a time jump over one second. Contact processing
   substeps normal updates at the clip rate, capped at 240 steps per update.
5. Cache the corrected pose. Reading snapshots does not advance cleanup state.

A second, independent source of visible skating was display scaling around
the moving per-frame skeleton centroid. At a non-unit skeleton scale, even a
stationary native foot acquired the centroid's velocity. `SceneCoordinates`
now applies one fixed world transform to joints, trajectories, markers, and
click targets, including the inverse recenter/scale mapping for picking. A
regression checks planted-point invariance and native/display round trips.

The viewer owns the skeleton used by its alignment strategy, cannot be copied
or moved with internal references still bound to an old object, and rejects
malformed requests. Its playback adapter is a separate `pmg_viewer_runtime`
target rather than part of the OpenGL workspace source list.

The core keeps cleanup opt-in via `RuntimeController::EnableFootLocking`;
the viewer connects it to the existing enabled-by-default Foot-lock control.
The preview cleanup also holds the strike-frame foot orientation.

## Reproduction and metric

Build with `PMG_BUILD_TESTS=ON`, then run:

```sh
ctest --test-dir build-core -C Release --output-on-failure
ctest --test-dir build-core -C Release -V -R 'test_(cmu_runtime_contacts|streaming_foot_lock)'
```

`test_cmu_runtime_contacts` builds both complete bundled gait graphs, runs each
for 1,200 samples at 60 Hz, requests run for the first half and walk for the
second, and compares raw/corrected distance to the **same active world anchors**.
For each frame it takes the maximum active ankle error, then reports the RMS
and maximum across contact-bearing frames. Units are the BVH's native units.
This is a constraint-enforcement metric, not independent ground-truth contact
accuracy, a ground-penetration measurement, or a perceptual quality score.

GCC 13.3 Release results:

| Graph | Contact-bearing frames | Completed transitions | Raw RMS | Corrected RMS | Raw max | Corrected max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| CMU subject 16 | 686 / 1200 | 3 | 0.647170 | 0.000062521 | 2.98687 | 0.000993158 |
| CMU subject 78 | 526 / 1200 | 14 | 1.615170 | 0.000096922 | 4.15640 | 0.000566979 |

Maximum pelvis corrections were 0.789275 and 0 native units, respectively.
A separate registered walk-space sweep required up to 3.58197 units of pelvis
correction; it is not represented by the two rows above. Its maximum ankle
anchor error was 0.000412116 across 496 contact-bearing frames out of 600.
The synthetic 0.295-unit sliding-foot case had maximum corrected error
0.00000430806 and preserved foot orientation and bone lengths.

Local Release: **35/35 CTest executables passed**, plus a separately compiled
viewer-runtime adapter regression. All 35 core tests also passed with
AddressSanitizer/UndefinedBehaviorSanitizer (33 via CTest; two damaged local
executables were relinked and run separately). Leak detection was disabled
locally because LeakSanitizer cannot inspect `/proc` in this environment;
CI retains its normal leak checking. Modified viewer translation units passed
GCC syntax checks against the pinned GLM/ImGui/GLFW/GLEW headers. A full OpenGL
viewer build and visual playback could not run here: OpenGL development
libraries are absent and the environment does not permit system package
installation. Windows viewer build/test CI is provided for that integration.


## Other fixes and regression coverage

- Restored 31 core tests from `dev/core`; replaced dependencies on unavailable
  Center clips with bundled CMU data. Omitted the test for continuous parameter
  tracking, whose API is intentionally absent from this distribution. Replaced
  exact float-zero comparison with a numerical tolerance and made initializer
  types explicit for GCC. Added input validation, streaming contact, complete
  CMU runtime, and viewer-adapter regressions.
- Connected `PMG_BUILD_TESTS` to CTest; checks also execute in Release builds.
- Shared finite/rate/frame-count validation and scale-invariant weight
  normalization. Reject NaN before floor-to-index conversion and reject
  unrepresentable frame counts before integer conversion/allocation.
- Reject empty/reversed parameter boxes before `std::clamp`, non-finite
  parameters, negative dimensions, and invalid skeleton parent order.
- BVH reads consume complete numeric tokens, check motion-value counts before
  allocation, and reject non-root translation channels the pose representation
  cannot preserve. Zero-channel joints are accepted; duplicate channels are not.
- Artifact numeric parsing no longer retries a partially consumed float token;
  unexpected trailing data and failed output flushes are reported.
- Runtime start stores the projected parameter that actually generated the
  clip. Cycle folding uses logarithmic transform composition instead of one
  iteration per skipped cycle. Completed clips are moved, not copied.
- Window speed uses `(frame_count - 1) / fps`, consistent with clip duration.
- Two-bone IK reach margins scale with limb length and validate parent indices.

## Limits and visual acceptance

This is **kinematic contact cleanup**, not a physical contact simulator. The
solver has no masses, forces, friction cones, dynamic balance, or terrain
collision. It preserves ankle height at acquisition; ankle origins are not
assumed to lie on the rendered floor. It does not reproduce the full Kovar
solver, which includes heel/ball constraints, neighborhood filtering and
limited bone stretching. Root-reach projection here has a bounded iteration
count; incompatible constraints can retain error. Knee popping and pelvis
smoothness remain visual acceptance criteria, especially near full extension.

Source contact labels remain heuristic. Sparse motion support, inaccurate
registration, missing transition motion, and airborne phases cannot be repaired
by an ankle lock. In-place preview intentionally removes root travel and is not
a ground-contact locomotion mode. The raw mode remains available for comparison.

Before merging, run the viewer with each bundled gait graph and compare the
Foot-lock toggle: steady walk, steady run, speed changes, cross-gait transitions,
restart, pause/resume, and turns. Check heel/toe slide, knee pops, pelvis snaps,
and floor penetration in addition to the displayed anchor error. A small
anchor error alone does not establish natural or physically valid animation.

## References

- Kovar, Schreiner, Gleicher (2002), *Footskate Cleanup for Motion Capture Editing*:
  https://graphics.cs.wisc.edu/Papers/2002/KSG02/
- Authors' implementation and integration notes:
  https://research.cs.wisc.edu/graphics/Gallery/FootskateSolver/

These support the separation of contact constraints, IK, root reach, and
boundary smoothing. No third-party solver code was copied into this change.
