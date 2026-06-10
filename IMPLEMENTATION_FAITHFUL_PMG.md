# Faithful Parametric Motion Graph Implementation Notes

This implementation follows Heck & Gleicher's PMG structure:

- Nodes hold smooth parametric motion spaces.
- Edges are built offline by sampling source and target parameter spaces.
- For each source parameter sample, target samples are classified GOOD / NEUTRAL / BAD with a point-cloud distance grid.
- GOOD target samples form an axis-aligned reachable parameter box.
- BAD target samples conservatively shrink the box out of the transition region.
- Each edge sample stores only source parameter, reachable target box, and averaged normalized source/target transition phases.
- Runtime lookup uses k-nearest source samples and interpolates boxes/phases.
- Runtime alignment is recomputed at scheduling time rather than stored in the graph.

Changes made in this generated version:

1. Added `ParameterDomain` to centralize valid continuous node domains and sampling.
2. Updated `ParametricMotionSpace` to use local k-nearest blend weights rather than global inverse-distance blending.
3. Updated edge sampling to draw from the explicit node domain abstraction.
4. Made the OpenGL viewer opt-in (`PMG_BUILD_VIEWER=OFF`) so default builds work in headless CI/sandbox environments.
5. Verified the full test suite with default CMake configuration.

Validation command:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Result at generation time: 14/14 tests passed.
