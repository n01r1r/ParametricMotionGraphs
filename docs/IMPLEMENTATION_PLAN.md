# PMG Implementation Plan

Goal: bring the existing Phase-1 scaffold to a faithful reproduction of
*Parametric Motion Graphs* (Heck & Gleicher), then add the paper's
applications and finally optional rendering / Phase-2 extensions.

Each phase lists **goal**, **subphase tasks**, and **tests** (extend the
existing `assert`-based executable test pattern; add to `CMakeLists.txt`).

Reference map to paper sections is in `docs/IMPLEMENTATION_PLAN.md` gap table
(see also README). Phases are ordered so each depends only on earlier ones.

> **Progress:**
> A ✅ · B ✅ · C ✅ (paper-faithful: empty-box reject, in-box average) ·
> D ✅ · E ✅ (point-cloud runtime alignment, C¹ blend, centered window) ·
> F 🟡 partial · G 🟡 partial · H ✅ viewer (heatmap + graph runtime) · I ⏸ deferred.
> `ctest` 9/9. D4/D6 documented intentional deviations. BVH loads in native units
> (display scale render-only).

---

## Phase A — Faithful similarity metric (paper §3.1)

**Goal:** replace single-frame, translation-only `PoseDistance` with the
Kovar et al. 2002 point-cloud metric used by the paper.

### A.1 Point cloud over a frame window
- Build a point cloud per transition candidate frame = joint world positions
  collected over a small window of `k` surrounding frames (config `window_size`).
- Add `BuildPointCloud(skeleton, clip, centerFrame, windowSize)` helper.

### A.2 Optimal 2D rigid alignment (translation on floor + yaw)
- Compute the closed-form optimal translation in the floor plane (x,z) and
  rotation about the vertical (y) axis that minimizes sum-of-squared distances
  between the two point clouds (Kovar'02 closed form).
- New `MotionDistance::AlignedPointCloudDistance(cloudA, cloudB)` returns
  distance **and** the recovered `(theta, dx, dz)` alignment transform.

### A.3 Optional weighting (velocity / per-joint)
- Add per-point weights (joint importance + finite-difference velocity term),
  configurable; default uniform to keep parity with current behavior.

### Tests `test_motion_distance.cpp`
- Identical clip → distance ≈ 0, alignment ≈ identity.
- Clip translated by (dx,dz) → distance ≈ 0, recovered translation ≈ (−dx,−dz).
- Clip rotated about y by θ → distance ≈ 0, recovered yaw ≈ −θ.
- Distinct poses → distance > 0 and symmetric `D(a,b)==D(b,a)`.

---

## Phase B — Optimal transition point via distance grid (paper §3.1, Fig 3)

**Goal:** find the optimal transition point as the minimum cell of the full
frame-pair distance grid, not a windowed sub-grid.

### B.1 Distance grid
- `MotionDistance::BuildDistanceGrid(skeleton, src, dst)` → 2D grid of aligned
  point-cloud distances over all (or strided) frame pairs.

### B.2 Optimal transition point + threshold gate
- Min cell = `(srcFrame, dstFrame, distance)`; expose normalized phases.
- Keep `TransitionSearchConfig` but interpret windows as a search restriction
  knob (default = full range) rather than the only sampling.

### Tests `test_distance_grid.cpp`
- Self-grid of a clip has near-zero band along the diagonal.
- Min cell of two known clips matches a brute-force reference loop.
- Normalized transition phases in [0,1].

---

## Phase C — Sampling-based transition region with double threshold (paper §3.2, Fig 4)

**Goal:** correct edge construction: random sampling + TGOOD/TBAD bounding-box
shrink (the dead `bad_transition_threshold` becomes live).

### C.1 Random parameter sampling
- `LSampleSource` (~50) over source space, `LTargetSamples` (~1000) over target
  space, seeded RNG for reproducibility (config `seed`, `source_sample_count`,
  `target_sample_count`).

### C.2 GOOD / BAD partition
- For each source sample: classify target samples into `L_GOOD` (D ≤ TGOOD) and
  `L_BAD` (D ≥ TBAD) using Phase-A/B metric.

### C.3 Conservative bounding-box shrink
- Bounding box of `L_GOOD`, then minimally shrink each dimension so **no**
  `L_BAD` sample lies inside (Fig 4c). Implement on `ParameterAabb`
  (`ShrinkToExclude(point, epsilon)`).
- Store per-sample: source parameter, shrunk target box, normalized transition
  point (avg of GOOD matches). Drop source sample if `L_GOOD` empty.

### C.4 Empty-edge rule
- If no source sample yields a box → `BuildEdge` returns empty edge; graph layer
  treats empty edge as "no edge" (paper: cannot create edge Ns→Nt).

### Tests `test_pmg_builder.cpp`
- Self-edge on a smooth space yields non-empty edge; every stored box is valid.
- Inject a planted BAD region → shrunk box excludes all BAD samples.
- Deterministic given fixed seed.
- Disconnected spaces (no good transitions) → empty edge.

---

## Phase D — Runtime k-NN bounding-box interpolation (paper §4, eqs 1–3, Fig 5)

**Goal:** replace `PmgEdge::LookupNearest` (single nearest) with k-nearest
interpolation of target boxes and transition points.

### D.1 k-NN over source samples
- `k = ParameterDimension + 1`; find k nearest source samples to query param.

### D.2 Allen 2002 weights (eq 1–2)
- `w'_i = 1/d(query, l_i) − 1/d(query, l_k)`, normalized `w_i = w'_i / Σ w'_j`.

### D.3 Interpolated bounding box + transition point (eq 3)
- `B(Ns,Nt) = Σ w_i · box_i` (component-wise on center+extent or min/max).
- Interpolate normalized source/target transition phases the same way.
- New `PmgEdge::LookupInterpolated(query)` → `{ targetBox, srcPhase, dstPhase }`.

### Tests `test_edge_lookup.cpp`
- Query exactly at a source sample → recovers that sample's box (weights degenerate).
- Query between two samples → interpolated box lies between the two.
- Weights sum to 1; box stays valid (`min ≤ max`).

---

## Phase E — Runtime alignment + transition execution (paper §3.1 end, §4)

**Goal:** runtime applies the recovered alignment transform before linear blend,
and uses interpolated transition points instead of fixed durations.

### E.1 Align before blend
- Before transitioning, align the target clip to the current clip using the
  stored/interpolated yaw+translation so the character does not pop.
- Accumulate a running world transform on the controller (root drift across
  repeated transitions must be continuous).

### E.2 Phase-driven transition window
- Start blend at interpolated source transition phase; blend length derived from
  the transition window, not a fixed `transition_duration_seconds_`.

### E.3 Target parameter from box
- Clamp/choose target parameter inside the interpolated box (already partly done;
  unify with D.3 box).

### Tests `test_runtime_controller.cpp`
- Continuous root trajectory across N self-transitions (no position jump > eps at
  the transition frame).
- Facing direction continuous across a turning transition (the Fig 2 "no bob" check).
- Controller never transitions before source transition phase.

---

## Phase F — BVH corpus → motion space → graph pipeline (paper §5.1)

**Goal:** build real parametric motion spaces and a graph from the `BVH/` clips,
not just synthetic data.

### F.1 Skeleton compatibility + clip ingestion
- Load a set of BVH clips sharing a skeleton; validate joint-count/name parity.
- Map clip → `ParametricMotionSpace` example with a caller-supplied parameter
  (e.g. curvature / travel-direction change as in §5.1).

### F.2 (Optional, can defer to Phase I) time registration
- Note: examples are currently blended at equal normalized phase. If real clips
  show foot-skate from phase misalignment, add timewarp/registration
  (Kovar-Gleicher'04) — tracked as G9, low priority.

### F.3 Graph assembly CLI
- `pmg_cli --build <spec>`: read a small text/JSON spec (nodes = clip groups +
  params, edges = node pairs + thresholds), build graph, report node/edge counts
  and edge box coverage. Persist graph to the paper's "<50KB plain text" format.

### Tests `test_bvh_pipeline.cpp`
- Load two real BVH clips from `BVH/`, build a 1-node space, generate a clip,
  assert frame count / fps.
- Build self-edge from real clips → non-empty (smoke).
- Graph save→load round-trips node/edge counts.

---

## Phase G — Applications (paper §5.2)

**Goal:** reproduce the three control applications.

### G.1 Random graph walk (§5.2.1)
- From current node, on reaching a transition region pick a random outgoing edge,
  random target parameter in box; stream indefinitely.

### G.2 Target-directed greedy control (§5.2.2)
- Choose the in-box parameter that best steers travel direction toward a target
  point (greedy, Srinivasan'05-style). Optional orientation term.

### G.3 Interactive control hook (§5.2.3)
- Per-node function: map a desired travel/facing direction request → parameter.
- CLI flags `--walk-random`, `--walk-to x z`, dumping root trajectory to stdout/CSV.

### Tests `test_applications.cpp`
- Random walk produces a finite continuous stream for K steps without throwing.
- Target-directed: final root position is closer to target than start (monotone
  enough on a straight-shot target).
- Deterministic given seed.

---

## Phase H — Visualization (optional, README checklist)

**Goal:** observe motion (paper uses rendered skeletons).

### H.1 Lightweight output first
- Export generated/streamed motion back to **BVH** (reuse skeleton + frames) so
  any external viewer can play it. No new deps.

### H.2 OpenGL/ImGui viewer (only if needed)
- Skeleton line render + joystick control, gated behind a CMake option; do not
  pull deps into `pmg_core`.

### Tests
- BVH export round-trips through `BvhLoader::Load` (frames/joints/fps preserved).

---

## Phase I — Phase-2 research extensions (deferred, see PHASE2_NOTES.md)

Trigger only on observed Phase A–H failures: partial-edge source domains,
phase-aware mid-clip transitions, contact-aware metric, manifold
parameterization, learned validity, inertialized blends, CUDA preprocessing.

---

## Cross-cutting

- Every new `src/*.cpp` added to `pmg_core` in `CMakeLists.txt`; every test added
  via `add_executable` + `add_test`.
- Keep `pmg_core` dependency-free (Phases A–G). Deps only behind CMake options (H).
- Public structs keep unit/assumption comments (existing convention).
- `ctest --output-on-failure` must stay green after each subphase.

## Suggested order / dependencies
A → B → C → D → E form the faithful-algorithm core (do in order).
F depends on A–B (real metric) and C (edges). G depends on D–E–F. H, I optional.
```
A ─► B ─► C ─► D ─► E ─► G
              └► F ─┘
                    └► H
```
