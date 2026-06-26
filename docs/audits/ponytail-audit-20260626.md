# Ponytail audit + roadmap — 2026-06-26

Durable reference. Work order, top to bottom: **(1) audit cuts**, then
**(2) improvements now**, then **(3) final goal / what's left** (discussion).

Scope of section 1 is over-engineering/bloat only — not correctness, security,
or performance. Section 2 items are unblocked code changes with prior evidence.
Section 3 is data/scope-gated and is a discussion, not a task.

---

## 1. Audit cuts (do first)

### 1a. `delete:` tracked CLI-output text — DONE 2026-06-26

`experiments/cmu/out/*.txt` (5 files, 115 lines) were checked-in command
output, regenerable via `experiments/cmu/fetch.sh` + `pmg_cli audit-corpus`.
Untracked and added to `.gitignore` (`experiments/cmu/out/`).

### 1b. `shrink:` `apps/PmgDiagnosticCommands.cpp` — DONE 2026-06-26

**Result:** all 9 dead commands removed (dispatch + usage banner + bodies).
File **6886 → 4375 lines** (−2511 in this file, −2523 across both apps files).
Full build + **50/50 ctest** green, viewer builds, removed cmds → usage,
kept cmds work.

Removed: recut-search/evaluate, space-sweep, simplex/parameter-response,
inventory-bvh, root-canonicalization, registration-phase-alignment,
reachable-region (CLI wrapper only — see below).

**Shared infrastructure deliberately KEPT** (montage/contact/acceptance reuse
it): `ReachableRegionAuditOptions/Row/Data`, `BuildReachableRegionAudit`,
`BuildAuditParameterSamples`, `EvaluateConfiguredTransition`,
`Root/Heading/VelocityJumpAtTransition`, `ResolveNodeIndex`, `ResolveEdgeIndex`,
`AppendUniqueParameter`, `HorizontalDistance`, `ParameterCsv/ParameterMd/Vec3Csv`,
`PoseSeamClass`. The reachable-region "audit" is the shared transition-metric
core; only its CLI Write/Parse/Command wrappers were exclusive and removed.

**Pitfalls hit (recorded so the next cut is cheaper):**
- Blind line-range delete swallowed the `}  // namespace` / `namespace pmgcli {`
  transition → linker error (`TryRunDiagnosticCommand` fell to internal linkage).
- `grep | head` truncated ref lists → deleted shared helpers (`HorizontalDistance`,
  `ReachableRegionAudit*`, `BuildReachableRegionAudit`) → build errors. Always
  use the FULL untruncated ref list and check for refs in KEPT line zones before
  cutting. Build is the net but costs a full MSVC cycle per miss.

Below = original full kill-list for reference.

#### Original kill-list (6886 lines)

Biggest single file in repo (core `ParametricMotionGraph.cpp` is 313). 20
CLI subcommands dispatched from `TryRunDiagnosticCommand` (line 6805). Only
**~9 are live gates** (referenced by `tests/` or `scripts/`); the rest are
one-shot investigation probes whose conclusions already live in agent memory
and `docs/audits/`.

**NOT a clean line-range delete.** Handlers interleave shared free-function
helpers (e.g. the 3735→5555 span mixes `ReachableRegionAuditCommand` with
helpers reused by montage/pop). Execution = remove handler + its
`Parse*Options` + options struct + dispatch branch + usage line, then let the
compiler flag now-unused statics, remove iteratively, **rebuild after each
group** (`cmake --build build --config Debug`, build is functional).

| Subcommand | Handler @line | Live gate? | Verdict |
|---|---|---|---|
| `--audit-transition-pop` | TransitionPopAuditCommand 5637 | tests | **KEEP** |
| `--audit-loop` | LoopAuditCommand 5581 | tests | **KEEP** |
| `--probe-transition` | TransitionProbeCommand 5708 | tests | **KEEP** |
| `--inspect-skeleton` | SkeletonAuditCommand 5571 | tests | **KEEP** |
| `--audit-transition-montage` | TransitionMontageAuditCommand 5555 | tests | **KEEP** |
| `--audit-transition-acceptance-consistency` | …AcceptanceConsistency 5769 | tests, canon gate | **KEEP** |
| `--audit-contact-transitions` | ContactTransitionAuditCommand 5696 | tests | **KEEP** |
| `--audit-motion-space-registration` | …RegistrationAudit 3078 | PHASE accept #5 | **KEEP** |
| `--audit-cyclic-continuity` | CyclicAuditCommand 1331 | cyclic gate | **KEEP** |
| `--space-sweep` | SpaceSweepCommand 247 (~1084 ln) | no | **CUT** |
| `--audit-reachable-region` | ReachableRegionAuditCommand 3735 (~big) | no | **CUT** (after helper trace) |
| `--audit-root-canonicalization` | RootCanonicalizationAudit 1665 (~525) | no, recorded faithful | **CUT** |
| `--audit-registration-phase-alignment` | …PhaseAlignment 2190 (~888) | no, recorded | **CUT** |
| `--search-cyclic-recuts` | CyclicRecutSearchCommand 5795 | no, gate closed 06-22 | **CUT** |
| `--evaluate-cyclic-recuts` | CyclicRecutEvaluateCommand 5872 | no, gate closed 06-22 | **CUT** |
| `--inventory-bvh` | InventoryBvhCommand 6753 | no | **CUT** |
| `--audit-simplex-node` / `--audit-parameter-node` | SimplexAuditCommand 6583 | no | **CUT** |
| `--audit-parameter-response` | SimplexAuditCommand 6583 (shared) | no, wobble recorded | **CUT** |
| `--validate-graph` | ValidateGraphCommand 6172 | check `--validate-graph-spec` is the real one | **VERIFY then CUT** |

Cut order (cleanest/most-self-contained first): recut-search pair →
inventory-bvh → simplex/parameter-response → space-sweep →
root-canonicalization → registration-phase-alignment → reachable-region.
Update `apps/PmgCommands.cpp` usage banner for each removed verb. Expected
net: **~-2000 to -3000 lines**. Each removed audit's finding stays in
`docs/audits/` so nothing is lost.

### 1c. `yagni:` `src/GraphIo.cpp` legacy readers — SKIPPED (audit finding was wrong)

Inspection of `GraphIo.cpp:16-49` killed this finding. Version handling is **not**
scattered branching — it is one declarative table (`FormatForHeader`), one row
per version, and all parsing is driven by the row's boolean flags (shared code,
no per-version branches). The design comment says exactly this: "adding a version
is one row, not a new branch threaded through the reader."

So cutting V2–V7 removes **6 table rows and zero logic**, while dropping the
ability to read old artifacts. ~6 lines saved, no complexity removed. Per the
ladder's rung 1 (does this even need doing): **keep it**. The original finding
assumed branching that the author already factored away.

### 1d. Doc nit — DONE 2026-06-26

README (4 spots) + CONTEXT.md (2 spots) updated V12 → V13 to match the writer
(`GraphIo.cpp:1036`).

---

## 2. Improvements now (do second — unblocked, code-side, evidenced)

1. **goto arrival ease-speed** — `arrival_speed_distance=3.0` too small →
   speed never eases → target misses. Geometry min-radius 16u is the other
   factor. P0, small diff. Evidence: `docs/audits/goto-arrival-ease-20260623.md`,
   `target-reaching-minradius-diagnosis` memory.
2. **online runtime foot-lock** (step 3) — post-process `LockFootContacts`
   on generated clips exists + `--audit-foot-skate` (-62% slide); online
   runtime application during pose streaming does not. Evidence:
   `foot-lock-physical-grounding` memory.

---

## 3. Final goal / what's left (discuss third)

Core PMG is code-complete for the available single-family corpus: single-node
`walk_2d` build, registration, sampled self-edges, Kovar metric, phase-gated
runtime lookup, reachable-box intersection, direct steering, single-node
waypoint path-follow, V13 persistence, MotionClipSegment windows, foot-lock
post-process audit, structure-tolerant contact registration.

**The final-goal gap is DATA-blocked, not code-blocked:**

- **Multi-node graph** (2nd node + inter-node edges + graph-walk demo) — the
  paper-Section-5 milestone. Blocked: reproducible corpus audit found only 1
  viable cyclic run candidate; 2026-06-22 recut follow-up kept the gate closed.
  Gate to reopen: `pmg_cli audit-corpus --corpus-root /path/to/cmubvh`. No
  code missing.
- **Visual transition quality (pops)** — report-only; cross-family data gap,
  blend guard correct, not a code bug. **RESOLVED 2026-06-27:** the transition-clip
  bridge idea (Tweak A2) was spiked and measured **NO-GO** — the point-cloud-aligned
  cross-fade is already smooth (runtime pop ~1.0) even across walk↔run/punch/squat/jump,
  because alignment always finds a compatible frame pair (shared near-neutral pose).
  Closed; see `docs/research/transition-clip-edges-A2-design.md` §7.

To "finish": supply a compatible multi-clip corpus (CMU gate) **or** formally
scope as PMG-core demo (README already states this). Decision, not engineering.
Deferred-by-choice until evidence demands: replay export, controller
comparison, foot-lock-as-default policy.
