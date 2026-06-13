# Agent Working Notes — ParametricMotionGraphs

Standing instructions for any AI coding agent working in this repo (Claude Code,
Codex, etc.). Agent-agnostic on purpose — `CLAUDE.md` is a thin loader that
imports this file, so every agent reads the same notes. See
[`docs/README.md`](docs/README.md) for the documentation map and
[`CONTEXT.md`](CONTEXT.md) for canonical vocabulary.

## Session hygiene (do this every session)

Before ending a work session or letting context cycle, always:

1. **Update docs to current state.** Reflect what changed this session in the
   relevant doc — `docs/STATUS.md` for core-scope/test claims, the matching
   `docs/HANDOFF_*.md` or design doc for feature work.
2. **Clean up stale references.** Fix merged/deleted branch names, bump
   `Last updated:` dates, retire handoff notes whose work has landed.
3. **Commit for a clean handoff.** Leave `git status` clean so the next session
   resumes from a known state, never from uncommitted drift.

## Git

Stable line is `main`; nothing commits to it directly. Day-to-day work lands on
long-lived, area-scoped integration branches and reaches `main` by PR once
stable:

- `dev/ui` — viewer / UI / UX (anything under `apps/viewer/`).
- `dev/core` — the `pmg_core` library, algorithms, and their tests (`src/`,
  `include/`, `tests/`).
- `dev/misc` — docs, build, memory, branch policy, and other cross-cutting work.

Rules:

- Commit each change to the matching `dev/*` branch. Do not branch per change and
  do not commit straight to `main`.
- A change spanning areas splits along these lines — UI code to `dev/ui`, its
  core test to `dev/core`, docs to `dev/misc` — so the branches stay file-disjoint
  and conflict-free and each area's history reads cleanly.
- Open a PR from a `dev/*` branch to `main` when its work is stable. No stacked
  PRs. Keep the `dev/*` branches alive across sessions instead of deleting them.
