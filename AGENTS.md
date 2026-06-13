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

- Branch per change, open a PR, merge, then delete the branch. No direct commits
  to `main`, no stacked PRs, no long-lived side branches.
