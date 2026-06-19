# Integrated Verification — 2026-06-19

## Scope

Debug build, full CTest suite, Phase 8 focused integration tests.

## Results

| Check | Result |
|---|---|
| `cmake --build build --config Debug` | PASS |
| `ctest --test-dir build -C Debug --output-on-failure` | PASS — 37/37 |
| `ctest --test-dir build -C Debug -R "viewer_runtime\|viewer_graph\|runtime\|graph_io\|graph_spec\|pmg_builder\|offline" --output-on-failure` | PASS — 8/8 |

## Notes

- Initial build invocation failed because inherited environment contained both `Path` and `PATH`. Re-run normalized process-local path casing; build passed.
- Build emitted existing `MSB8028` shared intermediate-directory warning and GLFW `C4819` encoding warning.
- Build repeatedly reported missing `pwsh.exe` from a post-build step, but MSBuild returned success and all test executables ran.

## Checklist

- [x] Debug targets build
- [x] Full automated suite passes
- [x] Focused viewer/runtime/graph/offline suite passes
- [ ] Resolve build warnings if warning-free build becomes acceptance criterion
