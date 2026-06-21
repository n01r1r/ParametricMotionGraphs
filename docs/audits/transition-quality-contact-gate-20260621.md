# Transition Quality/Contact Gate Audit — 2026-06-21

## Purpose

Catch box-accepted transitions that satisfy geometric `D/TBAD` consistency but
remain visually unsafe. This is a local diagnostic claim, not global visual
quality validation.

## FAIL-VIS-001

- Source parameter: `(-0.04, 0.8)`
- Requested/effective target: `(0.5, 0)` / `(0.5, 0)`
- Symptom: visible pop, hovering feet, stopped feet while legs remain elevated.
- Existing s11 result: 5,112 rows, 0 accepted BAD, 12 near-threshold accepted,
  `PASS_BUT_COVERAGE_SHRUNK`.
- Why box consistency is insufficient: target lies inside conservative
  interpolated box and `D=232.051 < TBAD=234`; neither fact measures runtime
  speed/contact continuity.

## Exact Probe

Artifact: `build/visual_gate/walk_2d_quality_gate_local.pmg`, rebuilt locally
from `specs/demo_walk_2d_triangulated.pmg_spec`.

Before quality gate:

- `accepted_by_box=true`
- `metric_class=NEUTRAL`
- `root_speed_ratio=1.57717`
- `yaw_rate_ratio=4.74812`
- left/right foot drift: `7.51078 / 3.75801` corpus units
- left foot height before/after: `1.94791 / 1.06863`
- right foot height before/after: `0.708700 / 0.362373`

After quality gate:

- `final_quality_gate_decision=reject`
- `reject_reason=root_speed`
- Experimental limits: root-speed ratio `1.5`, yaw-rate ratio `50`, stable
  contact drift `2.0`, stable-contact foot-height delta `2.0` corpus units.
- Temporal contact mismatch is reported but not rejected by default: short
  windows also classify normal strike/lift events as mismatch. CLI flag
  `--reject-contact-mismatch` enables that ablation.

Exact self regression `(0,1) -> (0,1)` remains accepted. Runtime schedules no
transition for an exact same-node/same-parameter request, so probe preserves
that behavior.

## s11 Result

- Evaluated rows: `5,112`
- Accepted BAD by box: `0`
- Accepted BAD after quality gate: `0`
- Near-threshold accepted after quality gate: `3`
- Box accepted, quality rejected: `110`
- Reject histogram: `root_speed=96`, `yaw_rate=14`
- Conclusion: `PASS_BUT_COVERAGE_SHRUNK`

Artifacts:

- `build/visual_gate/probe_FAIL_VIS_001.md`
- `build/visual_gate/probe_FAIL_VIS_001.csv`
- `build/visual_gate/acceptance_quality_gate_s11.md`
- `build/visual_gate/acceptance_quality_gate_s11.csv`

## Limitations

- Gate evaluates reconstructed smoothstep transition windows from artifact
  clips; it does not prove perceptual quality beyond sampled cases.
- Core `RuntimeController` scheduling is unchanged. Diagnostic proof now exists;
  runtime enforcement needs controller-owned skeleton/contact context and must
  be validated separately against live emitted poses.
- Existing intersected target-box behavior remains unchanged.
