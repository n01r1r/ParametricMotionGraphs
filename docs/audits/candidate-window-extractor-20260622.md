# Candidate window extractor audit — 2026-06-22

## Purpose

Move offline motion-space preparation one small step toward paper-faithful example extraction. Tool proposes candidate frame windows from longer BVH clips and preserves source/frame provenance for human review.

## Inputs and outputs

Input is one loaded BVH clip plus configurable minimum/maximum inclusive window lengths, scan stride, top-k count, endpoint pose threshold, and heading threshold. Output contains inclusive frame range, duration, normalized aligned endpoint-pose score, root displacement in native BVH units, wrapped heading delta in radians, and reason text.

## Method

Extractor scans windows, compares short start/end joint-position clouds with existing rigid-aligned Kovar-style distance, checks root heading continuity, then returns deterministic top-k proposals sorted by ascending score. Root displacement remains evidence, not a rejection term, because locomotion cycles legitimately translate.

## Claim boundary

This is an engineering-assisted candidate proposal tool, not full KG04 automatic database search. It does not discover motion families, classify walk/jog, select motion-space node membership, or change graph/runtime behavior. Contact state is not inferred without explicit trustworthy contact-joint settings. Human/spec author decides whether any proposed window belongs in a motion space.

## Checklist

- Inspect source BVH, frame count, and FPS in report.
- Review score, root displacement, heading delta, and reason.
- Treat empty output as “no plausible candidate under configured thresholds,” not proof no cycle exists.
- Keep final example and node-membership decisions human-authored.
