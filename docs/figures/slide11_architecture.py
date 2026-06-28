"""Slide 11 architecture animation: BVH data -> offline build -> runtime -> viewer.

A vertical module pipeline that reveals one box at a time, then groups the boxes
into four role-tiers (Data / Offline Build / Runtime / Viewer). Purpose is *role
separation*, not algorithm detail -- so each box carries the real implementation
name, and no equations / metric formulas appear.

Module names are the actual headers under include/pmg + apps/viewer:
  BvhLoader, Skeleton, ForwardKinematics, MotionSpacePreparation, PmgBuilder,
  PmgArtifact, GraphIo, RuntimeController, ViewerRuntimeModule.

Animated reveal runs ~6-7s, then the final frame holds so the presenter can talk
over a static image (capture that last frame as a PNG fallback -- see below).

Two variants:
  Slide11Architecture          - right-shifted pipeline + left tier braces.
  Slide11ArchitectureCentered  - centered pipeline, no braces, compact tier key
                                 (use when the slide gives the video the full
                                 width, or for a calmer single-column look).

Render (1080p, 30fps, white background):
    manim render -qh --fps 30 docs/figures/slide11_architecture.py Slide11Architecture
    manim render -qh --fps 30 docs/figures/slide11_architecture.py Slide11ArchitectureCentered

Transparent background for overlaying on a slide (PNG sequence / .mov):
    manim render -qh --fps 30 -t docs/figures/slide11_architecture.py Slide11Architecture

Grab the held last frame as a still fallback:
    manim render -qh -s docs/figures/slide11_architecture.py Slide11Architecture
"""

from manim import *

# Match concept_scenes.py palette so the deck reads as one visual family.
INK = "#1a1a2e"
MUTED = "#6b7280"
DATA = "#1d4ed8"        # blue   - data layer
OFFLINE = "#ea580c"     # orange - offline build
RUNTIME = "#15803d"     # green  - runtime
VIEWER = "#7c3aed"      # purple - viewer

FONT = "Arial"
MONO = "Consolas"       # impl names in a mono face; Arial fallback if missing


def _white_bg(scene):
    scene.camera.background_color = WHITE


def _module(role, impl, color, center, width=3.4, height=0.66):
    """One pipeline box: role title (sans) over implementation name (mono)."""
    box = RoundedRectangle(width=width, height=height, corner_radius=0.1,
                           color=color, stroke_width=3,
                           fill_color=color, fill_opacity=0.08)
    box.move_to(center)
    title = Text(role, font_size=21, color=INK, weight=BOLD, font=FONT)
    title.move_to(box.get_center() + UP * 0.13)
    name = Text(impl, font_size=15, color=color, font=MONO)
    name.move_to(box.get_center() + DOWN * 0.16)
    return VGroup(box, title, name)


class Slide11Architecture(Scene):
    """Vertical module pipeline grouped into four role-tiers."""

    def construct(self):
        _white_bg(self)

        title = Text("Implementation structure: BVH data to live Viewer",
                     font_size=30, color=INK, weight=BOLD, font=FONT)
        title.move_to([0, 3.5, 0])
        self.play(FadeIn(title, shift=DOWN * 0.2), run_time=0.5)

        # (role label, impl name, tier color) top -> bottom.
        specs = [
            ("BVH Loader",            "BvhLoader",                  DATA),
            ("Skeleton / FK",         "Skeleton · ForwardKinematics", DATA),
            ("Motion Space",          "MotionSpacePreparation",     OFFLINE),
            ("Transition Edges",      "PmgBuilder",                 OFFLINE),
            ("PMG Artifact / Spec",   "PmgArtifact · GraphIo",      OFFLINE),
            ("Runtime Controller",    "RuntimeController",          RUNTIME),
            ("Viewer / UI / Diag.",   "ViewerRuntimeModule (ImGui)", VIEWER),
        ]

        px = 0.9            # pipeline x (shifted right; tier braces sit left)
        y0, step = 2.55, 0.9
        boxes = [
            _module(role, impl, color, [px, y0 - i * step, 0])
            for i, (role, impl, color) in enumerate(specs)
        ]

        # Downward connector arrows between consecutive boxes.
        arrows = []
        for a, b in zip(boxes, boxes[1:]):
            arrows.append(Arrow(a[0].get_bottom(), b[0].get_top(), buff=0.04,
                                color=MUTED, stroke_width=4,
                                max_tip_length_to_length_ratio=0.5))

        # Reveal boxes + arrows in pipeline order.
        self.play(FadeIn(boxes[0], shift=DOWN * 0.15), run_time=0.55)
        for arrow, box in zip(arrows, boxes[1:]):
            self.play(GrowArrow(arrow), run_time=0.28)
            self.play(FadeIn(box, shift=DOWN * 0.15), run_time=0.5)

        # Group into four role-tiers with left braces + labels.
        tiers = [
            ("Data",          boxes[0:2], DATA),
            ("Offline Build", boxes[2:5], OFFLINE),
            ("Runtime",       boxes[5:6], RUNTIME),
            ("Viewer",        boxes[6:7], VIEWER),
        ]
        brace_group = VGroup()
        for name, grp, color in tiers:
            span = VGroup(*[b[0] for b in grp])
            brace = Brace(span, direction=LEFT, color=color, buff=0.25)
            lbl = Text(name, font_size=20, color=color, weight=BOLD, font=FONT)
            lbl.next_to(brace, LEFT, buff=0.18)
            brace_group.add(VGroup(brace, lbl))
        self.play(LaggedStart(*[FadeIn(g, shift=RIGHT * 0.15) for g in brace_group],
                              lag_ratio=0.18), run_time=0.9)

        # Hold the assembled frame for the presenter.
        self.wait(2.5)


class Slide11ArchitectureCentered(Scene):
    """Centered pipeline, no braces; a compact color key carries the tiers."""

    def construct(self):
        _white_bg(self)

        title = Text("Implementation structure: BVH data to live Viewer",
                     font_size=30, color=INK, weight=BOLD, font=FONT)
        title.move_to([0, 3.5, 0])
        self.play(FadeIn(title, shift=DOWN * 0.2), run_time=0.5)

        # Compact tier color key (replaces the braces).
        key = VGroup()
        for name, color in [("Data", DATA), ("Offline Build", OFFLINE),
                            ("Runtime", RUNTIME), ("Viewer", VIEWER)]:
            swatch = Square(side_length=0.22, color=color, stroke_width=2,
                            fill_color=color, fill_opacity=0.85)
            lbl = Text(name, font_size=18, color=INK, font=FONT)
            lbl.next_to(swatch, RIGHT, buff=0.12)
            key.add(VGroup(swatch, lbl))
        key.arrange(RIGHT, buff=0.55).move_to([0, 2.85, 0])

        specs = [
            ("BVH Loader",            "BvhLoader",                  DATA),
            ("Skeleton / FK",         "Skeleton · ForwardKinematics", DATA),
            ("Motion Space",          "MotionSpacePreparation",     OFFLINE),
            ("Transition Edges",      "PmgBuilder",                 OFFLINE),
            ("PMG Artifact / Spec",   "PmgArtifact · GraphIo",      OFFLINE),
            ("Runtime Controller",    "RuntimeController",          RUNTIME),
            ("Viewer / UI / Diag.",   "ViewerRuntimeModule (ImGui)", VIEWER),
        ]
        y0, step = 1.95, 0.85
        boxes = [
            _module(role, impl, color, [0, y0 - i * step, 0])
            for i, (role, impl, color) in enumerate(specs)
        ]
        arrows = [
            Arrow(a[0].get_bottom(), b[0].get_top(), buff=0.04, color=MUTED,
                  stroke_width=4, max_tip_length_to_length_ratio=0.5)
            for a, b in zip(boxes, boxes[1:])
        ]

        self.play(FadeIn(key, shift=DOWN * 0.1), run_time=0.5)
        self.play(FadeIn(boxes[0], shift=DOWN * 0.15), run_time=0.55)
        for arrow, box in zip(arrows, boxes[1:]):
            self.play(GrowArrow(arrow), run_time=0.28)
            self.play(FadeIn(box, shift=DOWN * 0.15), run_time=0.5)
        self.wait(2.5)
