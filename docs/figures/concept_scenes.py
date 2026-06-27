"""Manim concept scenes for the PMG report (explanatory support only).

Three short scenes, wording matched to docs/final_report.md framing:
  1. PmgPipeline   - clips -> parametric motion space -> graph node/edge -> stream
  2. ParametricNode - a node is a 2D motion space (turn_rate x travel_speed)
  3. EdgeSampling  - sample source/target windows, Kovar distance, accept/reject

Concept only: no quantitative values are shown (the numbers live in the
report figures). These illustrate intuition, not evidence.

Render (1080p, 30fps, white background):
    manim render -qh --fps 30 docs/figures/concept_scenes.py PmgPipeline
    manim render -qh --fps 30 docs/figures/concept_scenes.py ParametricNode
    manim render -qh --fps 30 docs/figures/concept_scenes.py EdgeSampling
"""

from manim import *

# Light scheme: dark ink on white background.
INK = "#1a1a2e"
MUTED = "#6b7280"
ACCENT = "#1d4ed8"      # blue  - walk node / primary
ACCENT2 = "#ea580c"     # orange - run node / target
GOOD = "#15803d"        # green  - accepted
BAD = "#b91c1c"         # red    - rejected


def _white_bg(scene):
    scene.camera.background_color = WHITE


def _label(text, size=28, color=INK, weight=NORMAL):
    return Text(text, font_size=size, color=color, weight=weight)


class PmgPipeline(Scene):
    """clips -> parametric motion space -> graph node/edge -> generated stream."""

    def construct(self):
        _white_bg(self)

        title = _label("Parametric Motion Graphs pipeline", 34, INK, BOLD)
        title.to_edge(UP, buff=0.5)
        self.play(FadeIn(title, shift=DOWN * 0.2), run_time=0.6)

        # Stage 1: a few BVH clips.
        clips = VGroup(*[
            RoundedRectangle(width=0.9, height=0.5, corner_radius=0.08,
                             color=INK, stroke_width=2)
            for _ in range(3)
        ]).arrange(DOWN, buff=0.18)
        clips_lbl = _label("BVH clips", 24, MUTED).next_to(clips, DOWN, buff=0.25)
        stage1 = VGroup(clips, clips_lbl).to_edge(LEFT, buff=0.6).shift(DOWN * 0.3)

        # Stage 2: parametric motion space (the node).
        space = Square(side_length=1.6, color=ACCENT, stroke_width=3)
        space_dots = VGroup(*[
            Dot(space.get_center() + p, radius=0.05, color=ACCENT)
            for p in [LEFT * 0.5 + UP * 0.4, RIGHT * 0.5 + UP * 0.3,
                      LEFT * 0.4 + DOWN * 0.5, RIGHT * 0.45 + DOWN * 0.4]
        ])
        space_lbl = _label("parametric\nmotion space", 22, ACCENT)
        space_lbl.next_to(space, DOWN, buff=0.25)
        stage2 = VGroup(space, space_dots, space_lbl).shift(LEFT * 2.3 + DOWN * 0.3)

        # Stage 3: graph - two nodes, a directed edge.
        n1 = Circle(radius=0.45, color=ACCENT, stroke_width=3)
        n2 = Circle(radius=0.45, color=ACCENT2, stroke_width=3).shift(RIGHT * 1.6)
        edge = Arrow(n1.get_right(), n2.get_left(), buff=0.08,
                     color=INK, stroke_width=4, max_tip_length_to_length_ratio=0.25)
        graph_lbl = _label("graph:\nnodes + edges", 22, INK)
        nodes = VGroup(n1, n2, edge)
        stage3 = VGroup(nodes, graph_lbl.next_to(nodes, DOWN, buff=0.25))
        stage3.shift(RIGHT * 1.1 + DOWN * 0.3)

        # Stage 4: generated motion stream (a continuous path).
        stream = VMobject(color=INK, stroke_width=3)
        stream.set_points_smoothly([
            LEFT * 0.9 + DOWN * 0.1, LEFT * 0.3 + UP * 0.25,
            RIGHT * 0.3 + DOWN * 0.2, RIGHT * 0.9 + UP * 0.15,
        ])
        stream_lbl = _label("generated\nmotion stream", 22, INK)
        stage4 = VGroup(stream, stream_lbl.next_to(stream, DOWN, buff=0.35))
        stage4.to_edge(RIGHT, buff=0.6).shift(DOWN * 0.3)

        a12 = Arrow(stage1.get_right(), stage2.get_left(), buff=0.2,
                    color=MUTED, stroke_width=4)
        a23 = Arrow(stage2.get_right(), stage3.get_left(), buff=0.2,
                    color=MUTED, stroke_width=4)
        a34 = Arrow(stage3.get_right(), stage4.get_left(), buff=0.2,
                    color=MUTED, stroke_width=4)

        self.play(FadeIn(stage1, shift=RIGHT * 0.2), run_time=0.7)
        self.play(GrowArrow(a12), run_time=0.4)
        self.play(FadeIn(stage2, shift=RIGHT * 0.2), run_time=0.7)
        self.play(GrowArrow(a23), run_time=0.4)
        self.play(FadeIn(stage3, shift=RIGHT * 0.2), run_time=0.7)
        self.play(GrowArrow(a34), run_time=0.4)
        self.play(Create(stream), FadeIn(stage4[1]), run_time=0.9)
        self.wait(1.6)


class ParametricNode(Scene):
    """A node is a 2D parameterized motion space: turn_rate x travel_speed."""

    def construct(self):
        _white_bg(self)

        title = _label("A node is a parameterized motion space", 32, INK, BOLD)
        title.to_edge(UP, buff=0.5)
        self.play(FadeIn(title, shift=DOWN * 0.2), run_time=0.6)

        axes = Axes(
            x_range=[0, 4, 1], y_range=[0, 4, 1],
            x_length=5.2, y_length=5.2,
            axis_config={"color": INK, "stroke_width": 2,
                         "include_ticks": False, "include_tip": True},
        ).shift(DOWN * 0.35)
        x_lbl = _label("turn_rate", 24, INK).next_to(axes.x_axis, DOWN, buff=0.2)
        y_lbl = _label("travel_speed", 24, INK).rotate(PI / 2)
        y_lbl.next_to(axes.y_axis, LEFT, buff=0.2)

        self.play(Create(axes), FadeIn(x_lbl), FadeIn(y_lbl), run_time=0.9)

        # Four anchor examples (positions are illustrative only).
        anchor_pts = [(0.8, 0.9), (1.4, 3.1), (3.2, 1.1), (3.0, 3.3)]
        anchors = VGroup(*[
            Dot(axes.c2p(x, y), radius=0.09, color=ACCENT) for x, y in anchor_pts
        ])
        anchors_lbl = _label("4 example clips (anchors)", 22, ACCENT)
        anchors_lbl.to_corner(UR, buff=0.5).shift(DOWN * 1.1)
        self.play(LaggedStart(*[GrowFromCenter(a) for a in anchors],
                              lag_ratio=0.2), FadeIn(anchors_lbl), run_time=1.0)

        # One query point inside the space.
        qx, qy = 2.1, 2.0
        query = Dot(axes.c2p(qx, qy), radius=0.11, color=BAD)
        query_lbl = _label("query parameter", 22, BAD).next_to(query, UP, buff=0.15)
        self.play(GrowFromCenter(query), FadeIn(query_lbl), run_time=0.6)

        # Weight lines: nearer anchor -> thicker line (interpolation intuition).
        lines = VGroup()
        for (x, y) in anchor_pts:
            d = ((x - qx) ** 2 + (y - qy) ** 2) ** 0.5
            w = max(1.5, 7.0 / (d + 0.4))
            lines.add(Line(axes.c2p(x, y), axes.c2p(qx, qy),
                           color=MUTED, stroke_width=w, stroke_opacity=0.7))
        weight_lbl = _label("blend nearby anchors\n(weights ~ closeness)", 22, INK)
        weight_lbl.to_corner(DR, buff=0.5)
        self.play(Create(lines), FadeIn(weight_lbl), run_time=1.0)
        self.wait(1.8)


class EdgeSampling(Scene):
    """Edge construction: sample windows, Kovar distance, accept/reject."""

    def construct(self):
        _white_bg(self)

        title = _label("Building an edge: sample candidate transitions", 30, INK, BOLD)
        title.to_edge(UP, buff=0.5)
        self.play(FadeIn(title, shift=DOWN * 0.2), run_time=0.6)

        # Source end-window (left) and target start-window (right) as pose strips.
        def window(color, n=4):
            frames = VGroup(*[
                Line(UP * 0.35, DOWN * 0.35, color=color, stroke_width=3)
                for _ in range(n)
            ]).arrange(RIGHT, buff=0.35)
            box = SurroundingRectangle(frames, color=color, buff=0.25,
                                       corner_radius=0.08, stroke_width=2)
            return VGroup(box, frames)

        src = window(ACCENT).shift(LEFT * 3.4 + UP * 0.4)
        tgt = window(ACCENT2).shift(RIGHT * 3.4 + UP * 0.4)
        src_lbl = _label("source end-window", 22, ACCENT).next_to(src, UP, buff=0.3)
        tgt_lbl = _label("target start-window", 22, ACCENT2).next_to(tgt, UP, buff=0.3)

        self.play(FadeIn(src, src_lbl, shift=RIGHT * 0.2),
                  FadeIn(tgt, tgt_lbl, shift=LEFT * 0.2), run_time=0.8)

        # Sample candidate transitions: source frame -> target frame.
        src_frames = src[1]
        tgt_frames = tgt[1]
        pairs = [(0, 0), (1, 0), (2, 1), (3, 2), (3, 3)]
        cands = VGroup(*[
            Line(src_frames[i].get_center(), tgt_frames[j].get_center(),
                 color=MUTED, stroke_width=2, stroke_opacity=0.6)
            for i, j in pairs
        ])
        kovar_lbl = _label("score each candidate by Kovar-style\ntransition distance",
                           22, INK).shift(DOWN * 1.4)
        self.play(LaggedStart(*[Create(c) for c in cands], lag_ratio=0.15),
                  run_time=1.0)
        self.play(FadeIn(kovar_lbl), run_time=0.5)
        self.wait(0.4)

        # Accept (low distance -> green) / reject (high distance -> red).
        verdicts = [GOOD, GOOD, BAD, GOOD, BAD]
        anims = []
        for c, v in zip(cands, verdicts):
            anims.append(c.animate.set_stroke(color=v, width=4, opacity=1.0))
        self.play(*anims, run_time=0.8)

        accept_lbl = _label("accept the edge when enough\ngood samples exist",
                            24, GOOD, BOLD).shift(DOWN * 2.6)
        self.play(FadeIn(accept_lbl, shift=UP * 0.2), run_time=0.6)
        self.wait(1.6)
