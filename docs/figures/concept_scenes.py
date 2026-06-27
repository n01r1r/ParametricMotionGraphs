"""Manim concept scenes for the PMG report (explanatory support only).

Three short scenes, wording and encodings matched to docs/final_report.md:
  1. PmgPipeline    - motion examples -> motion-space node -> transition graph
                      of motion-space nodes -> generated stream
  2. ParametricNode - a node is a 2D motion space (turn_rate x travel_speed)
                      with a *limited* convex support region (real anchors)
  3. EdgeSampling   - sample a source/target candidate matrix, score each pair
                      with a Kovar-style distance, accept the edge on enough
                      GOOD samples

Concept only: no quantitative values are shown (the numbers live in the report
figures). These illustrate intuition, not evidence. Layout uses fixed
coordinates (move_to) rather than chained next_to() to avoid overlaps.

Render (1080p, 30fps, white background):
    manim render -qh --fps 30 docs/figures/concept_scenes.py PmgPipeline
    manim render -qh --fps 30 docs/figures/concept_scenes.py ParametricNode
    manim render -qh --fps 30 docs/figures/concept_scenes.py EdgeSampling
"""

from manim import *

# Light scheme: dark ink on white background.
INK = "#1a1a2e"
MUTED = "#6b7280"
ACCENT = "#1d4ed8"      # blue   - source / primary
ACCENT2 = "#ea580c"     # orange - target / secondary
GOOD = "#15803d"        # green  - accepted
NEUTRAL = "#a16207"     # amber  - borderline
BAD = "#b91c1c"         # red    - rejected
QUERY = "#7c3aed"       # purple - query point (neutral, not "bad")
SHADE = "#bfdbfe"       # light blue - supported region fill


def _white_bg(scene):
    scene.camera.background_color = WHITE


# Explicit font: the Manim default renders with uneven word/letter spacing
# (collapsed spaces, loose kerning). A plain sans face spaces consistently.
FONT = "Arial"


def _label(text, size=24, color=INK, weight=NORMAL):
    return Text(text, font_size=size, color=color, weight=weight, font=FONT)


def _param_box(side=0.9, color=ACCENT, n_dots=4, seed=0):
    """A small parameter-space tile: a square with a few example dots inside.

    Used as the visual unit for "a node is a parameterized motion space".
    """
    box = Square(side_length=side, color=color, stroke_width=3)
    offs = [
        LEFT * 0.25 + UP * 0.2, RIGHT * 0.28 + UP * 0.15,
        LEFT * 0.22 + DOWN * 0.24, RIGHT * 0.24 + DOWN * 0.2,
    ]
    dots = VGroup(*[
        Dot(box.get_center() + o * (side / 0.9), radius=0.045, color=color)
        for o in offs[:n_dots]
    ])
    return VGroup(box, dots)


class PmgPipeline(Scene):
    """examples -> motion-space node -> transition graph -> generated stream."""

    def construct(self):
        _white_bg(self)

        title = _label("Parametric Motion Graphs pipeline", 34, INK, BOLD)
        title.move_to([0, 3.4, 0])
        self.play(FadeIn(title, shift=DOWN * 0.2), run_time=0.6)

        row_y = 0.3          # vertical centre of the diagram row
        cap_y = -2.6         # fixed caption baseline
        xs = [-5.0, -1.9, 1.5, 4.9]

        # Stage 1: a few motion examples (raw clips).
        clips = VGroup(*[
            RoundedRectangle(width=0.95, height=0.42, corner_radius=0.07,
                             color=INK, stroke_width=2)
            for _ in range(3)
        ]).arrange(DOWN, buff=0.2).move_to([xs[0], row_y, 0])
        cap1 = _label("motion examples", 22, MUTED).move_to([xs[0], cap_y, 0])

        # Stage 2: one motion-space node (the parameterized family).
        node = _param_box(side=1.5, color=ACCENT).move_to([xs[1], row_y, 0])
        cap2 = _label("motion-space node", 22, ACCENT).move_to([xs[1], cap_y, 0])
        cap2b = _label("node = parameterized family", 18, MUTED)
        cap2b.move_to([xs[1], cap_y - 0.42, 0])

        # Stage 3: transition graph - two *motion-space* nodes + gated edge.
        g_src = _param_box(side=0.95, color=ACCENT).move_to([xs[2] - 0.85, row_y, 0])
        g_tgt = _param_box(side=0.95, color=ACCENT2).move_to([xs[2] + 0.85, row_y, 0])
        g_edge = Arrow(g_src.get_right(), g_tgt.get_left(), buff=0.05,
                       color=INK, stroke_width=4,
                       max_tip_length_to_length_ratio=0.3)
        edge_lbl = _label("quality-gated edge", 16, INK)
        edge_lbl.move_to([xs[2], row_y + 0.95, 0])
        cap3 = _label("transition graph", 22, INK).move_to([xs[2], cap_y, 0])
        cap3b = _label("edges link motion spaces", 18, MUTED)
        cap3b.move_to([xs[2], cap_y - 0.42, 0])
        stage3 = VGroup(g_src, g_tgt, g_edge, edge_lbl)

        # Stage 4: generated motion stream (a continuous path).
        stream = VMobject(color=INK, stroke_width=4)
        stream.set_points_smoothly([
            [xs[3] - 0.85, row_y - 0.15, 0], [xs[3] - 0.3, row_y + 0.3, 0],
            [xs[3] + 0.3, row_y - 0.25, 0], [xs[3] + 0.85, row_y + 0.2, 0],
        ])
        cap4 = _label("generated stream", 22, INK).move_to([xs[3], cap_y, 0])

        # Connector arrows on a fixed baseline.
        def connect(a, b):
            return Arrow([a, row_y, 0], [b, row_y, 0], buff=0.25,
                         color=MUTED, stroke_width=4,
                         max_tip_length_to_length_ratio=0.35)
        a12 = connect(xs[0] + 0.55, xs[1] - 0.85)
        a23 = connect(xs[1] + 0.85, xs[2] - 1.4)
        a34 = connect(xs[2] + 1.4, xs[3] - 0.95)

        self.play(FadeIn(clips, shift=RIGHT * 0.2), FadeIn(cap1), run_time=0.7)
        self.play(GrowArrow(a12), run_time=0.35)
        self.play(FadeIn(node, shift=RIGHT * 0.2),
                  FadeIn(cap2), FadeIn(cap2b), run_time=0.7)
        self.play(GrowArrow(a23), run_time=0.35)
        self.play(FadeIn(stage3, shift=RIGHT * 0.2),
                  FadeIn(cap3), FadeIn(cap3b), run_time=0.7)
        self.play(GrowArrow(a34), run_time=0.35)
        self.play(Create(stream), FadeIn(cap4), run_time=0.9)
        self.wait(1.6)


class ParametricNode(Scene):
    """A node is a 2D motion space with a limited convex support region.

    Anchors are the real demo_walk_2d examples: three on the travel_speed=0
    turn_rate axis, one faster example reaching ~0.75. The supported region is
    their convex hull (a triangle) - not a full rectangle.
    """

    def construct(self):
        _white_bg(self)

        title = _label("A node is a parameterized motion space", 32, INK, BOLD)
        title.move_to([0, 3.4, 0])
        self.play(FadeIn(title, shift=DOWN * 0.2), run_time=0.6)

        axes = Axes(
            x_range=[-0.6, 1.3, 0.5], y_range=[0, 1.0, 0.5],
            x_length=6.4, y_length=4.4,
            axis_config={"color": INK, "stroke_width": 2,
                         "include_ticks": False, "include_tip": True},
        ).move_to([0, -0.5, 0])
        # Axis labels at the tips (centre is occupied by the sweep arrow / hull).
        x_lbl = _label("turn_rate", 22, INK).next_to(axes.x_axis.get_end(),
                                                     UR, buff=0.12)
        y_lbl = _label("travel_speed", 22, INK).next_to(axes.y_axis.get_end(),
                                                        RIGHT, buff=0.15)
        self.play(Create(axes), FadeIn(x_lbl), FadeIn(y_lbl), run_time=0.9)

        # Real anchors (turn_rate, travel_speed) from specs/demo_walk_2d.pmg_spec.
        bottom = [(-0.3, 0.0), (0.0, 0.0), (1.0, 0.0)]
        fast = (0.15, 0.75)
        anchors = VGroup(*[
            Dot(axes.c2p(x, y), radius=0.085, color=ACCENT)
            for x, y in bottom
        ])
        fast_dot = Dot(axes.c2p(*fast), radius=0.085, color=ACCENT2)

        # Supported region = convex hull of the four anchors (a triangle).
        hull = Polygon(
            axes.c2p(-0.3, 0.0), axes.c2p(1.0, 0.0), axes.c2p(*fast),
            color=ACCENT, stroke_width=2, fill_color=SHADE, fill_opacity=0.5,
        )
        region_lbl = _label("supported region", 20, ACCENT)
        region_lbl.move_to(axes.c2p(0.4, 0.42))

        self.play(Create(hull), FadeIn(region_lbl), run_time=0.8)
        self.play(LaggedStart(*[GrowFromCenter(a) for a in anchors],
                              lag_ratio=0.2), run_time=0.7)
        self.play(GrowFromCenter(fast_dot), run_time=0.4)

        # Primary axis: three anchors span a wide turn-rate sweep at speed 0.
        sweep = DoubleArrow(axes.c2p(-0.3, -0.0), axes.c2p(1.0, -0.0),
                            buff=0.0, color=ACCENT, stroke_width=3,
                            tip_length=0.18).shift(DOWN * 0.28)
        sweep_lbl = _label("primary turn-rate sweep (3 examples)", 18, ACCENT)
        sweep_lbl.next_to(sweep, DOWN, buff=0.12)

        # Secondary axis: a single faster example -> limited support.
        limit_lbl = _label("limited speed support\n(1 example)", 18, ACCENT2)
        limit_lbl.move_to(axes.c2p(0.15, 0.75) + RIGHT * 2.0)
        limit_arrow = Arrow(limit_lbl.get_left(), fast_dot.get_right(),
                            buff=0.12, color=ACCENT2, stroke_width=2.5,
                            max_tip_length_to_length_ratio=0.3)

        self.play(GrowFromCenter(sweep), FadeIn(sweep_lbl), run_time=0.7)
        self.play(FadeIn(limit_lbl), GrowArrow(limit_arrow), run_time=0.7)

        # Query is a point inside the supported region (purple = neutral).
        query = Dot(axes.c2p(0.3, 0.22), radius=0.1, color=QUERY)
        query_lbl = _label("query parameter\n(stays inside region)", 18, QUERY)
        query_lbl.move_to(axes.c2p(0.3, 0.22) + LEFT * 2.1 + DOWN * 0.05)
        q_arrow = Arrow(query_lbl.get_right(), query.get_left(), buff=0.12,
                        color=QUERY, stroke_width=2.5,
                        max_tip_length_to_length_ratio=0.3)
        self.play(GrowFromCenter(query), FadeIn(query_lbl),
                  GrowArrow(q_arrow), run_time=0.7)
        self.wait(1.8)


class EdgeSampling(Scene):
    """Edge construction: candidate matrix, Kovar-style scoring, accept on count."""

    def construct(self):
        _white_bg(self)

        title = _label("Building an edge: sample candidate transitions", 30, INK, BOLD)
        title.move_to([0, 3.4, 0])
        self.play(FadeIn(title, shift=DOWN * 0.2), run_time=0.6)

        n = 4  # source end-frames (rows) x target start-frames (cols)
        cell = 0.62
        grid_origin = [-3.4, 0.6, 0]  # top-left cell centre

        def cell_pos(i, j):  # i = source row (down), j = target col (right)
            return [grid_origin[0] + j * cell,
                    grid_origin[1] - i * cell, 0]

        # Quality class per (source_frame, target_frame) candidate pair.
        # Aligned frames (near-diagonal) score well; far pairs score poorly.
        G, N, B = GOOD, NEUTRAL, BAD
        classes = [
            [G, G, N, B],
            [G, G, G, N],
            [N, G, G, G],
            [B, N, G, G],
        ]
        cells = VGroup()
        for i in range(n):
            for j in range(n):
                c = classes[i][j]
                sq = Square(side_length=cell * 0.86, color=c, stroke_width=2,
                            fill_color=c, fill_opacity=0.85)
                sq.move_to(cell_pos(i, j))
                cells.add(sq)

        # Axis labels for the matrix.
        src_lbl = _label("source\nend-window", 20, ACCENT)
        src_lbl.move_to([grid_origin[0] - 1.6, grid_origin[1] - cell * 1.5, 0])
        tgt_lbl = _label("target start-window", 20, ACCENT2)
        tgt_lbl.move_to([grid_origin[0] + cell * 1.5, grid_origin[1] + 0.85, 0])
        src_tick = Arrow([grid_origin[0] - 0.55, grid_origin[1] + 0.1, 0],
                         [grid_origin[0] - 0.55, grid_origin[1] - cell * 3, 0],
                         buff=0.0, color=ACCENT, stroke_width=2.5,
                         max_tip_length_to_length_ratio=0.08)
        tgt_tick = Arrow([grid_origin[0] - 0.1, grid_origin[1] + 0.5, 0],
                         [grid_origin[0] + cell * 3, grid_origin[1] + 0.5, 0],
                         buff=0.0, color=ACCENT2, stroke_width=2.5,
                         max_tip_length_to_length_ratio=0.08)

        self.play(FadeIn(src_lbl), FadeIn(tgt_lbl),
                  GrowArrow(src_tick), GrowArrow(tgt_tick), run_time=0.7)
        self.play(LaggedStart(*[GrowFromCenter(s) for s in cells],
                              lag_ratio=0.04), run_time=1.2)

        # Right-side legend: Kovar-style distance -> quality class.
        leg_x = 3.0
        leg_top = 1.7
        legend_title = _label("Kovar-style distance", 20, INK, BOLD)
        legend_title.move_to([leg_x + 0.9, leg_top, 0])
        rows = [("low  -> accept", GOOD),
                ("mid  -> borderline", NEUTRAL),
                ("high -> reject", BAD)]
        legend = VGroup()
        for k, (txt, col) in enumerate(rows):
            y = leg_top - 0.55 - k * 0.55
            swatch = Square(side_length=0.34, color=col, stroke_width=2,
                            fill_color=col, fill_opacity=0.85)
            swatch.move_to([leg_x, y, 0])
            t = _label(txt, 18, INK).next_to(swatch, RIGHT, buff=0.2)
            legend.add(VGroup(swatch, t))
        self.play(FadeIn(legend_title), LaggedStart(
            *[FadeIn(g, shift=LEFT * 0.15) for g in legend], lag_ratio=0.15),
            run_time=0.9)
        self.wait(0.3)

        # Bottom decision box: enough GOOD samples -> edge enabled.
        n_good = sum(row.count(GOOD) for row in classes)
        box = RoundedRectangle(width=8.6, height=0.9, corner_radius=0.12,
                               color=GOOD, stroke_width=3)
        box.move_to([0, -2.9, 0])
        decision = _label(
            f"{n_good} GOOD samples >= threshold   ->   edge enabled", 22,
            GOOD, BOLD).move_to(box.get_center())
        self.play(Create(box), FadeIn(decision, shift=UP * 0.15), run_time=0.8)
        self.wait(1.6)
