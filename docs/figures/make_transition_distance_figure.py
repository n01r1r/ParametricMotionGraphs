"""Publication figure: within-gait vs cross-gait transition distance.

Proves the honest submission claim: within-gait (walk<->walk) transitions are
low-distance / safe, cross-gait transitions are high-distance / coverage-limited.

Data: existing CLI diagnostic CSV dumps (transition_distance = column index 19).
Run: python docs/figures/make_transition_distance_figure.py
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
DIST_COL = 19  # transition_distance, verified from CSV header

# (label, csv path, color) -- within-gait = subj16 walk, cross-gait = cmu127 graph
SERIES = [
    ("Within-gait\n(walk↔walk, subj16)", "experiments/cmu/out/walk_transitions.csv", "#2b8cbe"),
    ("Cross-gait\n(walk↔run, cmu127)", "outputs/cmu127_gait_graph/rw_trans.csv", "#e34a33"),
]


def load_distances(rel_path):
    path = ROOT / rel_path
    vals = []
    with open(path, newline="") as fh:
        reader = csv.reader(fh)
        next(reader)  # header
        for row in reader:
            if len(row) > DIST_COL and row[DIST_COL]:
                vals.append(float(row[DIST_COL]))
    if not vals:
        sys.exit(f"no distances in {rel_path}")
    return vals


def main():
    data = [(lbl, load_distances(p), c) for lbl, p, c in SERIES]

    plt.rcParams.update({"font.size": 11})
    fig, ax = plt.subplots(figsize=(5.0, 4.2))

    positions = range(len(data))
    box = ax.boxplot([d for _, d, _ in data], positions=list(positions),
                     widths=0.5, showfliers=False, patch_artist=True,
                     medianprops=dict(color="black"))
    for patch, (_, _, color) in zip(box["boxes"], data):
        patch.set_facecolor(color)
        patch.set_alpha(0.35)

    import random
    random.seed(0)
    for x, (_, vals, color) in zip(positions, data):
        jitter = [x + random.uniform(-0.12, 0.12) for _ in vals]
        ax.scatter(jitter, vals, s=22, color=color, edgecolor="white",
                   linewidth=0.4, zorder=3)
        med = sorted(vals)[len(vals) // 2]
        ax.annotate(f"median\n{med:.0f}", (x, med), textcoords="offset points",
                    xytext=(34, 0), va="center", fontsize=9, color=color)

    ax.set_yscale("log")
    ax.set_xticks(list(positions))
    ax.set_xticklabels([lbl for lbl, _, _ in data])
    ax.set_ylabel("Transition distance (Kovar metric, log scale)")
    ax.set_title("Transition cost: within-gait vs cross-gait", fontsize=12)
    ax.grid(axis="y", alpha=0.3, which="both")
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    fig.tight_layout()

    out = ROOT / "docs/figures/transition_distance_within_vs_cross.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"wrote {out}")
    for lbl, vals, _ in data:
        s = sorted(vals)
        print(f"  {lbl.splitlines()[0]:22s} n={len(vals):3d} "
              f"median={s[len(s)//2]:8.1f} min={s[0]:8.1f} max={s[-1]:8.1f}")


if __name__ == "__main__":
    main()
