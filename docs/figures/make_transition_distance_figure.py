"""Publication figure: within-gait vs cross-gait transition distance.

Reproducible from tracked data: regenerates a random-walk transitions CSV by
running pmg_cli on specs/cmu_gait_graph.pmg_spec (CMU subject 16, clips tracked
in BVH/), then splits per-transition Kovar distance by within-node
(walk<->walk, run<->run) vs cross-node (walk<->run). Honest evidence that
cross-gait transitions are systematically costlier than within-gait ones, on a
single subject.

Run: python docs/figures/make_transition_distance_figure.py
Requires: pmg_cli built (cmake --build build --target pmg_cli).
"""
import csv
import random
import subprocess
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
SPEC = "specs/cmu_gait_graph.pmg_spec"


def find_cli():
    for p in ("build/Release/pmg_cli.exe", "build/Debug/pmg_cli.exe",
              "build/pmg_cli", "build/Release/pmg_cli", "build/Debug/pmg_cli"):
        if (ROOT / p).exists():
            return str(ROOT / p)
    sys.exit("pmg_cli not found - build it: cmake --build build --target pmg_cli")


def regenerate_transitions_csv(out_csv):
    subprocess.run([find_cli(), "--random-walk", str(ROOT / SPEC),
                    "--seconds", "60", "--walk-seed", "1",
                    "--dump-transitions-csv", out_csv],
                   cwd=ROOT, check=True, stdout=subprocess.DEVNULL)


def load_split():
    """Return (within_node_distances, cross_node_distances)."""
    within, cross = [], []
    with tempfile.TemporaryDirectory() as tmp:
        trans_csv = str(Path(tmp) / "trans.csv")
        regenerate_transitions_csv(trans_csv)
        with open(trans_csv, newline="") as fh:
            for row in csv.DictReader(fh):
                d = row.get("transition_distance", "")
                if not d:
                    continue
                bucket = within if row["source_node"] == row["target_node"] else cross
                bucket.append(float(d))
    return within, cross


def main():
    within, cross = load_split()
    if not within or not cross:
        sys.exit(f"need both buckets; within={len(within)} cross={len(cross)} "
                 "(increase --seconds)")
    data = [("Within-gait\n(walk↔walk, run↔run)", within, "#2b8cbe"),
            ("Cross-gait\n(walk↔run)", cross, "#e34a33")]

    plt.rcParams.update({"font.size": 11})
    fig, ax = plt.subplots(figsize=(5.0, 4.2))
    positions = range(len(data))
    box = ax.boxplot([d for _, d, _ in data], positions=list(positions),
                     widths=0.5, showfliers=False, patch_artist=True,
                     medianprops=dict(color="black"))
    for patch, (_, _, color) in zip(box["boxes"], data):
        patch.set_facecolor(color)
        patch.set_alpha(0.35)

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
    ax.set_title("Transition cost: within-gait vs cross-gait (subj16)", fontsize=12)
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
