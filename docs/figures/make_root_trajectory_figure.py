"""Publication figure: top-down root trajectory of a graph random-walk.

Reproducible from tracked data: regenerates the motion CSV by running pmg_cli
on specs/cmu78_gait_graph.pmg_spec (CMU subject 78, clips tracked in BVH/),
then plots the synthesized root path (x,z) colored by active graph node with
transition points marked. Clean version of the viewer's T2 trajectory fan.

Run: python docs/figures/make_root_trajectory_figure.py
Requires: pmg_cli built (cmake --build build --target pmg_cli).
"""
import csv
import subprocess
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
SPEC = "specs/cmu78_gait_graph.pmg_spec"
NODE_COLORS = {0: "#2b8cbe", 1: "#e6550d"}
NODE_LABELS = {0: "node 0 (walk)", 1: "node 1 (run)"}


def find_cli():
    for p in ("build/Release/pmg_cli.exe", "build/Debug/pmg_cli.exe",
              "build/pmg_cli", "build/Release/pmg_cli", "build/Debug/pmg_cli"):
        if (ROOT / p).exists():
            return str(ROOT / p)
    sys.exit("pmg_cli not found - build it: cmake --build build --target pmg_cli")


def regenerate_motion_csv(out_csv):
    subprocess.run([find_cli(), "--random-walk", str(ROOT / SPEC),
                    "--seconds", "20", "--walk-seed", "1",
                    "--dump-motion-csv", out_csv],
                   cwd=ROOT, check=True, stdout=subprocess.DEVNULL)


def main():
    with tempfile.TemporaryDirectory() as tmp:
        motion_csv = str(Path(tmp) / "motion.csv")
        regenerate_motion_csv(motion_csv)
        xs, zs, nodes, trans = [], [], [], []
        with open(motion_csv, newline="") as fh:
            for row in csv.DictReader(fh):
                xs.append(float(row["root_x"]))
                zs.append(float(row["root_z"]))
                nodes.append(int(row["node"]))
                trans.append(int(row["transitioning"]))

    fig, ax = plt.subplots(figsize=(5.2, 5.0))
    plt.rcParams.update({"font.size": 11})

    seen = set()
    for i in range(1, len(xs)):
        n = nodes[i]
        lbl = NODE_LABELS[n] if n not in seen else None
        seen.add(n)
        ax.plot(xs[i - 1:i + 1], zs[i - 1:i + 1], color=NODE_COLORS.get(n, "gray"),
                linewidth=2.0, solid_capstyle="round", label=lbl)

    tx = [xs[i] for i in range(len(xs)) if trans[i]]
    tz = [zs[i] for i in range(len(zs)) if trans[i]]
    ax.scatter(tx, tz, s=40, facecolor="none", edgecolor="black",
               linewidth=1.0, zorder=5, label=f"transition ({len(tx)} frames)")
    ax.scatter([xs[0]], [zs[0]], s=70, color="black", marker="o", zorder=6, label="start")

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("root x (cm)")
    ax.set_ylabel("root z (cm)")
    ax.set_title("Synthesized root trajectory (graph random-walk, subj78)", fontsize=11)
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9, framealpha=0.9)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    fig.tight_layout()

    out = ROOT / "docs/figures/root_trajectory_subj78.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"wrote {out}  ({len(xs)} frames, {len(tx)} transition frames)")


if __name__ == "__main__":
    main()
