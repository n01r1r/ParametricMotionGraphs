"""Publication figure: graph-build edge acceptance by gait pair.

Reproducible from tracked data: regenerates tables/edge_samples.csv by running
pmg_cli --build-graph on specs/cmu_gait_graph.pmg_spec (CMU subject 16, clips
tracked in BVH/), then plots stacked good/neutral/bad transition-sample counts
per edge category with rejected edges annotated. Honest evidence: within-gait
(walk->walk, run->run) is clean; cross-gait (esp. run->walk) degrades and some
edges are rejected ("no GOOD target samples").

Run: python docs/figures/make_edge_acceptance_figure.py
Requires: pmg_cli built (cmake --build build --target pmg_cli).
"""
import csv
import subprocess
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
SPEC = "specs/cmu_gait_graph.pmg_spec"
ORDER = ["walk_cmu->walk_cmu", "run_cmu->run_cmu", "walk_cmu->run_cmu", "run_cmu->walk_cmu"]
PRETTY = {"walk_cmu->walk_cmu": "walk->walk", "run_cmu->run_cmu": "run->run",
          "walk_cmu->run_cmu": "walk->run", "run_cmu->walk_cmu": "run->walk"}


def find_cli():
    for p in ("build/Release/pmg_cli.exe", "build/Debug/pmg_cli.exe",
              "build/pmg_cli", "build/Release/pmg_cli", "build/Debug/pmg_cli"):
        if (ROOT / p).exists():
            return str(ROOT / p)
    sys.exit("pmg_cli not found - build it: cmake --build build --target pmg_cli")


def regenerate_edge_samples(out_dir):
    subprocess.run([find_cli(), "--build-graph", str(ROOT / SPEC),
                    str(Path(out_dir) / "graph.pmg")],
                   cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
    return Path(out_dir) / "tables" / "edge_samples.csv"


def main():
    agg = defaultdict(lambda: {"good": 0, "neutral": 0, "bad": 0, "rejected": 0})
    with tempfile.TemporaryDirectory() as tmp:
        edge_csv = regenerate_edge_samples(tmp)
        with open(edge_csv, newline="") as fh:
            for row in csv.DictReader(fh):
                e = agg[row["edge"]]
                e["good"] += int(row["good"])
                e["neutral"] += int(row["neutral"])
                e["bad"] += int(row["bad"])
                if row["accepted"] == "0":
                    e["rejected"] += 1

    edges = [e for e in ORDER if e in agg]
    good = [agg[e]["good"] for e in edges]
    neutral = [agg[e]["neutral"] for e in edges]
    bad = [agg[e]["bad"] for e in edges]
    rejected = [agg[e]["rejected"] for e in edges]
    labels = [PRETTY[e] for e in edges]

    fig, ax = plt.subplots(figsize=(5.6, 4.2))
    plt.rcParams.update({"font.size": 11})
    x = range(len(edges))
    ax.bar(x, good, label="good", color="#31a354")
    ax.bar(x, neutral, bottom=good, label="neutral", color="#fec44f")
    ax.bar(x, bad, bottom=[g + n for g, n in zip(good, neutral)], label="bad", color="#de2d26")

    ax.set_ylim(0, max(g + n + b for g, n, b in zip(good, neutral, bad)) * 1.22)
    for xi, (g, n, b, rej) in enumerate(zip(good, neutral, bad, rejected)):
        if rej:
            ax.annotate(f"{rej} edge(s)\nrejected", (xi, g + n + b),
                        textcoords="offset points", xytext=(0, 5), ha="center",
                        fontsize=8.5, color="#de2d26", fontweight="bold")

    ax.set_xticks(list(x))
    ax.set_xticklabels(labels)
    ax.set_ylabel("transition samples")
    ax.set_title("Edge acceptance by gait pair (graph build, subj16)", fontsize=11)
    ax.legend(loc="upper left", fontsize=9)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    fig.tight_layout()

    out = ROOT / "docs/figures/edge_acceptance_subj16.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"wrote {out}")
    for lbl, g, n, b, rej in zip(labels, good, neutral, bad, rejected):
        print(f"  {lbl:12s} good={g:3d} neutral={n:3d} bad={b:3d} rejected={rej}")


if __name__ == "__main__":
    main()
