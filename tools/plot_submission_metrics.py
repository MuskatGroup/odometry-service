"""Save paired benchmark evidence and a figure; requires the analysis dependency group."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from compare_benchmarks import compare


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--label", choices=("validation", "test"), required=True)
    args = parser.parse_args()
    before = json.loads(args.before.read_text())
    after = json.loads(args.after.read_text())
    rows = [r for r in compare(before, after) if r["preset"] == "M1-cal"]
    if not rows:
        parser.error("No paired M1-cal results")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    args.output.mkdir(parents=True, exist_ok=True)
    for name, path, document in (("before", args.before, before), ("after", args.after, after)):
        document = {**document, "source_report_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        (args.output / f"{args.label}-{name}.json").write_text(
            json.dumps(document, indent=2, allow_nan=False) + "\n", encoding="utf-8",
        )
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 6), sharey=True, layout="constrained")
    for axis, metric, label in zip(
        axes, ("speed_rmse_mps", "position_rmse_m"), ("RMSE скорости, м/с", "RMSE пути, м"), strict=True,
    ):
        for key, offset, color, legend in (
            ("before", -0.19, "#c58a35", "До"), ("after", 0.19, "#287caa", "После"),
        ):
            values = [row[metric][key] for row in rows]
            if any(v is None for v in values):
                parser.error("Figure requires complete paired metrics")
            axis.barh([i + offset for i in range(len(rows))], values, height=0.36,
                      color=color, label=legend)
        axis.set_xlabel(label)
        axis.grid(axis="x", alpha=0.2)
        axis.set_axisbelow(True)
        axis.legend(loc="lower right")
    axes[0].set_yticks(range(len(rows)), [r["scenario"] for r in rows])
    axes[0].invert_yaxis()
    fig.suptitle(f"{args.label}: {rows[0]['bags']} записей, 120 с, без GNSS-коррекций\n"
                 "Среднее RMSE по записям; невалидные оценки включены")
    fig.savefig(args.output / f"{args.label}-comparison.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
