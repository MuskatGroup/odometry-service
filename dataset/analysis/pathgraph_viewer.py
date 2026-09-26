"""Interactive Matplotlib viewer for the organizer-provided Pathgraph files."""

from __future__ import annotations

import argparse
from collections.abc import Iterable
from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib
import numpy as np
from pathgraph_analysis import PATHGRAPH_ROOT, ROUTE_FILE_NAMES, PathGraphRoute, load_pathgraph

if TYPE_CHECKING:
    from matplotlib.figure import Figure


ROUTE_LABELS = {
    "tallinskaya_to_shchukinskaya": "Таллинская → Щукинская",
    "shchukinskaya_to_tallinskaya": "Щукинская → Таллинская",
}


def _selected_routes(route_ids: Iterable[str], pathgraph_root: Path) -> list[PathGraphRoute]:
    return [load_pathgraph(route_id, pathgraph_root=pathgraph_root) for route_id in route_ids]


def create_pathgraph_figure(routes: list[PathGraphRoute]) -> Figure:
    """Create a route plan plus elevation and curvature profiles.

    Click near a route in the plan to inspect the nearest sampled point.
    Standard Matplotlib pan and zoom tools remain available.
    """

    import matplotlib.pyplot as plt

    figure = plt.figure(figsize=(14, 8), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, height_ratios=(1.25, 1.0))
    plan_axis = figure.add_subplot(grid[0, :])
    elevation_axis = figure.add_subplot(grid[1, 0])
    curvature_axis = figure.add_subplot(grid[1, 1], sharex=elevation_axis)

    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    line_routes: dict[object, PathGraphRoute] = {}

    for index, route in enumerate(routes):
        frame = route.points
        color = colors[index % len(colors)]
        label = ROUTE_LABELS.get(route.route_id, route.route_id)
        (line,) = plan_axis.plot(frame["x"], frame["y"], color=color, linewidth=1.5, label=label)
        line_routes[line] = route
        plan_axis.scatter(frame["x"].iloc[0], frame["y"].iloc[0], color=color, marker="o", s=55)
        plan_axis.scatter(frame["x"].iloc[-1], frame["y"].iloc[-1], color=color, marker="X", s=65)

        arrow_indices = np.linspace(0, len(frame) - 1, 10, dtype=int)
        plan_axis.quiver(
            frame["x"].iloc[arrow_indices],
            frame["y"].iloc[arrow_indices],
            np.cos(frame["yaw_rad"].iloc[arrow_indices]),
            np.sin(frame["yaw_rad"].iloc[arrow_indices]),
            color=color,
            angles="xy",
            scale_units="xy",
            scale=0.015,
            width=0.003,
        )
        elevation_axis.plot(frame["s_m"], frame["z"], color=color, linewidth=1.2, label=label)
        curvature_axis.plot(
            frame["s_m"], frame["curvature_inv_m"], color=color, linewidth=1.0, label=label
        )

    plan_axis.set_title("Pathgraph: план маршрута")
    plan_axis.set_xlabel("x, м")
    plan_axis.set_ylabel("y, м")
    plan_axis.set_aspect("equal", adjustable="box")
    plan_axis.grid(True, alpha=0.25)
    plan_axis.legend(loc="best")

    elevation_axis.set_title("Профиль высоты")
    elevation_axis.set_ylabel("z, м")
    elevation_axis.grid(True, alpha=0.25)
    elevation_axis.legend(loc="best")

    curvature_axis.set_title("Кривизна")
    curvature_axis.set_xlabel("s, м")
    curvature_axis.set_ylabel("curvature, 1/м")
    curvature_axis.grid(True, alpha=0.25)

    annotation = plan_axis.annotate(
        "",
        xy=(0.0, 0.0),
        xytext=(12, 12),
        textcoords="offset points",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9},
        arrowprops={"arrowstyle": "->"},
    )
    annotation.set_visible(False)

    def on_click(event: object) -> None:
        if getattr(event, "inaxes", None) is not plan_axis:
            return
        x_value = getattr(event, "xdata", None)
        y_value = getattr(event, "ydata", None)
        if x_value is None or y_value is None:
            return

        best: tuple[float, PathGraphRoute, int] | None = None
        for route in line_routes.values():
            frame = route.points
            distance_sq = (frame["x"].to_numpy() - x_value) ** 2 + (
                frame["y"].to_numpy() - y_value
            ) ** 2
            nearest_index = int(np.argmin(distance_sq))
            candidate = (float(distance_sq[nearest_index]), route, nearest_index)
            if best is None or candidate[0] < best[0]:
                best = candidate

        if best is None:
            return
        _, route, nearest_index = best
        point = route.points.iloc[nearest_index]
        annotation.xy = (point["x"], point["y"])
        annotation.set_text(
            f"{ROUTE_LABELS.get(route.route_id, route.route_id)}\n"
            f"index={int(point['point_index'])}, s={point['s_m']:.1f} м\n"
            f"x={point['x']:.2f}, y={point['y']:.2f}, z={point['z']:.2f}\n"
            f"yaw={point['yaw_rad']:.4f} рад, curv={point['curvature_inv_m']:.5f} 1/м"
        )
        annotation.set_visible(True)
        figure.canvas.draw_idle()

    figure.canvas.mpl_connect("button_press_event", on_click)
    return figure


def show_pathgraphs(
    route_ids: Iterable[str] = ROUTE_FILE_NAMES,
    *,
    pathgraph_root: Path = PATHGRAPH_ROOT,
) -> Figure:
    """Load selected routes, create a figure and display it."""

    import matplotlib.pyplot as plt

    figure = create_pathgraph_figure(_selected_routes(route_ids, pathgraph_root))
    plt.show()
    return figure


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--route",
        action="append",
        choices=sorted(ROUTE_FILE_NAMES),
        help="Route ID to display. Repeat the option for multiple routes; default: both.",
    )
    parser.add_argument(
        "--pathgraph-dir",
        type=Path,
        default=PATHGRAPH_ROOT,
        help=f"Directory containing Pathgraph JSON files (default: {PATHGRAPH_ROOT}).",
    )
    parser.add_argument("--save", type=Path, help="Save the figure to PNG/SVG/PDF.")
    parser.add_argument("--no-show", action="store_true", help="Do not open an interactive window.")
    parser.add_argument(
        "--backend",
        default="WebAgg",
        help="Matplotlib backend for interactive CLI display (default: WebAgg opens in a browser).",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    matplotlib.use("Agg" if args.no_show else args.backend)
    import matplotlib.pyplot as plt

    route_ids = args.route or list(ROUTE_FILE_NAMES)
    routes = _selected_routes(route_ids, args.pathgraph_dir)
    figure = create_pathgraph_figure(routes)
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.save, dpi=160)
        print(f"Saved Pathgraph figure to {args.save.resolve()}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
