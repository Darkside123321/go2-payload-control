"""Visualize measured v2 state estimation and ROS 2 stop traces."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Patch
import numpy as np


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "reports" / "sensor_v2"
FIGURES = ROOT / "figures"
BLUE = "#5B83BC"
CORAL = "#FC9262"
YELLOW = "#FEDE8E"
DARK = "#252930"
STATE_COLORS = {"RUN": "#94BFDC", "STOPPING": YELLOW, "STOPPED": BLUE}


def trace(case):
    with (DATA / case / "trace.csv").open(newline="") as file:
        rows = list(csv.DictReader(file))
    result = json.loads((DATA / case / "summary.json").read_text())
    if not rows or not result["completed"] or len(rows) != result["cycle_count"]:
        raise ValueError(f"Incomplete input: {case}")
    mode = np.array([row["mode"] for row in rows])
    numeric = {key: np.array([float(row[key]) for row in rows])
               for key in rows[0] if key != "mode"}
    if not all(np.isfinite(values).all() for values in numeric.values()):
        raise ValueError(f"Non-finite input: {case}")
    if not np.all(np.diff(numeric["time_s"]) > 0):
        raise ValueError(f"Non-monotonic time: {case}")
    return mode, numeric, result


def apply_style(font_path):
    if font_path is not None:
        font_manager.fontManager.addfont(str(font_path))
    font_manager.findfont(
        font_manager.FontProperties(family="Arial", weight="bold"),
        fallback_to_default=False,
    )
    plt.rcParams.update({
        "font.family": "Arial", "font.weight": "bold", "font.size": 13,
        "axes.labelsize": 14, "axes.labelweight": "bold", "axes.titleweight": "bold",
        "axes.titlesize": 15, "axes.grid": False, "axes.linewidth": 1.4,
        "axes.spines.top": True, "axes.spines.right": True,
        "xtick.direction": "in", "ytick.direction": "in",
        "xtick.top": True, "ytick.right": True,
        "xtick.major.width": 1.4, "ytick.major.width": 1.4,
        "legend.frameon": False, "legend.fontsize": 11.5,
        "lines.linewidth": 3.0, "lines.linestyle": "-",
        "figure.facecolor": "white", "axes.facecolor": "white", "savefig.facecolor": "white",
        "svg.fonttype": "none", "pdf.fonttype": 42,
    })


def legend(ax, columns, handles=None):
    ax.legend(handles=handles, loc="lower left", bbox_to_anchor=(0, 1.015), borderaxespad=0,
              handlelength=1.45, columnspacing=1.0, ncol=columns)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arial-bold", type=Path, help="Path to the Arial Bold font")
    parser.add_argument("--tiff", action="store_true", help="Also export a 600 dpi TIFF")
    args = parser.parse_args()
    apply_style(args.arial_bold)

    # Quantitative grid. The loaded trial compares the feedback estimate with simulator
    # truth, and the two ROS trials show the corresponding stop speed and controller states.
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 7.2))
    fig.subplots_adjust(left=0.117, right=0.981, bottom=0.105, top=0.851,
                        hspace=0.66, wspace=0.43)
    height, forward, speed, states = axes.ravel()

    _, loaded, result = trace("loaded")
    t = loaded["time_s"]
    for ax, truth, estimate, factor in (
        (height, "true_height_m", "estimated_height_m", 1000),
        (forward, "true_forward_mps", "estimated_forward_mps", 1),
    ):
        ax.plot(t, factor * loaded[truth], color=BLUE, label="MuJoCo truth")
        ax.plot(t, factor * loaded[estimate], color=CORAL, label="Estimate", linewidth=2.4)
        ax.set_xlim(0, result["sim_seconds"])
        ax.set_xticks([0, 2, 4, 6, 8, 10, 12])
        ax.set_xlabel("Simulation time (s)")
    forward.plot([0, 4, 4, 12], [0, 0, result["speed_command_mps"], result["speed_command_mps"]],
                 color=DARK, linewidth=1.8, label="Target", zorder=1)

    for row, (case, title, color) in enumerate((
        ("ros_zero", "Zero command", CORAL),
        ("ros_timeout", "Command timeout", BLUE),
    )):
        mode, data, result = trace(case)
        dds = json.loads((DATA / case / "dds.json").read_text())
        if not dds["passed"]:
            raise ValueError(f"DDS acceptance failed: {case}")
        relative_time = data["time_s"] - dds["first_stop_requested_sim_s"]
        speed.plot(relative_time, data["true_speed_mps"], label=title, color=color)
        boundaries = np.r_[0, np.flatnonzero(mode[1:] != mode[:-1]) + 1, len(mode)]
        edges = np.r_[relative_time, result["sim_seconds"] - dds["first_stop_requested_sim_s"]]
        for left, right in zip(boundaries[:-1], boundaries[1:]):
            states.barh(row, edges[right] - edges[left], left=edges[left], height=0.47,
                        color=STATE_COLORS[mode[left]])

    for ax in (speed, states):
        ax.set_xlim(-0.4, 2.0)
        ax.set_xticks([-0.4, 0, 0.4, 0.8, 1.2, 1.6, 2.0])
        ax.set_xlabel("Time from first stop status (s)")
    speed.set_ylim(bottom=0)
    states.set_yticks([0, 1], ["Zero command", "Timeout"])
    states.set_ylim(1.65, -0.65)

    titles = ("Base height", "Forward velocity", "Speed during stop", "Controller state")
    for index, (ax, title) in enumerate(zip(axes.ravel(), titles)):
        ax.set_title(f"{chr(97 + index)}   {title}", loc="left", pad=29)
    height.set_ylabel("Height (mm)")
    forward.set_ylabel("Forward speed (m/s)")
    speed.set_ylabel("Base speed (m/s)")
    for ax, columns in zip((height, forward, speed), (2, 3, 2)):
        legend(ax, columns)
    legend(states, 3, handles=[
        Patch(facecolor=color, label=mode) for mode, color in STATE_COLORS.items()
    ])

    prefix = FIGURES / "04_sensor_control_ros_stop"
    fig.savefig(prefix.with_suffix(".png"), dpi=300)
    fig.savefig(prefix.with_suffix(".svg"))
    fig.savefig(prefix.with_suffix(".pdf"))
    if args.tiff:
        fig.savefig(prefix.with_suffix(".tiff"), dpi=600, pil_kwargs={"compression": "tiff_lzw"})
    plt.close(fig)


if __name__ == "__main__":
    main()
