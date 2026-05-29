import argparse
from pathlib import Path

import matplotlib.pyplot as plt


def read_ascii_ply(path: Path) -> tuple[list[float], list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    intensities: list[float] = []

    with path.open("r", encoding="utf-8") as file:
        for line in file:
            if line.strip() == "end_header":
                break

        for line in file:
            parts = line.split()
            if len(parts) < 4:
                continue
            x, y, _z, intensity = map(float, parts[:4])
            xs.append(x)
            ys.append(y)
            intensities.append(intensity)

    return xs, ys, intensities


def render_bev(input_path: Path, output_path: Path, limit: float) -> None:
    xs, ys, intensities = read_ascii_ply(input_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(7, 7), dpi=160)
    scatter = ax.scatter(
        xs,
        ys,
        c=intensities,
        s=0.8,
        cmap="viridis",
        alpha=0.85,
        linewidths=0,
    )
    ax.scatter([0], [0], marker="^", s=70, c="white", edgecolors="black", label="ego")
    ax.set_title("LiDAR BEV Point Cloud")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")
    fig.colorbar(scatter, ax=ax, label="intensity")
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render a CARLA ASCII PLY LiDAR frame as a BEV PNG.")
    parser.add_argument("input", type=Path, help="Input .ply point cloud.")
    parser.add_argument("output", type=Path, help="Output .png image.")
    parser.add_argument("--limit", default=50.0, type=float, help="Axis limit in meters.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    render_bev(args.input, args.output, args.limit)


if __name__ == "__main__":
    main()

