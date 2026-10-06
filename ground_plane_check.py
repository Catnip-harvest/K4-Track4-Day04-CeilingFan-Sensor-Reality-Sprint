"""Estimate injected LiDAR roll/pitch from the road plane in KITTI scans.

Example:
    python ground_plane_check.py --drives 0048 0005 0052
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from t3calib.geometry import Drift


ROOT = Path(__file__).parent
MAGNITUDES_DEG = (0.0, 0.5, 1.0, 2.0)


def ground_candidates(points: np.ndarray) -> np.ndarray:
    """Keep a broad road ROI; robust fitting removes vehicles and kerbs."""
    mask = (
        (points[:, 0] > 2.0)
        & (points[:, 0] < 40.0)
        & (np.abs(points[:, 1]) < 10.0)
        & (points[:, 2] > -3.0)
        & (points[:, 2] < -0.5)
    )
    return points[mask, :3]


def fit_ground_normal(points: np.ndarray, minimum_points: int = 200) -> np.ndarray:
    """Fit z = ax + by + c and return its upward unit normal."""
    selected = ground_candidates(points)
    if len(selected) < minimum_points:
        raise ValueError(f"only {len(selected)} ground candidates")

    inliers = np.ones(len(selected), dtype=bool)
    for _ in range(4):
        xyz = selected[inliers]
        coefficients = np.linalg.lstsq(np.c_[xyz[:, :2], np.ones(len(xyz))], xyz[:, 2], rcond=None)[0]
        residuals = selected[:, 2] - (np.c_[selected[:, :2], np.ones(len(selected))] @ coefficients)
        centre = np.median(residuals)
        mad = np.median(np.abs(residuals - centre))
        threshold = max(0.05, 3.0 * 1.4826 * mad)
        updated = np.abs(residuals - centre) <= threshold
        if updated.sum() < minimum_points or np.array_equal(updated, inliers):
            break
        inliers = updated

    normal = np.array([-coefficients[0], -coefficients[1], 1.0])
    return normal / np.linalg.norm(normal)


def normal_roll_pitch(normal: np.ndarray) -> tuple[float, float]:
    """Roll and pitch represented by an upward plane normal, in degrees."""
    roll = np.degrees(np.arctan2(normal[1], normal[2]))
    pitch = np.degrees(np.arctan2(-normal[0], np.hypot(normal[1], normal[2])))
    return float(roll), float(pitch)


def rotate_scan_inverse(points: np.ndarray, drift: Drift) -> np.ndarray:
    """Express fixed world points in a LiDAR rotated by ``drift``."""
    rotation = drift.matrix()[:3, :3]
    rotated = points.copy()
    rotated[:, :3] = points[:, :3] @ rotation
    return rotated


def scan_paths(data_dir: Path, drive: str, frame_step: int) -> list[Path]:
    folder = data_dir / "2011_09_26" / f"2011_09_26_drive_{drive}_sync" / "velodyne_points" / "data"
    paths = sorted(folder.glob("*.bin"))[::frame_step]
    if not paths:
        raise FileNotFoundError(f"No scans in {folder}; run python get_data.py --drives {drive}")
    return paths


def evaluate_drive(data_dir: Path, drive: str, frame_step: int) -> list[dict[str, float | int | str]]:
    scans = [np.fromfile(path, dtype=np.float32).reshape(-1, 4).astype(np.float64) for path in scan_paths(data_dir, drive, frame_step)]
    baseline: list[tuple[float, float] | None] = []
    for scan in scans:
        try:
            baseline.append(normal_roll_pitch(fit_ground_normal(scan)))
        except ValueError:
            baseline.append(None)

    rows = []
    for axis in ("roll", "pitch"):
        for magnitude in MAGNITUDES_DEG:
            estimates = []
            for scan, reference in zip(scans, baseline):
                if reference is None:
                    continue
                try:
                    tilted = rotate_scan_inverse(scan, Drift.along(axis, magnitude))
                    measured = normal_roll_pitch(fit_ground_normal(tilted))
                except ValueError:
                    continue
                estimates.append((measured[0] - reference[0], measured[1] - reference[1]))
            if not estimates:
                raise RuntimeError(f"No usable frames for drive {drive}, {axis}={magnitude} deg")
            estimates_array = np.asarray(estimates)
            truth = np.array([magnitude if axis == "roll" else 0.0, magnitude if axis == "pitch" else 0.0])
            errors = estimates_array - truth
            rows.append({
                "drive": drive,
                "axis": axis,
                "injected_deg": magnitude,
                "frames_used": len(estimates),
                "estimated_roll_deg_median": np.median(estimates_array[:, 0]),
                "estimated_pitch_deg_median": np.median(estimates_array[:, 1]),
                "roll_error_deg_median": np.median(errors[:, 0]),
                "pitch_error_deg_median": np.median(errors[:, 1]),
                "absolute_error_deg_median": np.median(np.abs(errors[:, 0 if axis == "roll" else 1])),
            })
    return rows


def save_figure(results: pd.DataFrame, path: Path) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(10, 4), sharex=True, sharey=True)
    for plot, axis in zip(axes, ("roll", "pitch")):
        subset = results[results["axis"] == axis]
        estimate_column = f"estimated_{axis}_deg_median"
        for drive, group in subset.groupby("drive"):
            plot.plot(group["injected_deg"], group[estimate_column], marker="o", label=f"drive {drive}")
        plot.plot(MAGNITUDES_DEG, MAGNITUDES_DEG, "k--", linewidth=1, label="ideal")
        plot.set_title(axis.capitalize())
        plot.set_xlabel("Injected tilt (deg)")
        plot.grid(alpha=0.3)
    axes[0].set_ylabel("Estimated tilt, median (deg)")
    axes[1].legend()
    figure.suptitle("Ground-plane estimate of simulated LiDAR tilt")
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drives", nargs="+", default=["0048", "0005", "0052"])
    parser.add_argument("--frame-step", type=int, default=2, help="use every nth LiDAR frame")
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data")
    parser.add_argument("--out", type=Path, default=ROOT / "results" / "ground_plane")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    rows = []
    for drive in args.drives:
        drive_rows = evaluate_drive(args.data_dir, drive, args.frame_step)
        rows.extend(drive_rows)
        print(f"drive {drive}: {drive_rows[0]['frames_used']} usable frames")

    results = pd.DataFrame(rows)
    csv_path = args.out / "ground_plane.csv"
    figure_path = args.out / "ground_plane.png"
    results.to_csv(csv_path, index=False, float_format="%.4f")
    save_figure(results, figure_path)
    print(results.to_string(index=False, float_format=lambda value: f"{value:.3f}"))
    print(f"\nWrote {csv_path} and {figure_path}")


if __name__ == "__main__":
    main()
