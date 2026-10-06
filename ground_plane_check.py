"""Check the 20 held-out drifts using LiDAR road-plane roll/pitch.

    python ground_plane_check.py

The random cases are imported from validate_holdout.py so their seed and ordering cannot diverge.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from t3calib.geometry import Drift
from t3calib.kitti import Drive
from validate_holdout import FRAME_STEP, RANDOM_SEED, draw_random_drifts

ROOT = Path(__file__).parent
ROLL_DETECTION_DEG = 0.5


class Log:
    def __init__(self, path: Path):
        self.file = path.open("w", encoding="utf-8")

    def __call__(self, message: str = "") -> None:
        print(message, flush=True)
        self.file.write(message + "\n")
        self.file.flush()

    def close(self) -> None:
        self.file.close()


def ground_candidates(points: np.ndarray) -> np.ndarray:
    """Broad road ROI; RANSAC rejects vehicles, kerbs and other low outliers."""
    mask = (
        (points[:, 0] > 2.0)
        & (points[:, 0] < 40.0)
        & (np.abs(points[:, 1]) < 10.0)
        & (points[:, 2] > -3.0)
        & (points[:, 2] < -0.5)
    )
    return points[mask, :3]


def fit_ground_normal(
    points: np.ndarray,
    rng: np.random.Generator,
    iterations: int = 48,
    distance_threshold_m: float = 0.08,
    minimum_points: int = 200,
) -> np.ndarray:
    """Fit a plane with deterministic NumPy RANSAC and return its upward unit normal."""
    selected = ground_candidates(points)
    if len(selected) < minimum_points:
        raise ValueError(f"only {len(selected)} ground candidates")

    best_inliers: np.ndarray | None = None
    best_count = 0
    for _ in range(iterations):
        sample = selected[rng.choice(len(selected), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9 or abs(normal[2] / norm) < 0.8:
            continue
        normal /= norm
        distances = np.abs((selected - sample[0]) @ normal)
        inliers = distances < distance_threshold_m
        count = int(inliers.sum())
        if count > best_count:
            best_count, best_inliers = count, inliers

    if best_inliers is None or best_count < minimum_points:
        raise ValueError(f"RANSAC retained only {best_count} ground points")

    inlier_points = selected[best_inliers]
    coefficients = np.linalg.lstsq(
        np.c_[inlier_points[:, :2], np.ones(len(inlier_points))], inlier_points[:, 2], rcond=None
    )[0]
    normal = np.array([-coefficients[0], -coefficients[1], 1.0])
    return normal / np.linalg.norm(normal)


def normal_roll_pitch(normal: np.ndarray) -> tuple[float, float]:
    """Roll and pitch represented by an upward plane normal in LiDAR axes."""
    roll = np.degrees(np.arctan2(normal[1], normal[2]))
    pitch = np.degrees(np.arctan2(-normal[0], np.hypot(normal[1], normal[2])))
    return float(roll), float(pitch)


def rotate_scan_inverse(points: np.ndarray, drift: Drift) -> np.ndarray:
    """Express fixed world points in a LiDAR rotated by the drift; translation does not affect a normal."""
    rotation = drift.matrix()[:3, :3]
    rotated = points.copy()
    rotated[:, :3] = points[:, :3] @ rotation
    return rotated


def angular_distance(first: np.ndarray, second: np.ndarray) -> float:
    cosine = np.clip(np.dot(first, second), -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def evaluate(drive: Drive, log: Log) -> pd.DataFrame:
    frame_indices = drive.frame_indices[::FRAME_STEP]
    scans = [
        np.fromfile(
            drive.sync_dir / "velodyne_points" / "data" / f"{index:010d}.bin", dtype=np.float32
        ).reshape(-1, 4).astype(np.float64)
        for index in frame_indices
    ]
    lidar_to_camera_rotation = drive.calibration.velo_to_cam[:3, :3]

    baseline: list[tuple[np.ndarray, tuple[float, float]] | None] = []
    for position, scan in enumerate(scans):
        try:
            normal = fit_ground_normal(scan, np.random.default_rng(RANDOM_SEED + position))
            baseline.append((normal, normal_roll_pitch(normal)))
        except ValueError as error:
            log(f"skip baseline frame {frame_indices[position]}: {error}")
            baseline.append(None)

    rows: list[dict[str, float | int | bool]] = []
    for drift_id, drift in enumerate(draw_random_drifts()):
        measurements = []
        for position, (scan, reference) in enumerate(zip(scans, baseline)):
            if reference is None:
                continue
            baseline_normal, baseline_angles = reference
            try:
                simulated = rotate_scan_inverse(scan, drift)
                normal = fit_ground_normal(
                    simulated, np.random.default_rng(RANDOM_SEED * 100_000 + drift_id * 1_000 + position)
                )
            except ValueError as error:
                log(f"skip drift {drift_id} frame {frame_indices[position]}: {error}")
                continue

            roll, pitch = normal_roll_pitch(normal)
            normal_camera = lidar_to_camera_rotation @ normal
            baseline_camera = lidar_to_camera_rotation @ baseline_normal
            measurements.append(
                (
                    roll - baseline_angles[0],
                    pitch - baseline_angles[1],
                    angular_distance(baseline_camera, normal_camera),
                    *normal_camera,
                )
            )

        if not measurements:
            raise RuntimeError(f"No usable frames for drift {drift_id}")
        values = np.asarray(measurements)
        roll_estimate, pitch_estimate = np.median(values[:, :2], axis=0)
        row = {
            "drift_id": drift_id,
            "injected_roll_deg": drift.roll,
            "injected_pitch_deg": drift.pitch,
            "injected_yaw_deg": drift.yaw,
            "injected_tx_m": drift.tx,
            "injected_ty_m": drift.ty,
            "injected_tz_m": drift.tz,
            "frames_used": len(values),
            "estimated_roll_deg_median": roll_estimate,
            "estimated_pitch_deg_median": pitch_estimate,
            "roll_error_deg": roll_estimate - drift.roll,
            "pitch_error_deg": pitch_estimate - drift.pitch,
            "camera_normal_change_deg_median": np.median(values[:, 2]),
            "camera_normal_x_median": np.median(values[:, 3]),
            "camera_normal_y_median": np.median(values[:, 4]),
            "camera_normal_z_median": np.median(values[:, 5]),
            "roll_detected": abs(roll_estimate) >= ROLL_DETECTION_DEG,
        }
        rows.append(row)
        log(
            f"drift {drift_id:2d}: roll {drift.roll:+.3f} -> {roll_estimate:+.3f} deg, "
            f"pitch {drift.pitch:+.3f} -> {pitch_estimate:+.3f} deg, "
            f"camera-normal change {row['camera_normal_change_deg_median']:.3f} deg, "
            f"roll detected={row['roll_detected']}"
        )
    return pd.DataFrame(rows)


def save_figure(results: pd.DataFrame, path: Path) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(10, 4.2), sharex=True, sharey=True)
    for plot, axis in zip(axes, ("roll", "pitch")):
        actual = results[f"injected_{axis}_deg"]
        estimated = results[f"estimated_{axis}_deg_median"]
        plot.scatter(actual, estimated, color="tab:blue")
        limits = (-1.65, 1.65)
        plot.plot(limits, limits, "k--", linewidth=1, label="ideal")
        case_14 = results[results["drift_id"] == 14].iloc[0]
        plot.annotate(
            "drift 14",
            (case_14[f"injected_{axis}_deg"], case_14[f"estimated_{axis}_deg_median"]),
            xytext=(6, 6), textcoords="offset points",
        )
        plot.set(xlim=limits, ylim=limits, title=axis.capitalize(), xlabel="Injected angle (deg)")
        plot.grid(alpha=0.3)
    axes[0].set_ylabel("Ground-plane estimate, median (deg)")
    axes[1].legend()
    figure.suptitle("KITTI drive 0052 — 20 held-out drifts (seed 7)")
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drive", default="0052")
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data")
    parser.add_argument("--out", type=Path, default=ROOT / "results" / "ground_check")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    log = Log(args.out / "ground_check_log.txt")
    started = time.time()
    log(f"ground-plane check | python {sys.version.split()[0]} | numpy {np.__version__}")
    log(f"drive {args.drive} | every {FRAME_STEP} frame | 20 drifts from validate_holdout.py | seed {RANDOM_SEED}")
    log(f"roll detection rule: |estimated roll| >= {ROLL_DETECTION_DEG:.1f} deg")
    results = evaluate(Drive(args.data_dir, args.drive), log)

    csv_path = args.out / "ground_check.csv"
    figure_path = args.out / "ground_check.png"
    results.to_csv(csv_path, index=False, float_format="%.4f")
    save_figure(results, figure_path)

    case_14 = results.loc[results["drift_id"] == 14].iloc[0]
    log()
    log(
        f"case 14: injected roll {case_14['injected_roll_deg']:+.4f} deg; "
        f"estimated {case_14['estimated_roll_deg_median']:+.4f} deg; "
        f"error {case_14['roll_error_deg']:+.4f} deg; detected={case_14['roll_detected']}"
    )
    log(f"median |roll error|: {results['roll_error_deg'].abs().median():.4f} deg")
    log(f"median |pitch error|: {results['pitch_error_deg'].abs().median():.4f} deg")
    log(f"runtime: {time.time() - started:.1f} s")
    log(f"wrote {csv_path} and {figure_path}")
    log.close()


if __name__ == "__main__":
    main()
