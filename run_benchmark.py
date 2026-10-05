"""T3 - Calibration drift impact on KITTI raw: one command produces every table, figure and log.

    python run_benchmark.py                   # drives 0048 + 0005 (the full benchmark)
    python run_benchmark.py --drives 0048     # quick run on the 22-frame drive

Stages
  1. sweep   perturb the camera-LiDAR extrinsic (rotation 0-3 deg, translation 0-20 cm) and measure
             reprojection error, LiDAR-to-box association and late-fusion range misses
  2. monitor targetless health check (Levinson & Thrun 2013): is the current extrinsic a local maximum
             of LiDAR-edge / image-edge alignment? -> re-calibration trigger
  3. refine  climb the same score from a realistic drift and measure what is recovered
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

from t3calib.edge_alignment import EdgeAlignment  # noqa: E402
from t3calib.geometry import ROTATION_AXES, TRANSLATION_AXES, Drift, drift_between, project  # noqa: E402
from t3calib.kitti import Drive  # noqa: E402
from t3calib.metrics import association_and_range, collect_objects, reprojection_error_by_depth  # noqa: E402

ROOT = Path(__file__).parent
ROTATION_DEG = [0, 0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0]
TRANSLATION_M = [0, 0.02, 0.05, 0.10, 0.20]
MONITOR_ROTATION_DEG = [0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0]
MONITOR_TRANSLATION_M = [0, 0.05, 0.10, 0.15, 0.20]
MONITOR_STEP_DEG, MONITOR_STEP_M = 0.5, 0.10  # Levinson & Thrun detect >0.25 deg or >10 cm; 0.25 deg flags KITTI itself
# A plausible field drift after a bracket knock: mostly yaw/pitch, a little roll, 5 cm forward.
REALISTIC_DRIFT = Drift(roll=0.3, pitch=-0.8, yaw=1.2, tx=0.05)
AXIS_LABEL = {"roll": "roll (quanh trục x)", "pitch": "pitch (quanh trục y)", "yaw": "yaw (quanh trục z)",
              "tx": "tx (dọc xe)", "ty": "ty (ngang xe)", "tz": "tz (thẳng đứng)"}


class Log:
    def __init__(self, path: Path):
        self.file = open(path, "w", encoding="utf-8")

    def __call__(self, message: str = "") -> None:
        print(message, flush=True)
        self.file.write(message + "\n")
        self.file.flush()


def sweep(frames, objects, calibration, axes, magnitudes, unit_scale, unit_name):
    rows = []
    for axis in axes:
        for magnitude in magnitudes:
            drifted = Drift.along(axis, magnitude).applied_to(calibration.velo_to_cam)
            rows.append({"axis": axis, unit_name: magnitude * unit_scale,
                         **reprojection_error_by_depth(frames, calibration, drifted),
                         **association_and_range(frames, objects, calibration, drifted)})
    return pd.DataFrame(rows)


def monitor(alignment, calibration, flag_below):
    rows = []
    cases = [("rotation", axis, magnitude, Drift.along(axis, magnitude)) for axis in ROTATION_AXES for magnitude in MONITOR_ROTATION_DEG]
    cases += [("translation", axis, magnitude, Drift.along(axis, magnitude)) for axis in TRANSLATION_AXES for magnitude in MONITOR_TRANSLATION_M]
    for kind, axis, magnitude, drift in cases:
        check = alignment.health_check(drift.applied_to(calibration.velo_to_cam), MONITOR_STEP_DEG, MONITOR_STEP_M)
        rows.append({"kind": kind, "axis": axis, "magnitude": magnitude, "score": check.score,
                     "neighbours_worse": check.fraction_of_neighbours_worse,
                     "best_neighbour_gain_pct": 100 * check.best_neighbour_gain,
                     "recalibration_triggered": check.fraction_of_neighbours_worse < flag_below})
    return pd.DataFrame(rows)


def draw_overlay(ax, frame, calibration, velo_to_cam, objects_in_frame, title):
    projection = project(frame.front_points, calibration.lidar_to_image(velo_to_cam), frame.image_size)
    pixels, depth = projection.pixels[projection.visible], projection.depth[projection.visible]
    ax.imshow((frame.image * 0.55).astype(np.uint8))  # dimmed so the LiDAR points stand out
    ax.scatter(pixels[:, 0], pixels[:, 1], c=depth, s=2.2, cmap="turbo", vmin=2, vmax=45, linewidths=0)
    for view in objects_in_frame:
        u0, v0, u1, v1 = view.box
        ax.add_patch(Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, edgecolor="lime", linewidth=1.4))
    ax.set_title(title, fontsize=10)
    ax.axis("off")


def draw_object_zoom(ax, frame, calibration, view, extrinsics_and_styles, title):
    """Crop around one object and draw its own LiDAR points under several extrinsics."""
    u0, v0, u1, v1 = view.box
    margin_u, margin_v = 0.45 * (u1 - u0), 0.45 * (v1 - v0)
    width, height = frame.image_size
    left, right = max(0, u0 - margin_u), min(width, u1 + margin_u)
    top, bottom = max(0, v0 - margin_v), min(height, v1 + margin_v)
    ax.imshow((frame.image * 0.6).astype(np.uint8))
    ax.add_patch(Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, edgecolor="white", linewidth=1.5, linestyle="--"))
    for extrinsic, colour, label in extrinsics_and_styles:
        own = project(view.own_points, calibration.lidar_to_image(extrinsic), frame.image_size)
        ax.scatter(own.pixels[own.visible, 0], own.pixels[own.visible, 1], s=9, color=colour, label=label, linewidths=0, alpha=0.9)
    ax.set_xlim(left, right)
    ax.set_ylim(bottom, top)
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=8, loc="lower right", markerscale=2)
    ax.axis("off")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data")
    parser.add_argument("--drives", nargs="+", default=["0048", "0005"])
    parser.add_argument("--frame-step", type=int, default=2, help="use every n-th frame of each drive")
    parser.add_argument("--monitor-frames", type=int, default=24, help="frames pooled by the edge-alignment monitor")
    parser.add_argument("--flag-below", type=float, default=0.9,
                        help="trigger re-calibration when fewer than this share of neighbours score lower")
    parser.add_argument("--out", type=Path, default=ROOT / "results")
    args = parser.parse_args()
    figures = args.out / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    log = Log(args.out / "run_log.txt")
    started = time.time()
    log(f"T3 calibration-drift benchmark | python {sys.version.split()[0]} | numpy {np.__version__}")
    log(f"command: python run_benchmark.py {' '.join(sys.argv[1:])}")

    # ---------------------------------------------------------------- data
    frames, calibration = [], None
    for name in args.drives:
        drive = Drive(args.data_dir, name)
        calibration = calibration or drive.calibration
        loaded = drive.load_frames(args.frame_step)
        frames += loaded
        log(f"drive {name}: {len(drive.frame_indices)} frames on disk, using {len(loaded)} (every {args.frame_step})")
    objects = collect_objects(frames, calibration)
    labels = pd.Series([view.label for view in objects]).value_counts().to_dict()
    distances = np.array([view.distance for view in objects])
    log(f"focal length {calibration.focal_length_px:.1f} px | {len(frames)} frames | {len(objects)} object views {labels}")
    log(f"object distance: median {np.median(distances):.1f} m, near<20 m {int(np.sum(distances < 20))}, far {int(np.sum(distances >= 20))}")
    log(f"[{time.time() - started:5.1f} s] data loaded")

    # ---------------------------------------------------------------- 1. sweeps
    rotation = sweep(frames, objects, calibration, ROTATION_AXES, ROTATION_DEG, 1, "drift_deg")
    translation = sweep(frames, objects, calibration, TRANSLATION_AXES, TRANSLATION_M, 100, "drift_cm")
    rotation.to_csv(args.out / "rotation_sweep.csv", index=False, float_format="%.3f")
    translation.to_csv(args.out / "translation_sweep.csv", index=False, float_format="%.3f")
    log(f"[{time.time() - started:5.1f} s] sweeps done")
    columns = ["axis", "drift_deg", "reproj_median_px", "association_retention_pct", "association_retention_pct_far",
               "objects_lost_pct", "range_miss_pct", "range_miss_pct_far"]
    log("\nRotation drift (KITTI calibration = ground truth)")
    log(rotation[columns].to_string(index=False, float_format=lambda x: f"{x:.1f}"))
    columns = ["axis", "drift_cm", "reproj_median_px_0-10m", "reproj_median_px_20-40m", "association_retention_pct_near",
               "association_retention_pct_far", "range_miss_pct"]
    log("\nTranslation drift")
    log(translation[columns].to_string(index=False, float_format=lambda x: f"{x:.1f}"))

    # ---------------------------------------------------------------- 2. monitor (both edge variants)
    pick = np.linspace(0, len(frames) - 1, min(args.monitor_frames, len(frames))).round().astype(int)
    monitor_frames = [frames[i] for i in pick]
    methods = {"paper": EdgeAlignment(monitor_frames, calibration, across_rings=False),
               "improved": EdgeAlignment(monitor_frames, calibration, across_rings=True)}
    health = pd.concat([monitor(alignment, calibration, args.flag_below).assign(method=name) for name, alignment in methods.items()])
    health.to_csv(args.out / "monitor_sweep.csv", index=False, float_format="%.4f")
    log(f"\n[{time.time() - started:5.1f} s] monitor done ({len(pick)} frames, neighbour step {MONITOR_STEP_DEG} deg / {MONITOR_STEP_M * 100:.0f} cm, "
        f"trigger when < {args.flag_below:.0%} of 52 neighbours score lower)")
    log("paper = depth jumps along each laser ring (Levinson & Thrun); improved = also across adjacent rings")
    wide = health.pivot_table(index=["kind", "axis", "magnitude"], columns="method", values="neighbours_worse", sort=False)
    wide["triggered_paper"] = wide["paper"] < args.flag_below
    wide["triggered_improved"] = wide["improved"] < args.flag_below
    log(wide.to_string(float_format=lambda x: f"{x:.3f}"))

    yaw_offsets = np.round(np.arange(-3, 3.001, 0.1), 2)
    yaw_curves = {name: [alignment.score(Drift(yaw=offset).applied_to(calibration.velo_to_cam)) for offset in yaw_offsets]
                  for name, alignment in methods.items()}
    pitch_curves = {name: [alignment.score(Drift(pitch=offset).applied_to(calibration.velo_to_cam)) for offset in yaw_offsets]
                    for name, alignment in methods.items()}

    # ---------------------------------------------------------------- 3. refine (both edge variants)
    truth = calibration.velo_to_cam
    drifted = REALISTIC_DRIFT.applied_to(truth)
    refinement = {"realistic_drift_definition": REALISTIC_DRIFT.__dict__}
    states = {"kitti_calibration": truth, "realistic_drift": drifted}
    for name, alignment in methods.items():
        states[f"refined_{name}"] = alignment.refine_rotation(drifted)
        floor_rotation, _ = drift_between(truth, alignment.refine_rotation(truth))  # where this score itself peaks
        refinement[f"score_peak_vs_kitti_deg_{name}"] = floor_rotation
    corrected = states["refined_improved"]
    judge = methods["improved"]
    for name, extrinsic in states.items():
        rotation_error, translation_error = drift_between(truth, extrinsic)
        relative = (np.linalg.inv(truth) @ extrinsic)[:3, :3]
        check = judge.health_check(extrinsic, MONITOR_STEP_DEG, MONITOR_STEP_M)
        refinement[name] = {
            "rotation_error_deg": rotation_error, "translation_error_cm": 100 * translation_error,
            "roll_error_deg": float(np.degrees(np.arctan2(relative[2, 1], relative[2, 2]))),
            "pitch_error_deg": float(np.degrees(-np.arcsin(relative[2, 0]))),
            "yaw_error_deg": float(np.degrees(np.arctan2(relative[1, 0], relative[0, 0]))),
            "neighbours_worse": check.fraction_of_neighbours_worse, "recalibration_triggered": check.fraction_of_neighbours_worse < args.flag_below,
            **reprojection_error_by_depth(frames, calibration, extrinsic), **association_and_range(frames, objects, calibration, extrinsic),
        }
    (args.out / "refinement.json").write_text(json.dumps(refinement, indent=2, default=float), encoding="utf-8")
    log(f"\n[{time.time() - started:5.1f} s] refinement done (rotation only; health check by the improved monitor)")
    keys = ["rotation_error_deg", "roll_error_deg", "pitch_error_deg", "yaw_error_deg", "translation_error_cm", "neighbours_worse",
            "recalibration_triggered", "reproj_median_px", "association_retention_pct", "association_retention_pct_far", "range_miss_pct"]
    log(pd.DataFrame({name: {key: refinement[name][key] for key in keys} for name in states}).to_string(float_format=lambda x: f"{x:.2f}"))
    for name in methods:
        log(f"score peak of the {name} variant sits {refinement[f'score_peak_vs_kitti_deg_{name}']:.2f} deg from the KITTI calibration")

    # ---------------------------------------------------------------- figures
    by_frame = {}
    for view in objects:
        by_frame.setdefault(view.frame_position, []).append(view)
    showcase = max(by_frame, key=lambda position: (len(by_frame[position]), -min(v.distance for v in by_frame[position])))
    frame = frames[showcase]
    fig = plt.figure(figsize=(12, 12.5))
    grid = fig.add_gridspec(4, 2, height_ratios=[1, 1, 1, 1.25])
    panels = [(truth, "(a) Hiệu chuẩn KITTI (ground truth)"),
              (drifted, f"(b) Lệch thực tế: roll {REALISTIC_DRIFT.roll}°, pitch {REALISTIC_DRIFT.pitch}°, yaw {REALISTIC_DRIFT.yaw}°, tx {REALISTIC_DRIFT.tx * 100:.0f} cm"),
              (corrected, f"(c) Sau tự hiệu chuẩn lại (edge alignment cải tiến): còn lệch {refinement['refined_improved']['rotation_error_deg']:.2f}°")]
    for row, (extrinsic, title) in enumerate(panels):
        draw_overlay(fig.add_subplot(grid[row, :]), frame, calibration, extrinsic, by_frame[showcase], title)
    styles = [(truth, "lime", "hiệu chuẩn đúng"), (drifted, "red", "đang lệch"), (corrected, "deepskyblue", "sau hiệu chuẩn lại")]
    near_object = min(by_frame[showcase], key=lambda v: v.distance)
    far_object = max(by_frame[showcase], key=lambda v: v.distance)
    draw_object_zoom(fig.add_subplot(grid[3, 0]), frame, calibration, near_object, styles,
                     f"(d) {near_object.label} cách {near_object.distance:.0f} m: điểm của chính nó")
    draw_object_zoom(fig.add_subplot(grid[3, 1]), frame, calibration, far_object, styles,
                     f"(e) {far_object.label} cách {far_object.distance:.0f} m: lệch cùng số pixel, box nhỏ hơn")
    fig.suptitle(f"KITTI raw drive {frame.drive}, frame {frame.index}: điểm LiDAR (màu = độ sâu) chiếu lên camera trái, khung xanh = box 2D thật", fontsize=11)
    fig.tight_layout()
    fig.savefig(figures / "01_overlay.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.3))
    for axis in ROTATION_AXES:
        rows = rotation[rotation.axis == axis]
        axes[0].plot(rows.drift_deg, rows.reproj_median_px, marker="o", label=AXIS_LABEL[axis])
        axes[1].plot(rows.drift_deg, rows.association_retention_pct, marker="o", label=AXIS_LABEL[axis])
        axes[2].plot(rows.drift_deg, rows.range_miss_pct, marker="o", label=AXIS_LABEL[axis])
    yaw_rows = rotation[rotation.axis == "yaw"]
    axes[1].plot(yaw_rows.drift_deg, yaw_rows.association_retention_pct_far, "k--", label="yaw, vật xa ≥ 20 m")
    axes[2].plot(yaw_rows.drift_deg, yaw_rows.range_miss_pct_far, "k--", label="yaw, vật xa ≥ 20 m")
    for ax, ylabel in zip(axes, ["Lỗi reprojection trung vị (px)", "Điểm LiDAR còn nằm trong box 2D (%)", "Late fusion đo sai khoảng cách (%)"]):
        ax.set_xlabel("Độ lệch xoay (°)")
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("Lệch xoay extrinsic camera-LiDAR: tác động lên chiếu điểm, association và đo khoảng cách", fontsize=11)
    fig.tight_layout()
    fig.savefig(figures / "02_rotation_sweep.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.3))
    rows = translation[translation.axis == "ty"]
    for band in ["0-10", "10-20", "20-40", "40-80"]:
        axes[0].plot(rows.drift_cm, rows[f"reproj_median_px_{band}m"], marker="o", label=f"điểm ở {band} m")
    yaw_one_degree = rotation[(rotation.axis == "yaw") & (rotation.drift_deg == 1.0)].reproj_median_px.iloc[0]
    axes[0].axhline(yaw_one_degree, color="k", linestyle="--", label="so sánh: yaw 1° (mọi độ sâu)")
    for axis in TRANSLATION_AXES:
        rows = translation[translation.axis == axis]
        axes[1].plot(rows.drift_cm, rows.association_retention_pct_near, marker="o", label=f"{AXIS_LABEL[axis]}, vật gần")
    axes[0].set_ylabel("Lỗi reprojection trung vị (px)")
    axes[1].set_ylabel("Điểm LiDAR còn trong box 2D (%)")
    for ax in axes:
        ax.set_xlabel("Độ lệch tịnh tiến (cm)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    axes[0].set_title("Lệch ngang ty: lỗi pixel tỉ lệ nghịch với độ sâu", fontsize=10)
    axes[1].set_title("Association của vật gần < 20 m", fontsize=10)
    fig.tight_layout()
    fig.savefig(figures / "03_translation_depth.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(16, 4.4))
    for name, style in (("paper", "--"), ("improved", "-")):
        label = "gốc (theo vòng quét)" if name == "paper" else "cải tiến (+ giữa các vòng)"
        axes[0].plot(yaw_offsets, np.array(yaw_curves[name]) / max(yaw_curves[name]), style, color="tab:purple", label=f"yaw, {label}")
        axes[0].plot(yaw_offsets, np.array(pitch_curves[name]) / max(pitch_curves[name]), style, color="tab:orange", label=f"pitch, {label}")
    axes[0].axvline(0, color="k", linestyle=":", label="hiệu chuẩn KITTI")
    axes[0].set_xlabel("Lệch so với KITTI (°)")
    axes[0].set_ylabel("Điểm edge alignment (chuẩn hóa)")
    axes[0].set_title("Hàm mục tiêu không cần target", fontsize=10)
    colours = {"roll": "tab:blue", "pitch": "tab:orange", "yaw": "tab:green", "tx": "tab:blue", "ty": "tab:orange", "tz": "tab:green"}
    for ax, kind, scale, xlabel in [(axes[1], "rotation", 1, "Độ lệch xoay (°)"), (axes[2], "translation", 100, "Độ lệch tịnh tiến (cm)")]:
        for (axis, name), rows in health[health.kind == kind].groupby(["axis", "method"], sort=False):
            style = "-o" if name == "improved" else "--x"
            label = f"{AXIS_LABEL[axis]}" + ("" if name == "improved" else " (gốc)")
            ax.plot(rows.magnitude * scale, 100 * rows.neighbours_worse, style, color=colours[axis], label=label, alpha=1 if name == "improved" else 0.55)
        ax.axhline(100 * args.flag_below, color="r", linestyle="--", label=f"ngưỡng trigger {args.flag_below:.0%}")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Hàng xóm có điểm thấp hơn (%)")
        ax.set_ylim(20, 103)
        ax.set_title("Health check: dưới ngưỡng đỏ = cần hiệu chuẩn lại", fontsize=10)
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(figures / "04_monitor.png", dpi=130)
    plt.close(fig)

    log(f"\n[{time.time() - started:5.1f} s] finished. Results in {args.out}")


if __name__ == "__main__":
    main()
