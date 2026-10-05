"""Live class demo: knock the camera-LiDAR calibration, watch the health alarm, auto-correct the rotation.

    python demo.py                                        # realistic knock on held-out drive 0052
    python demo.py --yaw 2.5 --pitch 0 --roll 0 --tx 0    # one big yaw knock: does the capture range hold?

No new algorithm: every number comes from t3calib (edge-alignment monitor + refine_rotation, metrics)
with the settings of run_benchmark.py / validate_holdout.py. Outputs in results/demo/:
demo_before_after.png, demo_log.txt, demo_summary.json.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

from run_benchmark import MONITOR_STEP_DEG, MONITOR_STEP_M, REALISTIC_DRIFT, draw_object_zoom, draw_overlay
from t3calib.edge_alignment import EdgeAlignment
from t3calib.geometry import Drift, drift_between
from t3calib.kitti import Drive
from t3calib.metrics import association_and_range, collect_objects
from validate_holdout import TRIGGER_BELOW, residual_roll_pitch_yaw

import matplotlib.pyplot as plt  # noqa: E402  (backend already set to Agg by run_benchmark)

ROOT = Path(__file__).parent
HELD_OUT_DRIVE, FALLBACK_DRIVE = "0052", "0048"
RULE = "=" * 64


class DemoLog:
    """Print one line to the console and keep the same line in demo_log.txt."""

    def __init__(self, path: Path, started: float):
        self.lines: list[str] = []
        self.path = path
        self.started = started

    def __call__(self, message: str = "", timed: bool = False) -> None:
        if timed:
            message = f"[{time.time() - self.started:5.1f} s] {message}"
        print(message, flush=True)
        self.lines.append(message)
        self.path.write_text("\n".join(self.lines) + "\n", encoding="utf-8")


def health_verdict(alignment: EdgeAlignment, extrinsic: np.ndarray) -> tuple[float, bool]:
    """Share of the 52 neighbours that score lower, and whether that is below the 90 % trigger."""
    share = alignment.health_check(extrinsic, MONITOR_STEP_DEG, MONITOR_STEP_M).fraction_of_neighbours_worse
    return share, share < TRIGGER_BELOW


def verdict_text(alarm: bool) -> str:
    return "BÁO ĐỘNG - cần hiệu chuẩn lại" if alarm else "OK - hiệu chuẩn tốt"


def short_verdict(alarm: bool) -> str:
    return "BÁO ĐỘNG" if alarm else "OK"


def drift_text(drift: Drift) -> str:
    parts = [f"{axis} {getattr(drift, axis):+g}°" for axis in ("roll", "pitch", "yaw") if getattr(drift, axis)]
    parts += [f"{axis} {100 * getattr(drift, axis):+g} cm" for axis in ("tx", "ty", "tz") if getattr(drift, axis)]
    return ", ".join(parts) or "không lệch"


def shown(path: Path) -> str:
    """Short path for the projector: relative to the current folder when possible."""
    try:
        return str(path.resolve().relative_to(Path.cwd().resolve()))
    except ValueError:
        return str(path)


def open_drive(data_dir: Path, requested: str, log: DemoLog) -> Drive:
    try:
        return Drive(data_dir, requested)
    except FileNotFoundError:
        if requested != HELD_OUT_DRIVE:
            raise
        log(f"!! Không tìm thấy drive {HELD_OUT_DRIVE} (drive giữ lại để kiểm tra) -> dùng drive {FALLBACK_DRIVE}.")
        log(f"!! Lưu ý: {FALLBACK_DRIVE} là drive đã dùng để tinh chỉnh, kết quả sẽ đẹp hơn thực tế.")
        return Drive(data_dir, FALLBACK_DRIVE)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--drive", default=HELD_OUT_DRIVE, help="KITTI raw drive (default 0052, held out from tuning)")
    for axis in ("roll", "pitch", "yaw"):
        parser.add_argument(f"--{axis}", type=float, default=getattr(REALISTIC_DRIFT, axis), help=f"{axis} knock in degrees")
    for axis in ("tx", "ty", "tz"):
        parser.add_argument(f"--{axis}", type=float, default=getattr(REALISTIC_DRIFT, axis), help=f"{axis} knock in metres")
    parser.add_argument("--frames", type=int, default=12, help="evenly spaced frames pooled by the monitor")
    parser.add_argument("--variant", choices=("improved", "paper"), default="improved",
                        help="improved = depth edges along and across laser rings; paper = along rings only")
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data")
    parser.add_argument("--out", type=Path, default=ROOT / "results" / "demo")
    args = parser.parse_args()

    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")  # Vietnamese text on a Windows console
    args.out.mkdir(parents=True, exist_ok=True)
    started = time.time()
    log = DemoLog(args.out / "demo_log.txt", started)
    knock = Drift(roll=args.roll, pitch=args.pitch, yaw=args.yaw, tx=args.tx, ty=args.ty, tz=args.tz)

    log(RULE)
    log("  DEMO: camera-LiDAR bị va lệch -> báo động -> tự hiệu chuẩn lại")
    log(RULE)

    # ------------------------------------------------------------------ data
    drive = open_drive(args.data_dir, args.drive, log)
    pick = np.linspace(0, len(drive.frame_indices) - 1, min(args.frames, len(drive.frame_indices))).round().astype(int)
    frames = [drive.load_frame(drive.frame_indices[i]) for i in pick]
    calibration = drive.calibration
    objects = collect_objects(frames, calibration)
    alignment = EdgeAlignment(frames, calibration, across_rings=args.variant == "improved")
    truth = calibration.velo_to_cam
    log(f"Dữ liệu: KITTI drive {drive.name}, {len(frames)} khung hình, {len(objects)} vật có nhãn", timed=True)
    log("")

    # ------------------------------------------------------------------ 1. true calibration
    step = time.time()
    share_truth, alarm_truth = health_verdict(alignment, truth)
    baseline = association_and_range(frames, objects, calibration, truth)
    log("1) Hiệu chuẩn đúng (KITTI)")
    log(f"   Sức khỏe: {verdict_text(alarm_truth)}   ({share_truth:.0%} hàng xóm kém hơn, ngưỡng {TRIGGER_BELOW:.0%})")
    log(f"   ({time.time() - step:.1f} s)")
    log("")

    # ------------------------------------------------------------------ 2. knock
    step = time.time()
    drifted = knock.applied_to(truth)
    injected_rotation, injected_translation = drift_between(truth, drifted)
    share_drifted, alarm_drifted = health_verdict(alignment, drifted)
    before = association_and_range(frames, objects, calibration, drifted)
    log(f"2) Va chạm: lệch {drift_text(knock)}")
    log(f"   Sức khỏe: {verdict_text(alarm_drifted)}   ({share_drifted:.0%} hàng xóm kém hơn, ngưỡng {TRIGGER_BELOW:.0%})")
    log(f"   Điểm LiDAR còn trong box 2D: {before['association_retention_pct']:.0f} %   (khi đúng: {baseline['association_retention_pct']:.0f} %)")
    log(f"   Late fusion đo sai khoảng cách: {before['range_miss_pct']:.0f} % số vật   (khi đúng: {baseline['range_miss_pct']:.0f} %)")
    log(f"   ({time.time() - step:.1f} s)")
    log("")

    # ------------------------------------------------------------------ 3. self-recalibration
    step = time.time()
    corrected = alignment.refine_rotation(drifted)
    rotation_left, translation_left = drift_between(truth, corrected)
    roll_left, pitch_left, yaw_left = residual_roll_pitch_yaw(truth, corrected)
    share_after, alarm_after = health_verdict(alignment, corrected)
    after = association_and_range(frames, objects, calibration, corrected)
    log("3) Tự hiệu chuẩn lại (chỉ sửa xoay, không cần bảng target)")
    log(f"   Lệch xoay: {injected_rotation:.2f}° -> còn {rotation_left:.2f}°")
    log(f"   Theo trục: roll {roll_left:+.2f}°, pitch {pitch_left:+.2f}°, yaw {yaw_left:+.2f}°")
    log(f"   Lệch tịnh tiến còn lại: {100 * translation_left:.1f} cm   (phương pháp không sửa tịnh tiến)")
    log(f"   Sức khỏe: {verdict_text(alarm_after)}   ({share_after:.0%} hàng xóm kém hơn)")
    log(f"   Điểm LiDAR còn trong box 2D: {before['association_retention_pct']:.0f} % -> {after['association_retention_pct']:.0f} %")
    log(f"   Late fusion đo sai khoảng cách: {before['range_miss_pct']:.0f} % -> {after['range_miss_pct']:.0f} %")
    log(f"   ({time.time() - step:.1f} s)")
    log("")

    # ------------------------------------------------------------------ 4. picture
    step = time.time()
    by_frame: dict[int, list] = {}
    for view in objects:
        by_frame.setdefault(view.frame_position, []).append(view)
    if by_frame:
        showcase = max(by_frame, key=lambda position: (len(by_frame[position]), -min(v.distance for v in by_frame[position])))
        in_frame = by_frame[showcase]
    else:
        showcase, in_frame = len(frames) // 2, []
    frame = frames[showcase]
    figure_path = args.out / "demo_before_after.png"
    fig = plt.figure(figsize=(12, 12.5))
    grid = fig.add_gridspec(4, 2, height_ratios=[1, 1, 1, 1.25])
    panels = [(truth, f"(a) Hiệu chuẩn đúng (KITTI): sức khỏe {short_verdict(alarm_truth)}"),
              (drifted, f"(b) Sau va chạm ({drift_text(knock)}): sức khỏe {short_verdict(alarm_drifted)}"),
              (corrected, f"(c) Sau tự hiệu chuẩn lại, còn lệch xoay {rotation_left:.2f}°: sức khỏe {short_verdict(alarm_after)}")]
    for row, (extrinsic, title) in enumerate(panels):
        draw_overlay(fig.add_subplot(grid[row, :]), frame, calibration, extrinsic, in_frame, title)
    if in_frame:
        far_object = max(in_frame, key=lambda v: v.distance)
        styles = [(truth, "lime", "hiệu chuẩn đúng"), (drifted, "red", "sau va chạm"), (corrected, "deepskyblue", "sau hiệu chuẩn lại")]
        draw_object_zoom(fig.add_subplot(grid[3, 0]), frame, calibration, far_object, styles,
                         f"(d) {far_object.label} xa nhất, cách {far_object.distance:.0f} m: điểm LiDAR của chính nó")
    summary_ax = fig.add_subplot(grid[3, 1])
    summary_ax.axis("off")
    summary_lines = [
        ("Trước -> sau hiệu chuẩn lại", 14, "bold"),
        (f"Lệch xoay: {injected_rotation:.2f}° -> {rotation_left:.2f}°", 12, "normal"),
        (f"Điểm LiDAR trong box 2D: {before['association_retention_pct']:.0f}% -> {after['association_retention_pct']:.0f}%", 12, "normal"),
        (f"Đo sai khoảng cách: {before['range_miss_pct']:.0f}% -> {after['range_miss_pct']:.0f}%", 12, "normal"),
        (f"Hàng xóm kém hơn: {share_drifted:.0%} -> {share_after:.0%} (ngưỡng {TRIGGER_BELOW:.0%})", 12, "normal"),
        (f"Tịnh tiến còn lệch: {100 * translation_left:.1f} cm (không sửa)", 11, "normal"),
        (f"drive {drive.name}, {len(frames)} khung hình, biến thể {args.variant}", 10, "normal"),
    ]
    for line_number, (text, size, weight) in enumerate(summary_lines):
        summary_ax.text(0.02, 0.92 - 0.14 * line_number, text, fontsize=size, fontweight=weight, transform=summary_ax.transAxes, va="top")
    fig.suptitle(f"KITTI drive {frame.drive}, khung {frame.index}: điểm LiDAR (màu = độ sâu) chiếu lên camera trái, khung xanh = box 2D thật",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(figure_path, dpi=120)
    plt.close(fig)
    log(f"4) Ảnh trước/sau: {shown(figure_path)}   ({time.time() - step:.1f} s)")

    total = time.time() - started
    summary = {
        "command": "python demo.py " + " ".join(sys.argv[1:]),
        "drive": drive.name, "frames": [int(f.index) for f in frames], "object_views": len(objects), "variant": args.variant,
        "knock": knock.__dict__, "trigger_below": TRIGGER_BELOW, "neighbour_step": {"deg": MONITOR_STEP_DEG, "m": MONITOR_STEP_M},
        "true_calibration": {"neighbours_worse": share_truth, "alarm": alarm_truth, **baseline},
        "knocked": {"neighbours_worse": share_drifted, "alarm": alarm_drifted, "rotation_deg": injected_rotation,
                    "translation_cm": 100 * injected_translation, **before},
        "corrected": {"neighbours_worse": share_after, "alarm": alarm_after, "rotation_left_deg": rotation_left,
                      "roll_left_deg": roll_left, "pitch_left_deg": pitch_left, "yaw_left_deg": yaw_left,
                      "translation_left_cm": 100 * translation_left, **after},
        "showcase_frame": int(frame.index), "figure": str(figure_path), "runtime_s": total,
    }
    (args.out / "demo_summary.json").write_text(json.dumps(summary, indent=2, default=float), encoding="utf-8")
    log(RULE)
    log(f"  Xong trong {total:.0f} s. Kết quả: {shown(args.out)}")
    log(RULE)


if __name__ == "__main__":
    main()
