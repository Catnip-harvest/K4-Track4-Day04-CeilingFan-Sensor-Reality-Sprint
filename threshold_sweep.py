"""Issue #8 - pick the re-calibration trigger threshold from data: sensitivity versus false alarms.

    python threshold_sweep.py                                  # drives 0048, 0005, 0052 from ./data
    python threshold_sweep.py --data-dir D:/datasets/KITTI     # data kept outside the repo

The monitor (t3calib/edge_alignment.py) reports health = share of the 52 neighbour extrinsics that score
lower than the current one, and run_benchmark.py / validate_holdout.py trigger when health < 0.90. That
0.90 was never chosen from data. Here every threshold from 0.80 to 0.98 is scored on:

  false alarms  health at the TRUE KITTI calibration on many windows of consecutive frames (each window
                is one sample; every frame of the drive, windows overlap by half). Measured twice:
                12-frame windows (what issue #8 asks for) and 24-frame windows (what the monitor pools in
                run_benchmark.py and validate_holdout.py, so it matches the sensitivity numbers).
  sensitivity   health of the 20 seeded random drifts (rotation +-1.5 deg per axis, translation +-5 cm)
                already stored in results/holdout/*_random_drifts.csv (24 pooled frames), and of the
                single-axis drifts in results/holdout/*_summary.json -> detection_sweep. Nothing is
                recomputed for these, so the threshold 0.90 must give back the stored trigger columns.

Monitor settings (neighbour step 0.5 deg / 10 cm, 52 neighbours) are copied from validate_holdout.py.
Outputs: results/threshold/threshold.csv, threshold.png, false_alarm_windows.csv, threshold_log.txt
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

from t3calib.edge_alignment import EdgeAlignment  # noqa: E402
from t3calib.kitti import Drive  # noqa: E402
from validate_holdout import MONITOR_STEP_DEG, MONITOR_STEP_M, TRIGGER_BELOW, VARIANTS  # noqa: E402

ROOT = Path(__file__).parent
THRESHOLDS = np.round(np.arange(0.80, 0.9801, 0.01), 2)
WINDOW_SIZES = (12, 24)
DRIFT_SETS = {"dev": "0048 + 0005", "holdout": "0052", "holdout_0001": "0001"}  # label in results/holdout -> drives
SINGLE_AXIS_WORTH_CATCHING = {"rotation_deg": 0.5, "translation_m": 0.10}  # Levinson & Thrun's 0.25 deg is below KITTI's own noise


class Log:
    def __init__(self, path: Path):
        self.file = open(path, "w", encoding="utf-8")

    def __call__(self, message: str = "") -> None:
        print(message, flush=True)
        self.file.write(message + "\n")
        self.file.flush()


def window_starts(frame_count: int, size: int) -> list[int]:
    return list(range(0, frame_count - size + 1, size // 2))


def false_alarm_windows(data_dir: Path, drives: list[str], log: Log) -> pd.DataFrame:
    """Health at the true calibration on half-overlapping windows of consecutive frames, per drive."""
    rows = []
    for name in drives:
        drive = Drive(data_dir, name)
        frames = drive.load_frames(1)
        truth = drive.calibration.velo_to_cam
        for variant, across in VARIANTS.items():
            # Build the edge maps once per drive, then score each window on a slice of them.
            full = EdgeAlignment(frames, drive.calibration, across_rings=across)
            for size in WINDOW_SIZES:
                for start in window_starts(len(frames), size):
                    window = EdgeAlignment([], drive.calibration, across_rings=across)
                    window.frames = full.frames[start:start + size]
                    check = window.health_check(truth, MONITOR_STEP_DEG, MONITOR_STEP_M)
                    rows.append({"drive": name, "variant": variant, "window_frames": size, "first_frame": frames[start].index,
                                 "neighbours_worse": check.fraction_of_neighbours_worse, "score": check.score})
        counts = {size: len(window_starts(len(frames), size)) for size in WINDOW_SIZES}
        log(f"drive {name}: {len(frames)} frames, windows {counts}")
        del frames
    return pd.DataFrame(rows)


def stored_drifts(results: Path) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """Random drifts, single-axis drifts and the health at the true calibration that validate_holdout.py stored."""
    random_rows, single_rows, truth_rows = [], [], []
    for label, drives in DRIFT_SETS.items():
        csv = results / f"{label}_random_drifts.csv"
        summary = results / f"{label}_summary.json"
        if not csv.exists() or not summary.exists():
            continue
        table = pd.read_csv(csv)
        for variant in VARIANTS:
            random_rows += [{"set": label, "drives": drives, "variant": variant, "drift_id": int(row.drift_id),
                             "injected_rotation_deg": row.injected_rotation_deg,
                             "neighbours_worse": row[f"neighbours_worse_{variant}"], "stored_trigger": bool(row[f"triggered_{variant}"])}
                            for _, row in table.iterrows()]
        stored = json.loads(summary.read_text(encoding="utf-8"))
        truth_rows += [{"set": label, "drives": drives, "variant": variant, "neighbours_worse": stored["methods"][variant]["neighbours_worse_at_truth"]}
                       for variant in VARIANTS]
        for row in stored["detection_sweep"]:
            rotation = row["axis"] in ("roll", "pitch", "yaw")
            limit = SINGLE_AXIS_WORTH_CATCHING["rotation_deg" if rotation else "translation_m"]
            for variant in VARIANTS:
                single_rows.append({"set": label, "drives": drives, "variant": variant, "axis": row["axis"], "magnitude": row["magnitude"],
                                    "worth_catching": row["magnitude"] >= limit, "neighbours_worse": row[f"neighbours_worse_{variant}"]})
    return pd.DataFrame(random_rows), pd.DataFrame(single_rows), pd.DataFrame(truth_rows)


def rate(values: pd.Series, threshold: float) -> float:
    return 100 * float(np.mean(values < threshold)) if len(values) else float("nan")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data")
    parser.add_argument("--drives", nargs="+", default=["0048", "0005", "0052"], help="drives for the false-alarm windows")
    parser.add_argument("--holdout-results", type=Path, default=ROOT / "results" / "holdout")
    parser.add_argument("--out", type=Path, default=ROOT / "results" / "threshold")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    log = Log(args.out / "threshold_log.txt")
    started = time.time()
    log(f"trigger threshold sweep | python {sys.version.split()[0]} | numpy {np.__version__}")
    log(f"command: python threshold_sweep.py {' '.join(sys.argv[1:])}")
    log(f"monitor: neighbour step {MONITOR_STEP_DEG} deg / {MONITOR_STEP_M * 100:.0f} cm, trigger when health < threshold")

    windows = false_alarm_windows(args.data_dir, args.drives, log)
    windows.to_csv(args.out / "false_alarm_windows.csv", index=False, float_format="%.4f")
    log(f"[{time.time() - started:5.1f} s] false-alarm windows done ({len(windows)} health checks)")

    random_drifts, single_axis, at_truth = stored_drifts(args.holdout_results)
    log(f"stored drifts: {random_drifts.groupby(['set', 'variant']).size().to_dict()}")

    # Cross-check: at the current threshold the stored trigger columns must come back unchanged.
    for (label, variant), rows in random_drifts.groupby(["set", "variant"], sort=False):
        recomputed = rows.neighbours_worse < TRIGGER_BELOW
        assert (recomputed == rows.stored_trigger).all(), f"{label}/{variant}: threshold {TRIGGER_BELOW} does not reproduce the stored triggers"
        log(f"cross-check {label:13s} {variant:8s}: threshold {TRIGGER_BELOW:.2f} -> {100 * recomputed.mean():.0f} % triggered (matches CSV)")

    rows = []
    for variant in VARIANTS:
        for threshold in THRESHOLDS:
            row = {"variant": variant, "threshold": threshold}
            for size in WINDOW_SIZES:
                own = windows[(windows.variant == variant) & (windows.window_frames == size)]
                row[f"false_alarm_pct_{size}f"] = rate(own.neighbours_worse, threshold)
                row[f"false_alarm_windows_{size}f"] = len(own)
                for drive in args.drives:
                    row[f"false_alarm_pct_{size}f_{drive}"] = rate(own[own.drive == drive].neighbours_worse, threshold)
            # Pooled 24-frame samples spread over whole drives (validate_holdout.py), including drive 0001.
            own = at_truth[at_truth.variant == variant]
            row["false_alarm_pooled_sets"] = int(np.sum(own.neighbours_worse < threshold))
            row["false_alarm_pooled_sets_of"] = len(own)
            own = random_drifts[random_drifts.variant == variant]
            row["random_drift_detect_pct"] = rate(own.neighbours_worse, threshold)
            for label in DRIFT_SETS:
                row[f"random_drift_detect_pct_{label}"] = rate(own[own.set == label].neighbours_worse, threshold)
            own = single_axis[(single_axis.variant == variant) & single_axis.worth_catching]
            row["single_axis_detect_pct"] = rate(own.neighbours_worse, threshold)
            own = single_axis[(single_axis.variant == variant) & ~single_axis.worth_catching]
            row["single_axis_small_flagged_pct"] = rate(own.neighbours_worse, threshold)
            rows.append(row)
    table = pd.DataFrame(rows)
    table.round(1).assign(threshold=table.threshold).to_csv(args.out / "threshold.csv", index=False)

    log("health at the true calibration stored by validate_holdout.py (24 frames spread over the drive): "
        + ", ".join(f"{r.set}/{r.variant} {r.neighbours_worse:.3f}" for r in at_truth.itertuples()))
    columns = ["threshold", "false_alarm_pct_12f", "false_alarm_pct_24f", "false_alarm_pooled_sets", "random_drift_detect_pct",
               *[f"random_drift_detect_pct_{label}" for label in DRIFT_SETS], "single_axis_detect_pct"]
    for variant in VARIANTS:
        log(f"\n{variant}: false alarm on {int(table.false_alarm_windows_12f.iloc[0])} x 12-frame / "
            f"{int(table.false_alarm_windows_24f.iloc[0])} x 24-frame windows at the true calibration; "
            f"detection on {len(random_drifts[random_drifts.variant == variant])} random drifts and "
            f"{len(single_axis[(single_axis.variant == variant) & single_axis.worth_catching])} single-axis drifts "
            f">= {SINGLE_AXIS_WORTH_CATCHING['rotation_deg']} deg / {SINGLE_AXIS_WORTH_CATCHING['translation_m'] * 100:.0f} cm")
        log(table[table.variant == variant][columns].to_string(index=False, formatters={"threshold": "{:.2f}".format},
                                                                float_format=lambda x: f"{x:.1f}"))
    log("\nhealth at the true calibration, per drive and window size (min / median)")
    log(windows.groupby(["variant", "window_frames", "drive"]).neighbours_worse.agg(["count", "min", "median"]).to_string(float_format=lambda x: f"{x:.3f}"))

    # ---------------------------------------------------------------- figure
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))
    colours = {"paper": "tab:blue", "improved": "tab:orange"}
    for variant in VARIANTS:
        own = table[table.variant == variant]
        name = "gốc (paper)" if variant == "paper" else "cải tiến (improved)"
        axes[0].plot(own.threshold, own.random_drift_detect_pct, "-o", color=colours[variant], label=f"{name}: phát hiện 60 lệch ngẫu nhiên")
        axes[0].plot(own.threshold, own.false_alarm_pct_24f, "--s", color=colours[variant], label=f"{name}: báo động giả, cửa sổ 24 frame")
        axes[0].plot(own.threshold, own.false_alarm_pct_12f, ":^", color=colours[variant], label=f"{name}: báo động giả, cửa sổ 12 frame")
        for size, marker in ((24, "s"), (12, "^")):
            axes[1].plot(own[f"false_alarm_pct_{size}f"], own.random_drift_detect_pct, f"-{marker}", color=colours[variant],
                         alpha=1 if size == 24 else 0.5, label=f"{name}, cửa sổ {size} frame")
            for _, point in own[own.threshold.isin([0.85, 0.90, 0.95])].iterrows():
                axes[1].annotate(f"{point.threshold:.2f}", (point[f"false_alarm_pct_{size}f"], point.random_drift_detect_pct),
                                 fontsize=7, xytext=(3, -9), textcoords="offset points", color=colours[variant])
        axes[2].plot(own.threshold, own.single_axis_detect_pct, "-o", color=colours[variant],
                     label=f"{name}: lệch 1 trục ≥ 0.5° / 10 cm")
    axes[0].axvline(TRIGGER_BELOW, color="k", linestyle=":", label=f"ngưỡng hiện tại {TRIGGER_BELOW:.2f}")
    lowest = at_truth[at_truth.variant == "improved"].sort_values("neighbours_worse").iloc[0]
    axes[0].axvline(lowest.neighbours_worse, color="tab:orange", linestyle="-.", alpha=0.7,
                    label=f"health improved ở hiệu chuẩn đúng, drive {lowest.drives}: {lowest.neighbours_worse:.3f}")
    axes[2].axvline(TRIGGER_BELOW, color="k", linestyle=":")
    axes[0].set(xlabel="Ngưỡng health (báo khi health < ngưỡng)", ylabel="%", title="Độ nhạy và báo động giả theo ngưỡng")
    axes[1].set(xlabel="Báo động giả ở hiệu chuẩn đúng (%)", ylabel="Phát hiện lệch ngẫu nhiên (%)", title="Đánh đổi (số = ngưỡng)")
    axes[2].set(xlabel="Ngưỡng health", ylabel="Phát hiện (%)", title="Lệch một trục (roll, tx gần như không bắt được)")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.suptitle("Monitor edge alignment trên KITTI raw: chọn ngưỡng kích hoạt hiệu chuẩn lại", fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out / "threshold.png", dpi=130)
    plt.close(fig)
    log(f"\n[{time.time() - started:5.1f} s] finished. Results in {args.out}")


if __name__ == "__main__":
    main()
