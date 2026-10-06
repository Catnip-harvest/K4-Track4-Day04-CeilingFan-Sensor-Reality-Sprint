"""Issue #7 - which object class loses its LiDAR points first under the same extrinsic drift?

    python per_class_analysis.py                                  # drives 0048 + 0005, every 2nd frame
    python per_class_analysis.py --data-dir D:/datasets/KITTI     # data kept outside the repo

Same frames, same objects and the same metrics as run_benchmark.py: for every tracklet object that
collect_objects keeps, association retention is the share of its own LiDAR points that still land in its
true 2D box, and a range miss is the late-fusion depth being off by > max(1 m, 10 %). Here the object
list is split by KITTI label and by distance (near < 20 m, far >= 20 m) and association_and_range is
called once per group, so t3calib/metrics.py is reused unchanged.

Outputs: results/per_class/per_class.csv, per_class.png, per_class_log.txt
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter
from pathlib import Path
from xml.etree import ElementTree

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from t3calib.geometry import Drift  # noqa: E402
from t3calib.kitti import Drive  # noqa: E402
from t3calib.metrics import association_and_range, collect_objects  # noqa: E402

ROOT = Path(__file__).parent
AXES = ("yaw", "pitch")
DRIFT_DEG = [0, 0.5, 1.0, 2.0]
FAR_FROM_M = 20
FEW_SAMPLES = 30  # fewer object views than this: report, but do not draw conclusions
LABEL_ORDER = ["Car", "Van", "Truck", "Cyclist", "Pedestrian", "Person_sitting", "Tram", "Misc"]
LABEL_COLOUR = {"Car": "tab:blue", "Van": "tab:orange", "Cyclist": "tab:green", "Pedestrian": "tab:red", "Truck": "tab:purple"}


def tracklets_per_label(drives: list[Drive]) -> Counter:
    """Distinct physical objects per label: many object views can come from one tracked object."""
    counts = Counter()
    for drive in drives:
        root = ElementTree.parse(drive.sync_dir / "tracklet_labels.xml").getroot().find("tracklets")
        counts.update(item.findtext("objectType") for item in root.findall("item"))
    return counts


class Log:
    def __init__(self, path: Path):
        self.file = open(path, "w", encoding="utf-8")

    def __call__(self, message: str = "") -> None:
        print(message, flush=True)
        self.file.write(message + "\n")
        self.file.flush()


def groups(objects):
    """(label, distance group) -> object views; 'all' rows pool every distance."""
    labels = sorted({view.label for view in objects}, key=lambda name: LABEL_ORDER.index(name) if name in LABEL_ORDER else 99)
    for label in labels:
        own = [view for view in objects if view.label == label]
        yield label, "all", own
        yield label, f"near < {FAR_FROM_M} m", [view for view in own if view.distance < FAR_FROM_M]
        yield label, f"far >= {FAR_FROM_M} m", [view for view in own if view.distance >= FAR_FROM_M]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data")
    parser.add_argument("--drives", nargs="+", default=["0048", "0005"])
    parser.add_argument("--frame-step", type=int, default=2)
    parser.add_argument("--out", type=Path, default=ROOT / "results" / "per_class")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    log = Log(args.out / "per_class_log.txt")
    started = time.time()
    log(f"per-class association | python {sys.version.split()[0]} | numpy {np.__version__}")
    log(f"command: python per_class_analysis.py {' '.join(sys.argv[1:])}")

    frames, calibration, drives = [], None, []
    for name in args.drives:
        drive = Drive(args.data_dir, name)
        drives.append(drive)
        calibration = calibration or drive.calibration
        loaded = drive.load_frames(args.frame_step)
        frames += loaded
        log(f"drive {name}: {len(drive.frame_indices)} frames on disk, using {len(loaded)} (every {args.frame_step})")
    objects = collect_objects(frames, calibration)
    tracklets = tracklets_per_label(drives)
    log(f"{len(frames)} frames | {len(objects)} object views | tracklets in these drives {dict(tracklets)}")

    rows = []
    for label, distance_group, views in groups(objects):
        if not views:
            continue
        for axis in AXES:
            for magnitude in DRIFT_DEG:
                if magnitude == 0 and axis != AXES[0]:
                    continue  # baseline is the same for every axis
                drifted = Drift.along(axis, magnitude).applied_to(calibration.velo_to_cam)
                metrics = association_and_range(frames, views, calibration, drifted)
                rows.append({
                    "label": label, "distance_group": distance_group, "axis": axis if magnitude else "none",
                    "drift_deg": magnitude, "object_views": len(views), "tracklets_in_drives": tracklets[label],
                    "median_distance_m": float(np.median([v.distance for v in views])),
                    "median_box_width_px": float(np.median([v.box[2] - v.box[0] for v in views])),
                    "few_samples": len(views) < FEW_SAMPLES,
                    "association_retention_pct": metrics["association_retention_pct"],
                    "objects_lost_pct": metrics["objects_lost_pct"],
                    "range_miss_pct": metrics["range_miss_pct"],
                })
    table = pd.DataFrame(rows)
    # Baseline rows apply to both axes: copy them so every (axis) curve starts at 0 deg.
    baseline = table[table.drift_deg == 0]
    table = pd.concat([baseline.assign(axis=axis) for axis in AXES] + [table[table.drift_deg > 0]], ignore_index=True)
    table["label"] = pd.Categorical(table.label, [name for name in LABEL_ORDER if name in set(table.label)] +
                                    sorted(set(table.label) - set(LABEL_ORDER)), ordered=True)
    table = table.sort_values(["label", "distance_group", "axis", "drift_deg"]).reset_index(drop=True)
    table.to_csv(args.out / "per_class.csv", index=False, float_format="%.2f")
    log(f"[{time.time() - started:5.1f} s] metrics done")

    counts = table[(table.drift_deg == 0) & (table.axis == AXES[0])][["label", "distance_group", "object_views", "tracklets_in_drives", "median_distance_m", "median_box_width_px"]]
    log("\nObject views per label (few = fewer than %d, no conclusion drawn)" % FEW_SAMPLES)
    log(counts.to_string(index=False, float_format=lambda x: f"{x:.1f}"))
    for metric in ("association_retention_pct", "range_miss_pct"):
        wide = table[table.distance_group == "all"].pivot_table(index="label", columns=["axis", "drift_deg"], values=metric, sort=False)
        log(f"\n{metric} (all distances)")
        log(wide.to_string(float_format=lambda x: f"{x:.1f}"))
        wide = table[table.distance_group != "all"].pivot_table(index=["label", "distance_group"], columns=["axis", "drift_deg"], values=metric, sort=False)
        log(f"\n{metric} by distance")
        log(wide.to_string(float_format=lambda x: f"{x:.1f}"))

    # ---------------------------------------------------------------- figure
    pooled = table[table.distance_group == "all"]
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    for ax_index, axis in enumerate(AXES):
        ax = axes[ax_index]
        for label, rows_label in pooled[pooled.axis == axis].groupby("label", sort=False):
            n, objects_tracked = int(rows_label.object_views.iloc[0]), int(rows_label.tracklets_in_drives.iloc[0])
            few = bool(rows_label.few_samples.iloc[0])
            ax.plot(rows_label.drift_deg, rows_label.association_retention_pct, ":" if few else "-", marker="x" if few else "o",
                    color=LABEL_COLOUR.get(label), label=f"{label} (n={n}, {objects_tracked} vật{', ít mẫu' if few else ''})")
        ax.set_title(f"Lệch {axis}: điểm LiDAR còn trong box 2D, theo loại vật", fontsize=10)
        ax.set_xlabel(f"Độ lệch {axis} (°)")
        ax.set_ylabel("Association retention (%)")
        ax.set_ylim(0, 103)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    ax = axes[2]
    split = table[(table.distance_group != "all") & (table.axis == "yaw") & (~table.few_samples)]
    for (label, group), rows_group in split.groupby(["label", "distance_group"], sort=False):
        ax.plot(rows_group.drift_deg, rows_group.association_retention_pct, "-" if group.startswith("near") else "--", marker="o",
                color=LABEL_COLOUR.get(label),
                label=f"{label}, {group.replace('near', 'gần').replace('far', 'xa')} (n={int(rows_group.object_views.iloc[0])})")
    ax.set_title(f"Lệch yaw: gần / xa (ngưỡng {FAR_FROM_M} m), chỉ nhóm ≥ {FEW_SAMPLES} lượt", fontsize=10)
    ax.set_xlabel("Độ lệch yaw (°)")
    ax.set_ylabel("Association retention (%)")
    ax.set_ylim(0, 103)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    fig.suptitle(f"KITTI raw {' + '.join(args.drives)}, {len(frames)} frame, {len(objects)} lượt vật; box 2D = chiếu box 3D thật (detector hoàn hảo)", fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out / "per_class.png", dpi=130)
    plt.close(fig)
    log(f"\n[{time.time() - started:5.1f} s] finished. Results in {args.out}")


if __name__ == "__main__":
    main()
