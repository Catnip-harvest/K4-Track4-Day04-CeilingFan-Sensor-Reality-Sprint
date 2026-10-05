"""Held-out check of the edge-alignment monitor and rotation refinement, with FIXED settings.

Why this exists: the edge-field settings in t3calib/edge_alignment.py and the monitor step used by
run_benchmark.py were tuned on drive 0048 and on 24 frames pooled from 0048 + 0005. Numbers measured
on those drives therefore flatter the method. This script re-runs the same checks on any drive list
WITHOUT changing a single setting, so a drive that was never looked at during tuning gives an honest
estimate. Nothing here is tuned: the neighbour step, the trigger threshold, the frame step and the
number of pooled frames are copied from run_benchmark.py.

What it measures, for both edge variants (paper = depth jumps along each laser ring, Levinson & Thrun
2013; improved = also across adjacent rings):
  1. false alarm   health check at the true KITTI calibration (should NOT trigger)
  2. detection     single-axis drifts: roll/pitch/yaw 0.25-2 deg, tx/ty/tz 5-20 cm (should trigger)
  3. random drifts 20 drifts drawn with a fixed seed: does the monitor trigger, and how much rotation
                   is left after refine_rotation (total and per axis)

Usage:
    python validate_holdout.py --drives 0052 --label holdout
    python validate_holdout.py --drives 0048 0005 --label dev
Outputs go to results/holdout/<label>_summary.json, <label>_random_drifts.csv, <label>_log.txt.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np
import pandas as pd

from t3calib.edge_alignment import EdgeAlignment
from t3calib.geometry import ROTATION_AXES, TRANSLATION_AXES, Drift, drift_between
from t3calib.kitti import Drive

ROOT = Path(__file__).parent

# Settings copied from run_benchmark.py. They are deliberately not exposed as tuning knobs.
FRAME_STEP = 2
MONITOR_STEP_DEG, MONITOR_STEP_M = 0.5, 0.10
TRIGGER_BELOW = 0.90  # re-calibrate when fewer than 90 % of the 52 neighbours score lower

DETECTION_ROTATION_DEG = [0.25, 0.5, 1.0, 2.0]
DETECTION_TRANSLATION_M = [0.05, 0.10, 0.20]

RANDOM_SEED = 7
RANDOM_DRIFT_COUNT = 20
RANDOM_ROTATION_LIMIT_DEG = 1.5
RANDOM_TRANSLATION_LIMIT_M = 0.05

VARIANTS = {"paper": False, "improved": True}  # name -> across_rings


class Log:
    """Print a line and keep a copy in the log file."""

    def __init__(self, path: Path):
        self.file = open(path, "w", encoding="utf-8")

    def __call__(self, message: str = "") -> None:
        print(message, flush=True)
        self.file.write(message + "\n")
        self.file.flush()


def residual_roll_pitch_yaw(truth: np.ndarray, estimate: np.ndarray) -> tuple[float, float, float]:
    """Rotation still left between two extrinsics, split into roll, pitch, yaw (degrees)."""
    relative = (np.linalg.inv(truth) @ estimate)[:3, :3]
    roll = np.degrees(np.arctan2(relative[2, 1], relative[2, 2]))
    pitch = np.degrees(-np.arcsin(np.clip(relative[2, 0], -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(relative[1, 0], relative[0, 0]))
    return float(roll), float(pitch), float(yaw)


def neighbours_worse(alignment: EdgeAlignment, extrinsic: np.ndarray) -> float:
    return alignment.health_check(extrinsic, MONITOR_STEP_DEG, MONITOR_STEP_M).fraction_of_neighbours_worse


def draw_random_drifts() -> list[Drift]:
    generator = np.random.default_rng(RANDOM_SEED)
    drifts = []
    for _ in range(RANDOM_DRIFT_COUNT):
        roll, pitch, yaw = generator.uniform(-RANDOM_ROTATION_LIMIT_DEG, RANDOM_ROTATION_LIMIT_DEG, 3)
        tx, ty, tz = generator.uniform(-RANDOM_TRANSLATION_LIMIT_M, RANDOM_TRANSLATION_LIMIT_M, 3)
        drifts.append(Drift(roll=roll, pitch=pitch, yaw=yaw, tx=tx, ty=ty, tz=tz))
    return drifts


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--drives", nargs="+", required=True)
    parser.add_argument("--label", required=True, help="name used for the output files, e.g. holdout or dev")
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data")
    parser.add_argument("--monitor-frames", type=int, default=24, help="frames pooled by the edge score (benchmark uses 24)")
    parser.add_argument("--out", type=Path, default=ROOT / "results" / "holdout")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    log = Log(args.out / f"{args.label}_log.txt")
    started = time.time()
    log(f"held-out edge-alignment check | python {sys.version.split()[0]} | numpy {np.__version__}")
    log(f"command: python validate_holdout.py {' '.join(sys.argv[1:])}")
    log(f"fixed settings: frame step {FRAME_STEP}, neighbour step {MONITOR_STEP_DEG} deg / {MONITOR_STEP_M * 100:.0f} cm, "
        f"trigger when < {TRIGGER_BELOW:.0%} of neighbours score lower")

    # ------------------------------------------------------------------ data
    frames, calibration, drive_info = [], None, {}
    for name in args.drives:
        drive = Drive(args.data_dir, name)
        calibration = calibration or drive.calibration
        loaded = drive.load_frames(FRAME_STEP)
        frames += loaded
        drive_info[name] = {"frames_on_disk": len(drive.frame_indices), "frames_used": len(loaded)}
        log(f"drive {name}: {len(drive.frame_indices)} frames on disk, using {len(loaded)} (every {FRAME_STEP})")
    labels = Counter(obj.label for frame in frames for obj in frame.objects)
    log(f"tracklet boxes in the loaded frames: {dict(labels)}")

    pick = np.linspace(0, len(frames) - 1, min(args.monitor_frames, len(frames))).round().astype(int)
    monitor_frames = [frames[i] for i in pick]
    alignments = {name: EdgeAlignment(monitor_frames, calibration, across_rings=across) for name, across in VARIANTS.items()}
    log(f"[{time.time() - started:5.1f} s] edge fields built on {len(pick)} frames "
        f"({', '.join(f'{frames[i].drive}/{frames[i].index}' for i in pick[[0, -1]])} ... evenly spaced)")

    truth = calibration.velo_to_cam

    # ------------------------------------------------------------------ 1. false alarm at the true calibration
    at_truth = {name: neighbours_worse(alignment, truth) for name, alignment in alignments.items()}
    for name, fraction in at_truth.items():
        log(f"at KITTI calibration, {name:8s}: neighbours worse {fraction:.3f} -> "
            f"{'FALSE ALARM' if fraction < TRIGGER_BELOW else 'no trigger'}")

    # ------------------------------------------------------------------ 2. single-axis detection sweep
    cases = [(axis, magnitude) for axis in ROTATION_AXES for magnitude in DETECTION_ROTATION_DEG]
    cases += [(axis, magnitude) for axis in TRANSLATION_AXES for magnitude in DETECTION_TRANSLATION_M]
    detection_rows = []
    for axis, magnitude in cases:
        drifted = Drift.along(axis, magnitude).applied_to(truth)
        row = {"axis": axis, "magnitude": magnitude}
        for name, alignment in alignments.items():
            row[f"neighbours_worse_{name}"] = neighbours_worse(alignment, drifted)
            row[f"triggered_{name}"] = row[f"neighbours_worse_{name}"] < TRIGGER_BELOW
        detection_rows.append(row)
    detection = pd.DataFrame(detection_rows)
    log(f"\n[{time.time() - started:5.1f} s] single-axis detection sweep (magnitude in deg for rotation, m for translation)")
    log(detection.to_string(index=False, float_format=lambda x: f"{x:.3f}"))

    # ------------------------------------------------------------------ 3. random drifts: trigger + refinement
    random_rows = []
    for number, drift in enumerate(draw_random_drifts()):
        drifted = drift.applied_to(truth)
        injected_rotation, injected_translation = drift_between(truth, drifted)
        row = {"drift_id": number, **{key: value for key, value in drift.__dict__.items()},
               "injected_rotation_deg": injected_rotation, "injected_translation_m": injected_translation}
        for name, alignment in alignments.items():
            row[f"neighbours_worse_{name}"] = neighbours_worse(alignment, drifted)
            row[f"triggered_{name}"] = row[f"neighbours_worse_{name}"] < TRIGGER_BELOW
            refined = alignment.refine_rotation(drifted)
            row[f"residual_rotation_deg_{name}"] = drift_between(truth, refined)[0]
            roll, pitch, yaw = residual_roll_pitch_yaw(truth, refined)
            row[f"residual_roll_deg_{name}"], row[f"residual_pitch_deg_{name}"], row[f"residual_yaw_deg_{name}"] = roll, pitch, yaw
        random_rows.append(row)
        log(f"[{time.time() - started:5.1f} s] drift {number:2d}: injected {injected_rotation:.2f} deg / {100 * injected_translation:.1f} cm | "
            + " | ".join(f"{name} trig {row[f'triggered_{name}']!s:5s} left {row[f'residual_rotation_deg_{name}']:.2f} deg" for name in alignments))
    random_drifts = pd.DataFrame(random_rows)
    random_drifts.to_csv(args.out / f"{args.label}_random_drifts.csv", index=False, float_format="%.4f")

    # Where each score itself peaks when started from the truth: the floor refinement cannot beat.
    score_peak = {name: drift_between(truth, alignment.refine_rotation(truth))[0] for name, alignment in alignments.items()}

    # ------------------------------------------------------------------ summary
    summary = {
        "label": args.label, "drives": drive_info, "tracklet_boxes": dict(labels),
        "settings": {"frame_step": FRAME_STEP, "monitor_frames": len(pick), "monitor_step_deg": MONITOR_STEP_DEG,
                     "monitor_step_m": MONITOR_STEP_M, "trigger_below": TRIGGER_BELOW, "random_seed": RANDOM_SEED,
                     "random_drift_count": RANDOM_DRIFT_COUNT, "random_rotation_limit_deg": RANDOM_ROTATION_LIMIT_DEG,
                     "random_translation_limit_m": RANDOM_TRANSLATION_LIMIT_M},
        "detection_sweep": detection_rows, "methods": {},
    }
    for name in alignments:
        residual = random_drifts[f"residual_rotation_deg_{name}"]
        summary["methods"][name] = {
            "neighbours_worse_at_truth": at_truth[name],
            "false_alarm_at_truth": at_truth[name] < TRIGGER_BELOW,
            "single_axis_detection_rate": float(detection[f"triggered_{name}"].mean()),
            "random_drift_detection_rate": float(random_drifts[f"triggered_{name}"].mean()),
            "residual_rotation_deg_median": float(residual.median()),
            "residual_rotation_deg_p90": float(residual.quantile(0.9)),
            "residual_rotation_deg_max": float(residual.max()),
            **{f"residual_{axis}_deg_median_abs": float(random_drifts[f"residual_{axis}_deg_{name}"].abs().median()) for axis in ROTATION_AXES},
            "score_peak_vs_kitti_deg": score_peak[name],
        }
    summary["injected_rotation_deg_median"] = float(random_drifts["injected_rotation_deg"].median())
    summary["runtime_s"] = time.time() - started
    (args.out / f"{args.label}_summary.json").write_text(json.dumps(summary, indent=2, default=float), encoding="utf-8")

    table = pd.DataFrame({name: {
        "false alarm at truth": f"{'YES' if values['false_alarm_at_truth'] else 'no'} ({values['neighbours_worse_at_truth']:.3f})",
        "single-axis detection": f"{100 * values['single_axis_detection_rate']:.0f} %",
        "random-drift detection": f"{100 * values['random_drift_detection_rate']:.0f} %",
        "residual rot median": f"{values['residual_rotation_deg_median']:.2f} deg",
        "residual rot p90": f"{values['residual_rotation_deg_p90']:.2f} deg",
        "median |roll| left": f"{values['residual_roll_deg_median_abs']:.2f} deg",
        "median |pitch| left": f"{values['residual_pitch_deg_median_abs']:.2f} deg",
        "median |yaw| left": f"{values['residual_yaw_deg_median_abs']:.2f} deg",
        "score peak vs KITTI": f"{values['score_peak_vs_kitti_deg']:.2f} deg",
    } for name, values in summary["methods"].items()})
    log(f"\n[{time.time() - started:5.1f} s] summary for '{args.label}' (drives {' '.join(args.drives)}, "
        f"{len(pick)} monitor frames; random drifts median {summary['injected_rotation_deg_median']:.2f} deg injected)")
    log(table.to_string())
    log(f"\noutputs in {args.out}")


if __name__ == "__main__":
    main()
