"""Score SoftCorr (training-free LiDAR-camera calibration, MoGe-2 depth) on the T3 drift harness.

What this does
    The edge-alignment methods in t3calib/ only refine rotation. SoftCorr
    (third_party/SoftCorr, https://github.com/yuhyun00/SoftCorr) estimates the full 6-DoF extrinsic
    from one image and one LiDAR sweep by maximising a local correlation between MoGe-2 monocular
    depth and projected LiDAR range. This script runs SoftCorr in-process on the SAME 20 random
    drifts as validate_holdout.py (numpy default_rng(7), roll/pitch/yaw uniform +-1.5 deg,
    translation +-5 cm) and scores the corrected extrinsic with the harness's own drift_between.

Conventions (the part that is easy to get wrong)
    harness   velo_to_cam: LiDAR -> unrectified cam0, 4x4. Pixels = P_rect_02 @ R_rect_00 @ velo_to_cam.
    SoftCorr  K (3x3) and T (4x4, LiDAR -> camera); pixels = K @ (R p + t).
    For cam 2:  K = P_rect_02[:, :3]  and  T = [[I, K^-1 P_rect_02[:, 3]], [0, 1]] @ R_rect_00 @ velo_to_cam.
    The script checks this before scoring: with the true calibration, SoftCorr's own projection code
    must land every LiDAR point on the same pixel as t3calib.geometry.project (max difference < 0.5 px).

How several frames become one answer
    SoftCorr is run independently on each chosen frame, starting from the drifted extrinsic.
    The primary answer ("median") is the element-wise median of the per-frame corrections
    (rotation vector and translation of T_frame @ T_start^-1). A second answer ("best_score")
    picks the per-frame estimate whose SoftCorr score, summed over ALL chosen frames, is highest;
    raw per-frame scores are not comparable across scenes, the cross-frame sum is.

Usage (inside .venv-softcorr):
    python softcorr_eval.py --drives 0052 --label holdout
    python softcorr_eval.py --drives 0048 0005 --label dev --time-budget-min 15
Outputs: results/softcorr/<label>_random_drifts.csv, <label>_summary.json, <label>_log.txt.
MoGe-2 depth is cached per frame in .cache/softcorr_monodepth/ (it does not depend on the extrinsic).
"""

from __future__ import annotations

import argparse
import json
import logging
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
import torch
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
SOFTCORR_DIR = ROOT / "third_party" / "SoftCorr"
sys.path.insert(0, str(SOFTCORR_DIR))

from softcorr.config import load_config  # noqa: E402
from softcorr.data import FrameTensors, make_frame  # noqa: E402
from softcorr.monodepth import MonoDepth  # noqa: E402
from softcorr.objective import CalibObjective  # noqa: E402
from softcorr.optim import run_calibration  # noqa: E402

from t3calib.geometry import ROTATION_AXES, drift_between, project  # noqa: E402
from t3calib.kitti import Calibration, Drive  # noqa: E402
from validate_holdout import (RANDOM_DRIFT_COUNT, RANDOM_ROTATION_LIMIT_DEG, RANDOM_SEED,  # noqa: E402
                              RANDOM_TRANSLATION_LIMIT_M, Log, draw_random_drifts, residual_roll_pitch_yaw)

CONVERSION_TOLERANCE_PX = 0.5
COMBINERS = ("median", "best_score")


# ---------------------------------------------------------------------------------------- conventions
def baseline_shift(calibration: Calibration) -> np.ndarray:
    """[[I, K^-1 P_rect_02[:, 3]], [0, 1]]: moves rectified cam0 coordinates to the cam 2 optical centre."""
    shift = np.eye(4)
    shift[:3, 3] = np.linalg.solve(calibration.projection[:, :3], calibration.projection[:, 3])
    return shift


def softcorr_intrinsics(calibration: Calibration) -> np.ndarray:
    return calibration.projection[:, :3].copy()


def to_softcorr(calibration: Calibration, velo_to_cam: np.ndarray) -> np.ndarray:
    return baseline_shift(calibration) @ calibration.rectification @ velo_to_cam


def from_softcorr(calibration: Calibration, lidar_to_camera: np.ndarray) -> np.ndarray:
    return np.linalg.inv(calibration.rectification) @ np.linalg.inv(baseline_shift(calibration)) @ lidar_to_camera


def check_conversion(calibration: Calibration, objective: CalibObjective, frame: FrameTensors,
                     image_size: tuple[int, int], log: Log) -> float:
    """Project the same points with the harness and with SoftCorr's own code; return the max pixel gap."""
    truth = calibration.velo_to_cam
    points = frame.points.double().cpu().numpy()
    harness = project(points, calibration.lidar_to_image(truth), image_size)
    device = frame.points.device
    with torch.no_grad():
        uv, depth = objective._project(frame.points, torch.tensor(to_softcorr(calibration, truth), dtype=torch.float32, device=device))
    uv, depth = uv.double().cpu().numpy(), depth.double().cpu().numpy()
    both = harness.visible & (depth > 1.0)
    gap = np.abs(uv[both] - harness.pixels[both]).max()
    round_trip = np.abs(from_softcorr(calibration, to_softcorr(calibration, truth)) - truth).max()
    log(f"conversion check: {both.sum()} points visible in both projections, max pixel difference {gap:.4f} px "
        f"(tolerance {CONVERSION_TOLERANCE_PX} px); round trip velo_to_cam -> SoftCorr -> velo_to_cam max error {round_trip:.2e}")
    if gap >= CONVERSION_TOLERANCE_PX:
        raise SystemExit("conversion check FAILED - refusing to score with a wrong convention")
    return float(gap)


# ---------------------------------------------------------------------------------------- SoftCorr runs
def correction(start: np.ndarray, estimate: np.ndarray) -> np.ndarray:
    """6-vector (rotation vector, translation) of the left correction estimate @ start^-1."""
    relative = estimate @ np.linalg.inv(start)
    return np.r_[Rotation.from_matrix(relative[:3, :3]).as_rotvec(), relative[:3, 3]]


def apply_correction(start: np.ndarray, vector: np.ndarray) -> np.ndarray:
    relative = np.eye(4)
    relative[:3, :3] = Rotation.from_rotvec(vector[:3]).as_matrix()
    relative[:3, 3] = vector[3:]
    return relative @ start


def calibrate_on_frames(frames: list[FrameTensors], K: torch.Tensor, cfg: dict, start: np.ndarray,
                        device: torch.device) -> dict:
    """Run SoftCorr on every frame from `start` (SoftCorr convention); return per-frame and combined estimates."""
    T_start = torch.tensor(start, dtype=torch.float32, device=device)
    objectives, estimates, own_scores = [], [], []
    for frame in frames:
        objective = CalibObjective(frame, K, cfg["objective"], T_start, device)
        result = run_calibration(objective, T_start, cfg["optim"])
        objectives.append(objective)  # left at the final (finest) bandwidth of the ladder
        estimates.append(result.T.double().cpu().numpy())
        own_scores.append(result.score)
    with torch.no_grad():
        cross_scores = np.array([[float(objective(torch.tensor(estimate, dtype=torch.float32, device=device)).score)
                                  for objective in objectives] for estimate in estimates])
    median = apply_correction(start, np.median([correction(start, estimate) for estimate in estimates], axis=0))
    best = int(cross_scores.sum(axis=1).argmax())
    return {"per_frame": estimates, "own_scores": own_scores, "cross_score_sums": cross_scores.sum(axis=1).tolist(),
            "median": median, "best_score": estimates[best], "best_frame": best}


def residuals(truth: np.ndarray, estimate: np.ndarray) -> dict[str, float]:
    rotation, translation = drift_between(truth, estimate)
    roll, pitch, yaw = residual_roll_pitch_yaw(truth, estimate)
    return {"rotation_deg": rotation, "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw, "translation_cm": 100 * translation}


# ---------------------------------------------------------------------------------------- comparison
def edge_runtime_per_drift(log_path: Path) -> float | None:
    """validate_holdout.py logs one timestamp per drift; both edge variants share each interval."""
    if not log_path.exists():
        return None
    stamps = [float(m.group(1)) for m in re.finditer(r"\[\s*([\d.]+) s\] drift\s+\d+:", log_path.read_text(encoding="utf-8"))]
    return float(np.median(np.diff(stamps)) / 2) if len(stamps) > 2 else None


def comparison_table(label: str, softcorr_rows: pd.DataFrame, softcorr_runtime: float, log: Log) -> list[dict]:
    edge_csv = ROOT / "results" / "holdout" / f"{label}_random_drifts.csv"
    edge_summary_path = ROOT / "results" / "holdout" / f"{label}_summary.json"
    done = softcorr_rows["drift_id"].tolist()
    rows = []
    if edge_csv.exists():
        edge = pd.read_csv(edge_csv)
        edge = edge[edge["drift_id"].isin(done)]
        edge_seconds = edge_runtime_per_drift(ROOT / "results" / "holdout" / f"{label}_log.txt")
        for name in ("paper", "improved"):
            residual = edge[f"residual_rotation_deg_{name}"]
            rows.append({"method": f"edge {name}", "drifts": len(residual), "median_deg": residual.median(),
                         "p90_deg": residual.quantile(0.9), "max_deg": residual.max(),
                         "translation_left_cm_median": 100 * edge["injected_translation_m"].median(),
                         "seconds_per_drift": edge_seconds})
        if edge_summary_path.exists() and len(done) == RANDOM_DRIFT_COUNT:
            stored = json.loads(edge_summary_path.read_text(encoding="utf-8"))["methods"]
            for row, name in zip(rows, ("paper", "improved")):
                assert abs(stored[name]["residual_rotation_deg_median"] - row["median_deg"]) < 1e-3, "edge CSV and summary disagree"
    else:
        log(f"(no {edge_csv.relative_to(ROOT)} - run validate_holdout.py --label {label} for the edge-alignment rows)")
    for combiner in COMBINERS:
        residual = softcorr_rows[f"residual_rotation_deg_{combiner}"]
        rows.append({"method": f"SoftCorr {combiner}", "drifts": len(residual), "median_deg": residual.median(),
                     "p90_deg": residual.quantile(0.9), "max_deg": residual.max(),
                     "translation_left_cm_median": softcorr_rows[f"residual_translation_cm_{combiner}"].median(),
                     "seconds_per_drift": softcorr_runtime})
    table = pd.DataFrame(rows).set_index("method")
    log(f"\ncomparison on the same {len(done)} drift(s) of '{label}' (residual rotation; edge methods do not touch translation, "
        f"so their 'translation left' is the injected offset):")
    log(table.to_string(float_format=lambda x: f"{x:.3f}"))
    return rows


# ---------------------------------------------------------------------------------------- main
def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--drives", nargs="+", required=True)
    parser.add_argument("--label", required=True, help="output name; must match results/holdout/<label>_* to compare")
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data")
    parser.add_argument("--frames-per-drive", type=int, default=3, help="evenly spaced frames per drive")
    parser.add_argument("--max-drifts", type=int, default=RANDOM_DRIFT_COUNT)
    parser.add_argument("--time-budget-min", type=float, default=20.0, help="stop starting new drifts after this")
    parser.add_argument("--out", type=Path, default=ROOT / "results" / "softcorr")
    parser.add_argument("--cache", type=Path, default=ROOT / ".cache" / "softcorr_monodepth")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    log = Log(args.out / f"{args.label}_log.txt")
    logging.basicConfig(level=logging.WARNING, format="    softcorr warning: %(message)s")
    started = time.time()

    if not torch.cuda.is_available():
        raise SystemExit("CUDA is not available in this interpreter - activate .venv-softcorr")
    device = torch.device("cuda")
    commit = subprocess.run(["git", "-C", str(SOFTCORR_DIR), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    cfg = load_config(SOFTCORR_DIR / "configs" / "softcorr.yaml")
    torch.manual_seed(int(cfg["protocol"]["seed"]))
    log(f"SoftCorr {commit[:12]} on the T3 harness | python {sys.version.split()[0]} | torch {torch.__version__} | "
        f"{torch.cuda.get_device_name(0)}")
    log(f"command: python softcorr_eval.py {' '.join(sys.argv[1:])}")
    log(f"SoftCorr config unchanged from configs/softcorr.yaml: ladder {cfg['optim']['sigma_ladder']} x "
        f"{cfg['optim']['steps_per_stage']} steps, max {cfg['data']['max_points']} points")

    # ------------------------------------------------------------------ frames + MoGe-2 depth (cached)
    calibration, raw_frames = None, []
    for name in args.drives:
        drive = Drive(args.data_dir, name)
        calibration = calibration or drive.calibration
        if args.frames_per_drive == 1:
            pick = np.array([len(drive.frame_indices) // 2])  # one frame: take the middle of the drive
        else:
            pick = np.linspace(0, len(drive.frame_indices) - 1, args.frames_per_drive).round().astype(int)
        raw_frames += [drive.load_frame(drive.frame_indices[i]) for i in pick]
    log(f"frames: {', '.join(f'{f.drive}/{f.index}' for f in raw_frames)}")
    monodepth = MonoDepth(args.cache, device)
    torch.cuda.reset_peak_memory_stats()
    depth_started = time.time()
    depths = [monodepth.get(frame.image, f"raw_{frame.drive}", frame.index) for frame in raw_frames]
    peak_moge_gb = torch.cuda.max_memory_allocated() / 2**30
    log(f"[{time.time() - started:6.1f} s] MoGe-2 depth for {len(depths)} frames in {time.time() - depth_started:.1f} s "
        f"(cached ones are instant), peak VRAM {peak_moge_gb:.2f} GB")
    monodepth._model = None
    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()

    frames = [make_frame(frame.image, frame.scan.astype(np.float32), depth, cfg["data"],
                         np.random.default_rng(int(cfg["protocol"]["seed"]) + frame.index), device)
              for frame, depth in zip(raw_frames, depths)]
    K = torch.tensor(softcorr_intrinsics(calibration), dtype=torch.float32, device=device)
    truth = calibration.velo_to_cam
    probe = CalibObjective(frames[0], K, cfg["objective"], torch.tensor(to_softcorr(calibration, truth), dtype=torch.float32, device=device), device)
    conversion_gap = check_conversion(calibration, probe, frames[0], raw_frames[0].image_size, log)
    del probe

    # ------------------------------------------------------------------ does it stay put at the truth?
    t0 = time.time()
    at_truth = calibrate_on_frames(frames, K, cfg, to_softcorr(calibration, truth), device)
    from_truth = {combiner: residuals(truth, from_softcorr(calibration, at_truth[combiner])) for combiner in COMBINERS}
    log(f"[{time.time() - started:6.1f} s] started AT the KITTI calibration ({time.time() - t0:.1f} s): moved away by "
        + " | ".join(f"{c} {r['rotation_deg']:.3f} deg / {r['translation_cm']:.1f} cm" for c, r in from_truth.items())
        + " | per frame " + ", ".join(f"{drift_between(truth, from_softcorr(calibration, e))[0]:.3f} deg" for e in at_truth["per_frame"]))

    # ------------------------------------------------------------------ the 20 validate_holdout.py drifts
    rows, seconds = [], []
    for number, drift in enumerate(draw_random_drifts()[: args.max_drifts]):
        if seconds and (time.time() - started) / 60 + np.mean(seconds) / 60 > args.time_budget_min:
            log(f"time budget of {args.time_budget_min:.0f} min reached - stopping after {len(rows)} drifts")
            break
        t0 = time.time()
        drifted = drift.applied_to(truth)
        injected_rotation, injected_translation = drift_between(truth, drifted)
        run = calibrate_on_frames(frames, K, cfg, to_softcorr(calibration, drifted), device)
        seconds.append(time.time() - t0)
        row = {"drift_id": number, **drift.__dict__, "injected_rotation_deg": injected_rotation,
               "injected_translation_cm": 100 * injected_translation}
        for combiner in COMBINERS:
            for key, value in residuals(truth, from_softcorr(calibration, run[combiner])).items():
                row[f"residual_{key}_{combiner}"] = value
        for i, estimate in enumerate(run["per_frame"]):
            rotation, translation = drift_between(truth, from_softcorr(calibration, estimate))
            row[f"frame{i}_rotation_deg"], row[f"frame{i}_translation_cm"] = rotation, 100 * translation
            row[f"frame{i}_cross_score_sum"] = run["cross_score_sums"][i]
        row["best_frame"], row["seconds"] = run["best_frame"], seconds[-1]
        rows.append(row)
        log(f"[{time.time() - started:6.1f} s] drift {number:2d}: injected {injected_rotation:.2f} deg / {100 * injected_translation:.1f} cm "
            f"-> median left {row['residual_rotation_deg_median']:.3f} deg / {row['residual_translation_cm_median']:.1f} cm, "
            f"best-score left {row['residual_rotation_deg_best_score']:.3f} deg / {row['residual_translation_cm_best_score']:.1f} cm "
            f"({seconds[-1]:.0f} s)")

    table = pd.DataFrame(rows)
    table.to_csv(args.out / f"{args.label}_random_drifts.csv", index=False, float_format="%.4f")
    peak_optimisation_gb = torch.cuda.max_memory_allocated() / 2**30
    comparison = comparison_table(args.label, table, float(np.mean(seconds)), log)

    summary = {
        "label": args.label, "drives": args.drives, "softcorr_commit": commit,
        "frames": [f"{f.drive}/{f.index}" for f in raw_frames], "frames_per_drive": args.frames_per_drive,
        "combiners": {"median": "element-wise median of per-frame (rotation vector, translation) corrections (primary)",
                      "best_score": "per-frame estimate with the highest SoftCorr score summed over all chosen frames"},
        "drift_generator": {"seed": RANDOM_SEED, "count": RANDOM_DRIFT_COUNT, "rotation_limit_deg": RANDOM_ROTATION_LIMIT_DEG,
                            "translation_limit_m": RANDOM_TRANSLATION_LIMIT_M, "drifts_evaluated": len(rows)},
        "conversion_check_max_px": conversion_gap, "from_truth": from_truth,
        "methods": {combiner: {
            "residual_rotation_deg_median": float(table[f"residual_rotation_deg_{combiner}"].median()),
            "residual_rotation_deg_p90": float(table[f"residual_rotation_deg_{combiner}"].quantile(0.9)),
            "residual_rotation_deg_max": float(table[f"residual_rotation_deg_{combiner}"].max()),
            "residual_translation_cm_median": float(table[f"residual_translation_cm_{combiner}"].median()),
            "residual_translation_cm_max": float(table[f"residual_translation_cm_{combiner}"].max()),
            **{f"residual_{axis}_deg_median_abs": float(table[f"residual_{axis}_deg_{combiner}"].abs().median()) for axis in ROTATION_AXES},
        } for combiner in COMBINERS},
        "injected_rotation_deg_median": float(table["injected_rotation_deg"].median()),
        "injected_translation_cm_median": float(table["injected_translation_cm"].median()),
        "comparison": comparison,
        "seconds_per_drift_mean": float(np.mean(seconds)), "seconds_per_frame_mean": float(np.mean(seconds)) / len(frames),
        "peak_vram_gb": {"moge2_inference": peak_moge_gb, "softcorr_optimisation": peak_optimisation_gb},
        "gpu": torch.cuda.get_device_name(0), "torch": torch.__version__, "runtime_s": time.time() - started,
    }
    (args.out / f"{args.label}_summary.json").write_text(json.dumps(summary, indent=2, default=float), encoding="utf-8")
    log(f"\n[{time.time() - started:6.1f} s] peak VRAM: MoGe-2 {peak_moge_gb:.2f} GB, SoftCorr optimisation {peak_optimisation_gb:.2f} GB; "
        f"outputs in {args.out}")


if __name__ == "__main__":
    main()
