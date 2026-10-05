"""Drift check from object detections: compare LiDAR objects projected into the image with camera detections.

The idea (the teacher's suggestion): in deployment there is no ground-truth point cloud painted on the
image, but both sensors already detect objects. Detect objects in 3D from the LiDAR, project each 3D box
into the image with the CURRENT extrinsic, and compare it with the camera detector's 2D box for the same
object. If the calibration is right the two boxes sit on top of each other; if the LiDAR has turned on its
mount, every projected box slides the same way, and that offset IS the drift.

What is real and what is a stand-in
  camera side   a real pretrained COCO detector (Ultralytics YOLO26n, falling back to YOLOv8n) run on the
                left colour images. Classes kept: car, truck, bus, person, bicycle, motorcycle, conf >= 0.4.
  LiDAR side    the KITTI tracklet 3D boxes. They play the role of a LiDAR 3D detector, which makes this an
                UPPER BOUND: a real LiDAR detector (PointPillars, CenterPoint ...) misses objects and has
                box noise of its own, so real numbers will be worse.

Procedure
  1. camera detections per frame, cached in results/detection_check/detections_<drive>.json
     (they do not depend on the calibration, so they are computed once)
  2. for an extrinsic T, project the 8 corners of every tracklet box -> 2D box (clipped to the image;
     skipped if a corner is behind the camera or the box is under 12 px)
  3. at the TRUE calibration, pair projected boxes with camera boxes by greedy IoU (>= 0.3, compatible
     class). The pairs are then frozen: for drifted extrinsics only the projected box moves, so the
     signal is the offset, not a change in who matched whom.
  4. per extrinsic: mean IoU of the frozen pairs, median centre offset du, dv (projected minus camera,
     pixels) and the share of pairs whose IoU fell below 0.5
  5. object-level re-calibration: yaw_hat ~ atan(du / f), pitch_hat ~ atan(dv / f), f = P_rect_02[0, 0],
     with the signs measured on a known +1 deg drift on the development drives. Roll turns boxes about the
     image centre instead of sliding them, so the median offset cannot see it.

Drives: 0048 + 0005 are the development drives (signs, detector bias and thresholds are read there,
at the true calibration only); 0052 is the held-out drive, as in validate_holdout.py.

Usage (in the .venv-yolo environment, which has ultralytics):
    .venv-yolo/Scripts/python detection_check.py
Outputs: results/detection_check/{summary.json, sweep.csv, random_drifts_0052.csv, detection_check.png, log.txt}
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

from t3calib.geometry import ROTATION_AXES, Drift, drift_between, project  # noqa: E402
from t3calib.kitti import Calibration, Drive, TrackedObject  # noqa: E402
from validate_holdout import FRAME_STEP, Log, draw_random_drifts, residual_roll_pitch_yaw  # noqa: E402

ROOT = Path(__file__).parent
DEV_DRIVES = ("0048", "0005")
HOLDOUT_DRIVE = "0052"

KEPT_CLASSES = {"person", "bicycle", "car", "motorcycle", "bus", "truck"}
MIN_CONFIDENCE = 0.4
DETECTOR_IMAGE_SIZE = 1280  # KITTI images are 1242 x 375; the default 640 would halve every object
WEIGHT_CANDIDATES = ("yolo26n.pt", "yolov8n.pt")

# Which camera classes may stand for which tracklet type (a van is often called car or truck by COCO).
COMPATIBLE_CLASSES = {
    "Car": {"car", "truck"},
    "Van": {"car", "truck", "bus"},
    "Truck": {"truck", "bus", "car"},
    "Tram": {"bus", "truck"},
    "Misc": {"car", "truck", "bus"},
    "Pedestrian": {"person"},
    "Person_sitting": {"person"},
    "Cyclist": {"person", "bicycle", "motorcycle"},
}

MIN_BOX_PX = 12
MATCH_IOU = 0.3
LOW_IOU = 0.5
IOU_DROP_FLAG = 0.05  # flag when the mean IoU falls this far below its value at the true calibration
SWEEP_DEG = [0, 0.25, 0.5, 1.0, 2.0]
REALISTIC_DRIFT = Drift(roll=0.3, pitch=-0.8, yaw=1.2, tx=0.05)  # same as run_benchmark.py
BOOTSTRAP_ROUNDS = 2000
IMAGE_SIZE = (1242, 375)  # width, height of image_02 in every 2011_09_26 drive

Box = tuple[float, float, float, float]  # u0, v0, u1, v1


@dataclass
class CameraDetection:
    label: str
    confidence: float
    box: Box


@dataclass
class Pair:
    """A LiDAR object and the camera detection it matched at the true calibration."""

    drive: str
    frame_index: int
    lidar_object: TrackedObject
    camera: CameraDetection
    iou_at_truth: float


# ---------------------------------------------------------------------------------------------- boxes


def iou(a: Box, b: Box) -> float:
    overlap_u = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    overlap_v = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    overlap = overlap_u * overlap_v
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - overlap
    return overlap / union if union > 0 else 0.0


def centre(box: Box) -> tuple[float, float]:
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2


def projected_box(lidar_object: TrackedObject, lidar_to_image: np.ndarray, image_size: tuple[int, int],
                  require_inside: bool = False) -> Box | None:
    """2D box around the 8 projected corners, clipped to the image; None if unusable.

    require_inside also rejects boxes that would need clipping, which is used when the pairs are formed:
    a clipped box cannot slide past the image border, so it would hide part of a drift.
    """
    corners = project(lidar_object.corners(), lidar_to_image, image_size, min_depth=0.5)
    if not np.all(corners.depth > 0.5):
        return None  # a corner is behind (or at) the camera
    width, height = image_size
    u0, v0 = corners.pixels.min(axis=0)
    u1, v1 = corners.pixels.max(axis=0)
    if require_inside and (u0 < 0 or v0 < 0 or u1 > width - 1 or v1 > height - 1):
        return None
    u0, u1 = np.clip([u0, u1], 0, width - 1)
    v0, v1 = np.clip([v0, v1], 0, height - 1)
    if u1 - u0 < MIN_BOX_PX or v1 - v0 < MIN_BOX_PX:
        return None
    return float(u0), float(v0), float(u1), float(v1)


# ---------------------------------------------------------------------------------------------- camera detector


def load_detector(weights_dir: Path):
    from ultralytics import YOLO  # imported here so the rest of the file loads without ultralytics

    weights_dir.mkdir(parents=True, exist_ok=True)
    errors = []
    for name in WEIGHT_CANDIDATES:
        try:
            return YOLO(str(weights_dir / name)), name  # downloads from the Ultralytics GitHub release if missing
        except Exception as error:  # noqa: BLE001 - try the next candidate, report all failures at the end
            errors.append(f"{name}: {error}")
    raise RuntimeError("no detector weights could be loaded:\n" + "\n".join(errors))


def camera_detections(drive: Drive, frame_indices: list[int], cache_dir: Path, detector_state: dict, log: Log) -> dict[int, list[CameraDetection]]:
    """Detections per frame index, read from the cache when it already covers these frames."""
    cache = cache_dir / f"detections_{drive.name}.json"
    if cache.exists():
        stored = json.loads(cache.read_text(encoding="utf-8"))
        if all(str(index) in stored["frames"] for index in frame_indices):
            log(f"drive {drive.name}: camera detections read from cache ({stored['detector']})")
            detector_state.setdefault("name", stored["detector"])
            return {index: [CameraDetection(d["label"], d["confidence"], tuple(d["box"])) for d in stored["frames"][str(index)]]
                    for index in frame_indices}

    if "model" not in detector_state:
        detector_state["model"], detector_state["name"] = load_detector(ROOT / ".cache" / "ultralytics")
        log(f"detector loaded: {detector_state['name']}")
    model = detector_state["model"]
    started = time.time()
    frames: dict[str, list[dict]] = {}
    for index in frame_indices:
        image_path = drive.sync_dir / "image_02" / "data" / f"{index:010d}.png"
        result = model.predict(str(image_path), imgsz=DETECTOR_IMAGE_SIZE, conf=MIN_CONFIDENCE, verbose=False)[0]
        kept = []
        for box, class_id, confidence in zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist(), result.boxes.conf.tolist()):
            label = result.names[int(class_id)]
            if label in KEPT_CLASSES and confidence >= MIN_CONFIDENCE:
                kept.append({"label": label, "confidence": round(float(confidence), 4), "box": [round(value, 2) for value in box]})
        frames[str(index)] = kept
    cache.write_text(json.dumps({"detector": detector_state["name"], "image_size": DETECTOR_IMAGE_SIZE,
                                 "min_confidence": MIN_CONFIDENCE, "classes": sorted(KEPT_CLASSES), "frames": frames}, indent=1),
                     encoding="utf-8")
    log(f"drive {drive.name}: detector ran on {len(frame_indices)} frames in {time.time() - started:.1f} s, cached to {cache.name}")
    return {index: [CameraDetection(d["label"], d["confidence"], tuple(d["box"])) for d in frames[str(index)]] for index in frame_indices}


# ---------------------------------------------------------------------------------------------- matching and metrics


def match_at_truth(drive: Drive, frame_indices: list[int], detections: dict[int, list[CameraDetection]]) -> list[Pair]:
    """Greedy IoU matching between projected tracklet boxes and camera boxes, at the true calibration."""
    calibration = drive.calibration
    matrix = calibration.lidar_to_image()
    image_size = IMAGE_SIZE
    pairs = []
    for index in frame_indices:
        lidar = [(obj, projected_box(obj, matrix, image_size, require_inside=True)) for obj in drive.objects_by_frame.get(index, [])]
        lidar = [(obj, box) for obj, box in lidar if box is not None]
        camera = detections[index]
        candidates = sorted(((iou(box, detection.box), i, j) for i, (obj, box) in enumerate(lidar)
                             for j, detection in enumerate(camera) if detection.label in COMPATIBLE_CLASSES.get(obj.label, set())),
                            reverse=True)
        used_lidar, used_camera = set(), set()
        for overlap, i, j in candidates:
            if overlap < MATCH_IOU:
                break
            if i in used_lidar or j in used_camera:
                continue
            used_lidar.add(i)
            used_camera.add(j)
            pairs.append(Pair(drive.name, index, lidar[i][0], camera[j], overlap))
    return pairs


def evaluate(pairs: list[Pair], calibration: Calibration, extrinsic: np.ndarray, image_size=IMAGE_SIZE) -> dict[str, float]:
    """Mean IoU, median centre offsets and low-IoU share of the frozen pairs under one extrinsic."""
    matrix = calibration.lidar_to_image(extrinsic)
    overlaps, offsets_u, offsets_v = [], [], []
    for pair in pairs:
        box = projected_box(pair.lidar_object, matrix, image_size)
        if box is None:
            overlaps.append(0.0)  # the object fell out of the image: counts as a total miss
            continue
        overlaps.append(iou(box, pair.camera.box))
        (u_lidar, v_lidar), (u_camera, v_camera) = centre(box), centre(pair.camera.box)
        offsets_u.append(u_lidar - u_camera)
        offsets_v.append(v_lidar - v_camera)
    overlaps = np.array(overlaps)
    return {
        "mean_iou": float(overlaps.mean()),
        "median_du_px": float(np.median(offsets_u)),
        "median_dv_px": float(np.median(offsets_v)),
        "share_iou_below_0.5": float(np.mean(overlaps < LOW_IOU)),
        "pairs_still_in_image": len(offsets_u),
    }


@dataclass
class Corrector:
    """Object-level re-calibration: turn the median offset into a yaw and pitch correction."""

    focal_length_px: float
    yaw_sign: float  # measured on a known +1 deg yaw drift (dev drives)
    pitch_sign: float
    bias_u_px: float = 0.0  # offset already present at the true calibration (detector vs 3D-box convention)
    bias_v_px: float = 0.0

    def estimate(self, median_du: float, median_dv: float) -> tuple[float, float]:
        yaw = self.yaw_sign * np.degrees(np.arctan((median_du - self.bias_u_px) / self.focal_length_px))
        pitch = self.pitch_sign * np.degrees(np.arctan((median_dv - self.bias_v_px) / self.focal_length_px))
        return float(yaw), float(pitch)

    def correct(self, drifted: np.ndarray, median_du: float, median_dv: float) -> np.ndarray:
        yaw, pitch = self.estimate(median_du, median_dv)
        return drifted @ np.linalg.inv(Drift(pitch=pitch, yaw=yaw).matrix())


def describe(pairs, calibration, truth, drifted, corrector_raw, corrector_debiased, flag_rule) -> dict:
    """Every number reported for one drifted extrinsic."""
    row = evaluate(pairs, calibration, drifted)
    row["injected_rotation_deg"] = drift_between(truth, drifted)[0]
    row.update(flag_rule(row))
    for name, corrector in (("raw", corrector_raw), ("debiased", corrector_debiased)):
        yaw_hat, pitch_hat = corrector.estimate(row["median_du_px"], row["median_dv_px"])
        corrected = corrector.correct(drifted, row["median_du_px"], row["median_dv_px"])
        row[f"yaw_hat_deg_{name}"], row[f"pitch_hat_deg_{name}"] = yaw_hat, pitch_hat
        row[f"residual_rotation_deg_{name}"] = drift_between(truth, corrected)[0]
        roll, pitch, yaw = residual_roll_pitch_yaw(truth, corrected)
        row[f"residual_roll_deg_{name}"], row[f"residual_pitch_deg_{name}"], row[f"residual_yaw_deg_{name}"] = roll, pitch, yaw
    return row


def bootstrap_spread(pairs, calibration, truth, rounds: int, sample_size: int, seed: int = 0) -> dict[str, float]:
    """How much mean IoU and the median offsets wobble at the TRUE calibration when the pairs are resampled."""
    matrix = calibration.lidar_to_image(truth)
    overlaps, du, dv = [], [], []
    for pair in pairs:
        box = projected_box(pair.lidar_object, matrix, IMAGE_SIZE)
        overlaps.append(iou(box, pair.camera.box))
        du.append(centre(box)[0] - centre(pair.camera.box)[0])
        dv.append(centre(box)[1] - centre(pair.camera.box)[1])
    overlaps, du, dv = np.array(overlaps), np.array(du), np.array(dv)
    generator = np.random.default_rng(seed)
    picks = generator.integers(0, len(pairs), size=(rounds, sample_size))
    return {"mean_iou_std": float(overlaps[picks].mean(axis=1).std()),
            "median_du_std_px": float(np.median(du[picks], axis=1).std()),
            "median_dv_std_px": float(np.median(dv[picks], axis=1).std()),
            "sample_size": sample_size}


# ---------------------------------------------------------------------------------------------- figure


def draw_frame(ax, drive: Drive, frame_index: int, pairs: list[Pair], extrinsic: np.ndarray, title: str) -> None:
    frame = drive.load_frame(frame_index)
    ax.imshow((frame.image * 0.7).astype(np.uint8))
    matrix = drive.calibration.lidar_to_image(extrinsic)
    for pair in pairs:
        u0, v0, u1, v1 = pair.camera.box
        ax.add_patch(Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, edgecolor="#22d3ee", linewidth=1.6))
        box = projected_box(pair.lidar_object, matrix, frame.image_size)
        if box is not None:
            u0, v0, u1, v1 = box
            ax.add_patch(Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, edgecolor="#f97316", linewidth=1.6, linestyle="--"))
    ax.set_title(title, fontsize=10, loc="left")
    ax.axis("off")


def make_figure(path: Path, sweep: pd.DataFrame, holdout_drive: Drive, holdout_pairs: list[Pair], truth: np.ndarray,
                realistic: np.ndarray, corrected: np.ndarray, realistic_row: dict, threshold: float) -> None:
    figure = plt.figure(figsize=(14, 17))
    grid = figure.add_gridspec(4, 3, height_ratios=[1.15, 1, 1, 1], hspace=0.22, wspace=0.25)
    colours = {"dev": "#64748b", "holdout": "#ea580c"}
    for column, axis in enumerate(("yaw", "pitch", "roll")):
        ax = figure.add_subplot(grid[0, column])
        for drive_set, label in (("dev", "dev 0048+0005"), ("holdout", "held-out 0052")):
            rows = sweep[(sweep.drive_set == drive_set) & (sweep.axis == axis)]
            ax.plot(rows.magnitude_deg, rows.mean_iou, marker="o", color=colours[drive_set], label=label)
        ax.axhline(threshold, color=colours["holdout"], linestyle=":", linewidth=1, label="flag line 0052 (truth - 0.05)")
        ax.set_title(f"{axis} drift", fontsize=11)
        ax.set_xlabel("drift (deg)")
        ax.set_ylabel("mean IoU of frozen pairs" if column == 0 else "")
        ax.set_ylim(0, 1)
        ax.grid(alpha=0.3)
        if column == 0:
            ax.legend(fontsize=8, loc="lower left")
    counts = pd.Series([pair.frame_index for pair in holdout_pairs]).value_counts()
    example = int(counts.index[0])
    example_pairs = [pair for pair in holdout_pairs if pair.frame_index == example]
    panels = [
        (truth, f"0052 frame {example}, true calibration: camera box (cyan) vs projected LiDAR box (orange, dashed)"),
        (realistic, f"realistic drift roll 0.3 / pitch -0.8 / yaw 1.2 deg, tx 5 cm: mean IoU {realistic_row['mean_iou']:.2f}, "
                    f"du {realistic_row['median_du_px']:+.1f} px, dv {realistic_row['median_dv_px']:+.1f} px"),
        (corrected, f"after object-level correction (yaw/pitch from the median offset): "
                    f"{realistic_row['residual_rotation_deg_debiased']:.2f} deg left (roll is not seen)"),
    ]
    for row, (extrinsic, title) in enumerate(panels, start=1):
        draw_frame(figure.add_subplot(grid[row, :]), holdout_drive, example, example_pairs, extrinsic, title)
    figure.suptitle("Drift check from detections: projected LiDAR objects vs camera detector (KITTI tracklets stand in for a LiDAR detector)",
                    fontsize=12)
    figure.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(figure)


# ---------------------------------------------------------------------------------------------- main


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data")
    parser.add_argument("--out", type=Path, default=ROOT / "results" / "detection_check")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    log = Log(args.out / "log.txt")
    started = time.time()
    log(f"detection-based drift check | python {sys.version.split()[0]} | numpy {np.__version__}")

    # ------------------------------------------------------------------ 1-3. detections and frozen pairs
    detector_state: dict = {}
    drives, pairs_by_drive = {}, {}
    for name in (*DEV_DRIVES, HOLDOUT_DRIVE):
        drive = Drive(args.data_dir, name)
        indices = drive.frame_indices[::FRAME_STEP]
        detections = camera_detections(drive, indices, args.out, detector_state, log)
        pairs = match_at_truth(drive, indices, detections)
        drives[name], pairs_by_drive[name] = drive, pairs
        tracklet_count = sum(len(drive.objects_by_frame.get(index, [])) for index in indices)
        log(f"drive {name}: {len(indices)} frames, {sum(map(len, detections.values()))} camera boxes kept, "
            f"{tracklet_count} tracklet boxes, {len(pairs)} pairs matched at truth "
            f"(median IoU {np.median([p.iou_at_truth for p in pairs]):.2f})")
    calibration = drives[HOLDOUT_DRIVE].calibration
    truth = calibration.velo_to_cam
    focal = calibration.focal_length_px
    dev_pairs = [pair for name in DEV_DRIVES for pair in pairs_by_drive[name]]
    holdout_pairs = pairs_by_drive[HOLDOUT_DRIVE]
    pair_sets = {"dev": dev_pairs, "holdout": holdout_pairs}

    # ------------------------------------------------------------------ dev-only calibration of the check itself
    dev_truth = evaluate(dev_pairs, calibration, truth)
    yaw_probe = evaluate(dev_pairs, calibration, Drift(yaw=1.0).applied_to(truth))
    pitch_probe = evaluate(dev_pairs, calibration, Drift(pitch=1.0).applied_to(truth))
    yaw_sign = float(np.sign(yaw_probe["median_du_px"] - dev_truth["median_du_px"]))
    pitch_sign = float(np.sign(pitch_probe["median_dv_px"] - dev_truth["median_dv_px"]))
    log(f"sign check on dev, +1 deg yaw: du {dev_truth['median_du_px']:+.2f} -> {yaw_probe['median_du_px']:+.2f} px "
        f"(expected about {focal * np.tan(np.radians(1)):.1f} px) -> yaw_hat = {yaw_sign:+.0f} * atan(du / f)")
    log(f"sign check on dev, +1 deg pitch: dv {dev_truth['median_dv_px']:+.2f} -> {pitch_probe['median_dv_px']:+.2f} px "
        f"-> pitch_hat = {pitch_sign:+.0f} * atan(dv / f)")
    raw = Corrector(focal, yaw_sign, pitch_sign)
    debiased = Corrector(focal, yaw_sign, pitch_sign, dev_truth["median_du_px"], dev_truth["median_dv_px"])
    spread = bootstrap_spread(dev_pairs, calibration, truth, BOOTSTRAP_ROUNDS, sample_size=len(holdout_pairs))
    offset_tolerance = max(3.0, 4 * max(spread["median_du_std_px"], spread["median_dv_std_px"]))
    log(f"dev at truth: mean IoU {dev_truth['mean_iou']:.3f}, du {dev_truth['median_du_px']:+.2f} px, dv {dev_truth['median_dv_px']:+.2f} px; "
        f"bootstrap ({spread['sample_size']} pairs): IoU std {spread['mean_iou_std']:.3f}, du std {spread['median_du_std_px']:.2f} px, "
        f"dv std {spread['median_dv_std_px']:.2f} px -> offset tolerance {offset_tolerance:.1f} px")

    baselines = {drive_set: evaluate(pairs, calibration, truth)["mean_iou"] for drive_set, pairs in pair_sets.items()}

    def flag_rule_for(drive_set: str):
        def flag(row: dict) -> dict:
            return {
                # primary rule: IoU fell 0.05 below its value at the true calibration (reference recorded right after calibration)
                "flag_iou": row["mean_iou"] < baselines[drive_set] - IOU_DROP_FLAG,
                # deployable rule: offsets moved beyond what the detector noise explains (bias and tolerance from dev truth only)
                "flag_offset": bool(abs(row["median_du_px"] - debiased.bias_u_px) > offset_tolerance
                                    or abs(row["median_dv_px"] - debiased.bias_v_px) > offset_tolerance),
            }
        return flag

    # ------------------------------------------------------------------ 5. sweeps
    rows = []
    for drive_set, pairs in pair_sets.items():
        for axis in ROTATION_AXES:
            for magnitude in SWEEP_DEG:
                drifted = Drift.along(axis, magnitude).applied_to(truth)
                rows.append({"drive_set": drive_set, "axis": axis, "magnitude_deg": magnitude,
                             **describe(pairs, calibration, truth, drifted, raw, debiased, flag_rule_for(drive_set))})
        realistic = REALISTIC_DRIFT.applied_to(truth)
        rows.append({"drive_set": drive_set, "axis": "realistic", "magnitude_deg": np.nan,
                     **describe(pairs, calibration, truth, realistic, raw, debiased, flag_rule_for(drive_set))})
    sweep = pd.DataFrame(rows)
    sweep.to_csv(args.out / "sweep.csv", index=False, float_format="%.4f")
    shown = ["drive_set", "axis", "magnitude_deg", "mean_iou", "median_du_px", "median_dv_px", "share_iou_below_0.5",
             "flag_iou", "flag_offset", "yaw_hat_deg_debiased", "pitch_hat_deg_debiased", "residual_rotation_deg_debiased"]
    log(f"\n[{time.time() - started:5.1f} s] single-axis sweep + realistic drift")
    log(sweep[shown].to_string(index=False, float_format=lambda x: f"{x:.3f}"))

    random_rows = []
    for number, drift in enumerate(draw_random_drifts()):
        drifted = drift.applied_to(truth)
        random_rows.append({"drift_id": number, **drift.__dict__,
                            **describe(holdout_pairs, calibration, truth, drifted, raw, debiased, flag_rule_for("holdout"))})
    random_drifts = pd.DataFrame(random_rows)
    random_drifts.to_csv(args.out / f"random_drifts_{HOLDOUT_DRIVE}.csv", index=False, float_format="%.4f")
    log(f"\n[{time.time() - started:5.1f} s] 20 random drifts on {HOLDOUT_DRIVE} (validate_holdout.py generator, seed 7)")
    log(random_drifts[["drift_id", "roll", "pitch", "yaw", "injected_rotation_deg", "mean_iou", "flag_iou", "flag_offset",
                       "residual_rotation_deg_raw", "residual_rotation_deg_debiased", "residual_roll_deg_debiased"]]
        .to_string(index=False, float_format=lambda x: f"{x:.3f}"))

    # ------------------------------------------------------------------ summary + comparison with the edge monitor
    edge = json.loads((ROOT / "results" / "holdout" / "holdout_summary.json").read_text(encoding="utf-8"))["methods"]
    holdout_sweep = sweep[(sweep.drive_set == "holdout") & (sweep.axis != "realistic")]
    single_axis_nonzero = holdout_sweep[holdout_sweep.magnitude_deg > 0]
    at_truth_holdout = holdout_sweep[holdout_sweep.magnitude_deg == 0].iloc[0]
    realistic_holdout = sweep[(sweep.drive_set == "holdout") & (sweep.axis == "realistic")].iloc[0]
    method = {}
    for variant in ("raw", "debiased"):
        residual = random_drifts[f"residual_rotation_deg_{variant}"]
        method[variant] = {
            "residual_rotation_deg_median": float(residual.median()),
            "residual_rotation_deg_p90": float(residual.quantile(0.9)),
            "residual_rotation_deg_max": float(residual.max()),
            **{f"residual_{axis}_deg_median_abs": float(random_drifts[f"residual_{axis}_deg_{variant}"].abs().median()) for axis in ROTATION_AXES},
            "realistic_drift_residual_deg": float(realistic_holdout[f"residual_rotation_deg_{variant}"]),
        }
    summary = {
        "detector": detector_state.get("name"),
        "lidar_side": "KITTI tracklet 3D boxes as a stand-in for a LiDAR 3D detector (upper bound)",
        "settings": {"frame_step": FRAME_STEP, "classes": sorted(KEPT_CLASSES), "min_confidence": MIN_CONFIDENCE,
                     "detector_image_size": DETECTOR_IMAGE_SIZE, "match_iou": MATCH_IOU, "min_box_px": MIN_BOX_PX,
                     "iou_drop_flag": IOU_DROP_FLAG, "offset_tolerance_px": offset_tolerance, "focal_length_px": focal},
        "pairs_at_truth": {name: len(pairs) for name, pairs in pairs_by_drive.items()},
        "signs": {"yaw_sign": yaw_sign, "pitch_sign": pitch_sign,
                  "dev_du_at_truth_px": dev_truth["median_du_px"], "dev_du_at_yaw_plus_1deg_px": yaw_probe["median_du_px"],
                  "dev_dv_at_truth_px": dev_truth["median_dv_px"], "dev_dv_at_pitch_plus_1deg_px": pitch_probe["median_dv_px"]},
        "dev_truth": dev_truth, "dev_truth_bootstrap": spread,
        "mean_iou_at_truth": baselines,
        "flag_rules": {
            "flag_iou": "mean IoU of the frozen pairs < its value at the true calibration on the same drive - 0.05",
            "flag_offset": f"|median du - dev bias| or |median dv - dev bias| > {offset_tolerance:.1f} px "
                           "(bias and tolerance = max(3 px, 4 x bootstrap std) from dev drives at the true calibration)",
        },
        "holdout": {
            "false_alarm_at_truth": {"flag_iou": bool(at_truth_holdout.flag_iou), "flag_offset": bool(at_truth_holdout.flag_offset)},
            "single_axis_detection_rate": {"flag_iou": float(single_axis_nonzero.flag_iou.mean()),
                                           "flag_offset": float(single_axis_nonzero.flag_offset.mean())},
            "random_drift_detection_rate": {"flag_iou": float(random_drifts.flag_iou.mean()),
                                            "flag_offset": float(random_drifts.flag_offset.mean())},
            "correction": method,
            "realistic_drift": {key: realistic_holdout[key] for key in ("mean_iou", "median_du_px", "median_dv_px", "share_iou_below_0.5",
                                                                        "flag_iou", "flag_offset", "yaw_hat_deg_debiased", "pitch_hat_deg_debiased")},
        },
        "edge_monitor_holdout_0052": {name: {key: values[key] for key in ("random_drift_detection_rate", "single_axis_detection_rate",
                                                                           "residual_rotation_deg_median", "residual_rotation_deg_p90",
                                                                           "residual_roll_deg_median_abs")}
                                      for name, values in edge.items()},
        "note_single_axis_rates": "edge single-axis rate also includes tx/ty/tz drifts; this check sweeps rotation only",
        "runtime_s": time.time() - started,
    }
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2, default=lambda value: value.item() if hasattr(value, "item") else float(value)),
                                           encoding="utf-8")

    corrected = debiased.correct(realistic, realistic_holdout.median_du_px, realistic_holdout.median_dv_px)
    make_figure(args.out / "detection_check.png", sweep, drives[HOLDOUT_DRIVE], holdout_pairs, truth,
                REALISTIC_DRIFT.applied_to(truth), corrected, realistic_holdout.to_dict(), baselines["holdout"] - IOU_DROP_FLAG)

    log(f"\n[{time.time() - started:5.1f} s] held-out {HOLDOUT_DRIVE}: random-drift detection IoU rule "
        f"{100 * summary['holdout']['random_drift_detection_rate']['flag_iou']:.0f} %, offset rule "
        f"{100 * summary['holdout']['random_drift_detection_rate']['flag_offset']:.0f} %; residual (debiased) median "
        f"{method['debiased']['residual_rotation_deg_median']:.2f} deg, p90 {method['debiased']['residual_rotation_deg_p90']:.2f} deg")
    for name, values in summary["edge_monitor_holdout_0052"].items():
        log(f"edge monitor ({name}) on {HOLDOUT_DRIVE}: detection {100 * values['random_drift_detection_rate']:.0f} %, residual median "
            f"{values['residual_rotation_deg_median']:.2f} deg, p90 {values['residual_rotation_deg_p90']:.2f} deg")
    log(f"outputs in {args.out}")


if __name__ == "__main__":
    main()
