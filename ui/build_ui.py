"""Build the self-contained class-demo page ui/index.html from KITTI raw data.

    python ui/build_ui.py                               # pick 4 showcase frames, write ui/index.html
    python ui/build_ui.py --parity-reference 0005:142   # print Python reference numbers as JSON (used by check_parity.mjs)

Everything the page needs is embedded as JSON: the left colour image (JPEG), the LiDAR points that can land in
the image, the LiDAR depth-edge points + weights, the image edge field (uint8), the true 2D boxes with each
object's own LiDAR points, and the calibration. The page re-implements score / health check / refinement /
metrics in plain JS (the <script id="t3-core"> block); ui/check_parity.mjs runs that same block in Node and
compares it with t3calib.
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from run_benchmark import MONITOR_STEP_DEG, MONITOR_STEP_M, REALISTIC_DRIFT  # noqa: E402
from t3calib.edge_alignment import EdgeAlignment, image_edge_field, lidar_depth_edges  # noqa: E402
from t3calib.geometry import Drift, drift_between, project  # noqa: E402
from t3calib.kitti import Drive  # noqa: E402
from t3calib.metrics import association_and_range, collect_objects, reprojection_error_by_depth  # noqa: E402
from validate_holdout import TRIGGER_BELOW  # noqa: E402

DATA_DIR = ROOT / "data"
OUT = Path(__file__).resolve().parent / "index.html"
REQUIRED = {"dev": ("0005", 142), "held-out": ("0052", 28)}
SECOND_DRIVE = {"dev": "0048", "held-out": "0052"}
MAX_POINTS = 40_000
IMAGE_MARGIN_PX = 300  # keep LiDAR points landing within this margin of the image under the true calibration
JPEG_QUALITY = 85
PARITY_DRIFTS = {"none": Drift(), "realistic": REALISTIC_DRIFT, "yaw 2.5": Drift(yaw=2.5)}


def log(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


def b64(array: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(array).tobytes()).decode("ascii")


def quantised_field(image: np.ndarray) -> np.ndarray:
    return np.round(image_edge_field(image) * 255).clip(0, 255).astype(np.uint8)


def frame_quality(drive: Drive, index: int) -> dict:
    """How good a showcase this frame is: labelled objects, and whether the monitor behaves on it."""
    calibration = drive.calibration
    frame = drive.load_frame(index)
    objects = collect_objects([frame], calibration)
    alignment = EdgeAlignment([frame], calibration, across_rings=True)
    truth = calibration.velo_to_cam
    drifted = REALISTIC_DRIFT.applied_to(truth)
    truth_health = alignment.health_check(truth, MONITOR_STEP_DEG, MONITOR_STEP_M).fraction_of_neighbours_worse
    drift_health = alignment.health_check(drifted, MONITOR_STEP_DEG, MONITOR_STEP_M).fraction_of_neighbours_worse
    residual_deg = drift_between(truth, alignment.refine_rotation(drifted))[0]
    behaves = truth_health >= TRIGGER_BELOW and drift_health < TRIGGER_BELOW and residual_deg < 0.3
    return {"index": index, "objects": len(objects), "truth_health": truth_health, "drift_health": drift_health,
            "residual_deg": residual_deg, "behaves": behaves}


def best_frame(drive: Drive, avoid_near: int | None, candidates: int = 8) -> dict:
    """Frame with the most clearly visible labelled objects among those where the monitor behaves."""
    def visible_count(index: int) -> int:
        return sum(obj.is_clearly_visible() for obj in drive.objects_by_frame.get(index, []))

    pool = [i for i in drive.frame_indices if avoid_near is None or abs(i - avoid_near) > 15]
    pool.sort(key=lambda i: (-visible_count(i), i))
    rated = [frame_quality(drive, index) for index in pool[:candidates]]
    for row in rated:
        log(f"  candidate {drive.name}:{row['index']:3d}  objects {row['objects']}  health truth {row['truth_health']:.0%}  "
            f"realistic {row['drift_health']:.0%}  residual {row['residual_deg']:.2f} deg  {'ok' if row['behaves'] else '-'}")
    return max(rated, key=lambda row: (row["behaves"], row["objects"], -row["residual_deg"]))


def export_frame(drive: Drive, index: int, split: str) -> dict:
    calibration = drive.calibration
    frame = drive.load_frame(index)
    width, height = frame.image_size

    ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    assert ok, "JPEG encoding failed"

    points = frame.front_points
    near = project(points, calibration.lidar_to_image(), frame.image_size, min_depth=0.5)
    margin = IMAGE_MARGIN_PX
    keep = ((near.depth > 0.5) & (near.pixels[:, 0] > -margin) & (near.pixels[:, 0] < width + margin)
            & (near.pixels[:, 1] > -margin) & (near.pixels[:, 1] < height + margin))
    points = points[keep]
    if len(points) > MAX_POINTS:
        points = points[np.linspace(0, len(points) - 1, MAX_POINTS).round().astype(int)]

    edge_points, edge_weights = lidar_depth_edges(frame.scan, across_rings=True)
    objects = collect_objects([frame], calibration)
    quality = frame_quality(drive, index)
    log(f"export {drive.name}:{index} ({split}): {len(points)} points, {len(edge_points)} edge points, {len(objects)} objects, "
        f"health truth {quality['truth_health']:.0%} / realistic {quality['drift_health']:.0%}, residual {quality['residual_deg']:.2f} deg")
    return {
        "drive": drive.name, "index": int(index), "split": split, "width": width, "height": height,
        "image": base64.b64encode(jpeg.tobytes()).decode("ascii"),
        "points": b64(points.astype(np.float32)),  # scan values are float32 on disk, so this is lossless
        "edge_points": b64(edge_points.astype(np.float32)),
        "edge_weights": b64(edge_weights.astype(np.float64)),
        "field": b64(quantised_field(frame.image)),
        "objects": [{
            "label": view.label, "box": [float(c) for c in view.box], "true_depth": view.true_depth,
            "distance": view.distance, "points": b64(view.own_points.astype(np.float32)),
        } for view in objects],
        "python": {key: quality[key] for key in ("truth_health", "drift_health", "residual_deg")},
    }


def build() -> None:
    started = time.time()
    drives = {name: Drive(DATA_DIR, name) for name in ("0005", "0048", "0052")}
    chosen: list[tuple[str, int, str]] = []
    for split in ("dev", "held-out"):
        required_drive, required_index = REQUIRED[split]
        second = drives[SECOND_DRIVE[split]]
        avoid = None
        if required_index in drives[required_drive].frame_indices:
            chosen.append((required_drive, required_index, split))
            avoid = required_index if second.name == required_drive else None
        log(f"choosing a {split} frame from drive {second.name}")
        chosen.append((second.name, best_frame(second, avoid)["index"], split))
        if sum(c[2] == split for c in chosen) < 2:  # the required frame was missing: take a second one
            chosen.append((second.name, best_frame(second, chosen[-1][1])["index"], split))

    calibration = drives["0005"].calibration
    data = {
        "calibration": {
            "velo_to_cam": calibration.velo_to_cam.tolist(),
            "rectification": calibration.rectification.tolist(),
            "projection": calibration.projection.tolist(),
        },
        "monitor": {"rotation_step_deg": MONITOR_STEP_DEG, "translation_step_m": MONITOR_STEP_M, "trigger_below": TRIGGER_BELOW},
        "frames": [export_frame(drives[name], index, split) for name, index, split in chosen],
    }
    payload = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    OUT.write_text(TEMPLATE.replace("__T3_DATA__", payload), encoding="utf-8")
    log(f"wrote {OUT} ({OUT.stat().st_size / 1e6:.2f} MB) in {time.time() - started:.1f} s")


def parity_reference(spec: str) -> None:
    """Python numbers for one frame and three drifts, with the float edge field and with the uint8 one the page uses."""
    name, index = spec.split(":")
    drive = Drive(DATA_DIR, name)
    calibration = drive.calibration
    frame = drive.load_frame(int(index))
    objects = collect_objects([frame], calibration)
    exact = EdgeAlignment([frame], calibration, across_rings=True)
    quantised = EdgeAlignment([frame], calibration, across_rings=True)
    quantised.frames[0].field = quantised_field(frame.image).astype(np.float64) / 255
    truth = calibration.velo_to_cam
    result = {}
    for label, drift in PARITY_DRIFTS.items():
        drifted = drift.applied_to(truth)
        row: dict = {"drift": {axis: getattr(drift, axis) for axis in ("roll", "pitch", "yaw", "tx", "ty", "tz")}}
        for field_kind, alignment in (("float", exact), ("uint8", quantised)):
            check = alignment.health_check(drifted, MONITOR_STEP_DEG, MONITOR_STEP_M)
            refined = alignment.refine_rotation(drifted)
            rotation, translation = drift_between(truth, refined)
            row[field_kind] = {"score": check.score, "health": check.fraction_of_neighbours_worse,
                               "refined": refined.tolist(), "residual_deg": rotation, "residual_m": translation}
        fused = association_and_range([frame], objects, calibration, drifted)
        row["association_pct"] = fused["association_retention_pct"]
        row["range_miss_pct"] = fused["range_miss_pct"]
        row["reproj_median_px"] = reprojection_error_by_depth([frame], calibration, drifted)["reproj_median_px"]
        result[label] = row
    print(json.dumps(result))


TEMPLATE = r"""<!doctype html>
<html lang="vi">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Lệch hiệu chuẩn camera–LiDAR</title>
<style>
:root {
  --bg: #f4f3ef; --panel: #ffffff; --line: #e2e0d9; --line-strong: #c9c6bc; --ink: #1c2024; --muted: #60656d; --soft: #efeee9;
  --accent: #c9400c; --accent-soft: #fbece4;   /* the one accent colour: only the alarm signal */
  --box: #1f9d48; --truth: #11c5e0;
}
* { box-sizing: border-box; }
html, body { margin: 0; background: var(--bg); color: var(--ink); }
body { font-family: "Segoe UI", system-ui, -apple-system, Roboto, "Helvetica Neue", Arial, sans-serif; font-size: 17px; line-height: 1.35; }
.app { max-width: 1900px; margin: 0 auto; padding: 12px 22px 8px; }
header { display: flex; align-items: center; justify-content: space-between; gap: 12px 20px; flex-wrap: wrap; margin-bottom: 10px; }
h1 { font-size: 23px; font-weight: 650; margin: 0; letter-spacing: -0.01em; }
h1 span { font-weight: 400; color: var(--muted); font-size: 16px; margin-left: 6px; }
button { font: inherit; color: inherit; background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 6px 12px; cursor: pointer; }
button:hover { border-color: var(--line-strong); }
button[aria-pressed="true"] { border-color: var(--ink); box-shadow: inset 0 0 0 1px var(--ink); }
button:disabled { opacity: .5; cursor: default; }
button:focus-visible, input:focus-visible { outline: 2px solid var(--ink); outline-offset: 2px; }
.frames { display: flex; gap: 6px; flex-wrap: wrap; }
.frames button { font-size: 15px; line-height: 1.2; text-align: left; }
.frames button small { display: block; color: var(--muted); font-size: 13px; }
.stage { display: flex; justify-content: center; }
#view { display: block; width: min(100%, calc((100vh - 455px) * 3.312)); min-width: min(100%, 560px); height: auto; border-radius: 6px; background: #ddd; }
.legend { display: flex; align-items: center; justify-content: space-between; flex-wrap: wrap; gap: 8px 18px; margin: 8px 0 10px; font-size: 15px; color: var(--muted); }
.legend .keys { display: flex; align-items: center; flex-wrap: wrap; gap: 6px 18px; }
.legend .key { display: inline-flex; align-items: center; gap: 7px; }
.ramp { width: 120px; height: 10px; border-radius: 5px; }
.swatch { width: 12px; height: 12px; border-radius: 3px; display: inline-block; }
.legend label { display: inline-flex; align-items: center; gap: 6px; color: var(--ink); cursor: pointer; }
.legend input { width: 18px; height: 18px; accent-color: var(--ink); }
#showing { color: var(--ink); font-weight: 600; }
.viewtoggle { display: inline-flex; gap: 4px; }
.viewtoggle[hidden] { display: none; }
.viewtoggle button { font-size: 14px; padding: 4px 10px; }
.grid { display: grid; grid-template-columns: 1.12fr 1fr 1.05fr; gap: 12px; }
.card { background: var(--panel); border: 1px solid var(--line); border-radius: 10px; padding: 12px 16px 14px; min-width: 0; }
.card h2 { font-size: 18px; margin: 0 0 4px; font-weight: 650; display: flex; align-items: baseline; gap: 8px; flex-wrap: wrap; }
.card h2 .tag { display: inline-grid; place-items: center; width: 24px; height: 24px; border-radius: 6px; background: var(--ink); color: #fff; font-size: 14px; }
.card h2 .sub { font-weight: 400; color: var(--muted); font-size: 16px; }
.note { margin: 0 0 10px; font-size: 14.5px; color: var(--muted); }
.note b { color: var(--ink); font-weight: 600; }
.presets { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin: 6px 0 10px; }
.presets button { font-size: 15px; text-align: left; line-height: 1.2; }
.sliders { display: grid; grid-template-columns: 1fr 1fr; gap: 6px 18px; }
.slider label { display: flex; justify-content: space-between; align-items: baseline; gap: 6px; font-size: 15px; }
.slider label em { font-style: normal; color: var(--muted); font-size: 13px; }
.slider output { font-variant-numeric: tabular-nums; font-weight: 600; min-width: 66px; text-align: right; }
.slider input { width: 100%; accent-color: var(--ink); margin: 2px 0 0; }
.verdict { display: flex; align-items: center; gap: 14px; margin: 2px 0 8px; }
.badge { font-size: 26px; font-weight: 750; letter-spacing: .02em; padding: 4px 14px; border-radius: 8px; border: 2px solid var(--ink); white-space: nowrap; }
.badge.alarm { border-color: var(--accent); color: var(--accent); background: var(--accent-soft); }
.action { font-size: 15.5px; }
.action.alarm { color: var(--accent); font-weight: 600; }
.figures { display: grid; grid-template-columns: 1fr 1fr; gap: 6px 16px; margin-bottom: 6px; }
.figure span { display: block; font-size: 13.5px; color: var(--muted); }
.figure b { font-size: 26px; font-variant-numeric: tabular-nums; font-weight: 650; }
.bar { position: relative; height: 8px; background: var(--soft); border-radius: 4px; margin: 2px 0 10px; }
.bar i { position: absolute; inset: 0 auto 0 0; background: var(--ink); border-radius: 4px; }
.bar i.alarm { background: var(--accent); }
.bar s { position: absolute; top: -4px; bottom: -4px; width: 2px; background: var(--muted); }
.bar em { position: absolute; top: 9px; font-size: 12px; color: var(--muted); font-style: normal; transform: translateX(-50%); }
.refine { display: flex; align-items: center; gap: 12px; margin-top: 12px; flex-wrap: wrap; }
.primary { background: var(--ink); color: #fff; border-color: var(--ink); font-weight: 600; padding: 8px 16px; }
.primary:hover { border-color: var(--ink); background: #33383e; }
.progress { flex: 1; min-width: 120px; height: 6px; background: var(--soft); border-radius: 3px; overflow: hidden; }
.progress i { display: block; height: 100%; width: 0; background: var(--ink); transition: width .12s linear; }
#progressText, #afterA { font-size: 14.5px; color: var(--muted); }
#afterA { margin-top: 6px; }
#afterA b { color: var(--ink); }
table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; }
th, td { padding: 6px 4px; border-bottom: 1px solid var(--line); text-align: right; font-size: 16px; }
th { font-size: 13.5px; color: var(--muted); font-weight: 600; }
th:first-child, td:first-child { text-align: left; }
td:first-child { font-size: 15px; }
td:first-child small { display: block; color: var(--muted); font-size: 12.5px; }
td b { font-weight: 650; }
.foot { margin: 8px 0 0; font-size: 13px; color: var(--muted); }
footer { display: flex; justify-content: space-between; flex-wrap: wrap; gap: 4px 16px; margin-top: 10px; font-size: 13.5px; color: var(--muted); }
footer a { color: inherit; }
@media (max-width: 1150px) { .grid { grid-template-columns: 1fr; } #view { width: 100%; } }
@media (max-width: 560px) { .sliders, .presets, .figures { grid-template-columns: 1fr; } .app { padding: 10px 16px; } }
</style>
</head>
<body>
<div class="app">
  <header>
    <h1>Lệch hiệu chuẩn camera–LiDAR: phát hiện và tự sửa <span>KITTI raw 2011_09_26</span></h1>
    <div class="frames" id="frames" role="group" aria-label="Chọn khung hình"></div>
  </header>

  <div class="stage"><canvas id="view" width="2484" height="750" aria-label="Ảnh camera trái với điểm LiDAR chiếu lên"></canvas></div>
  <div class="legend">
    <div class="keys">
      <span id="showing"></span>
      <span class="key"><span>gần</span><span class="ramp" id="ramp"></span><span>xa</span><span>(màu = độ sâu LiDAR)</span></span>
      <span class="key"><span class="swatch" style="background:var(--box)"></span>hộp 2D thật</span>
      <label><input type="checkbox" id="showTruth"> hiện điểm theo hiệu chuẩn đúng <span class="swatch" style="background:var(--truth)"></span></label>
    </div>
    <div class="viewtoggle" id="viewToggle" hidden role="group" aria-label="So sánh trước và sau">
      <button type="button" data-view="now">Trước (đang lệch)</button>
      <button type="button" data-view="after">Sau tự hiệu chuẩn</button>
    </div>
  </div>

  <div class="grid">
    <section class="card" aria-labelledby="h-controls">
      <h2 id="h-controls">Gây lệch hiệu chuẩn <span class="sub">(mô phỏng cú va vào giá gắn LiDAR)</span></h2>
      <div class="presets" id="presets"></div>
      <div class="sliders" id="sliders"></div>
    </section>

    <section class="card" aria-labelledby="h-a">
      <h2 id="h-a"><span class="tag">A</span>Bộ phát hiện <span class="sub">(không dùng ground truth)</span></h2>
      <p class="note">Chỉ dùng <b>ảnh camera + quét LiDAR của chính khung này</b>. Không dùng nhãn, không dùng hiệu chuẩn gốc KITTI — xe chạy thật cũng chạy được.</p>
      <div class="verdict"><span class="badge" id="badge">OK</span><span class="action" id="action"></span></div>
      <div class="figures">
        <div class="figure"><span>Điểm khớp cạnh LiDAR–ảnh</span><b id="score">–</b></div>
        <div class="figure"><span>Sức khỏe: lân cận khớp kém hơn</span><b id="health">–</b></div>
      </div>
      <div class="bar" aria-hidden="true"><i id="healthBar"></i><s id="tick"></s><em id="tickLabel">90 %</em></div>
      <p class="note" style="margin:16px 0 0">So 52 hiệu chuẩn lân cận (±0.5°, ±10 cm). Nếu hiện tại là đỉnh thì gần như mọi lân cận đều kém hơn; dưới 90 % là báo động.</p>
      <div class="refine">
        <button type="button" class="primary" id="refine">Tự hiệu chuẩn lại</button>
        <div class="progress" aria-hidden="true"><i id="progressBar"></i></div>
      </div>
      <div id="progressText"></div>
      <div id="afterA"></div>
    </section>

    <section class="card" aria-labelledby="h-b">
      <h2 id="h-b"><span class="tag">B</span>Chấm điểm <span class="sub">(chỉ khi có ground truth)</span></h2>
      <p class="note">So với hiệu chuẩn gốc KITTI và nhãn 3D. Xe chạy thật <b>không có</b> phần này — chỉ để chấm bộ phát hiện.</p>
      <table>
        <thead><tr><th></th><th>Hiện tại</th><th>Sau tự hiệu chuẩn</th></tr></thead>
        <tbody>
          <tr><td>Sai số chiếu lại <small>trung vị, so với hiệu chuẩn đúng</small></td><td id="bReprojNow">–</td><td id="bReprojAfter">–</td></tr>
          <tr><td>Điểm LiDAR của vật nằm trong hộp <small>ghép điểm–vật (association)</small></td><td id="bAssocNow">–</td><td id="bAssocAfter">–</td></tr>
          <tr><td>Đo khoảng cách hụt <small>late fusion: hộp camera + độ sâu LiDAR</small></td><td id="bRangeNow">–</td><td id="bRangeAfter">–</td></tr>
          <tr><td>Sai số góc quay còn lại</td><td id="bRotNow">–</td><td id="bRotAfter">–</td></tr>
          <tr><td>Sai số tịnh tiến còn lại <small>tự hiệu chuẩn chỉ sửa góc quay</small></td><td id="bTransNow">–</td><td id="bTransAfter">–</td></tr>
        </tbody>
      </table>
      <p class="foot">Hụt = trung vị độ sâu LiDAR trong 50 % giữa hộp lệch khỏi độ sâu thật quá max(1 m, 10 %). Cùng định nghĩa với t3calib/metrics.py.</p>
    </section>
  </div>

  <footer>
    <span>Phương pháp: Levinson &amp; Thrun, RSS 2013 + 2 cải tiến của nhóm (cạnh LiDAR giữa các vòng quét; trường cạnh ảnh hẹp hơn)</span>
    <a href="https://github.com/Catnip-harvest/track-4-mini-project-day-T3">https://github.com/Catnip-harvest/track-4-mini-project-day-T3</a>
  </footer>
</div>

<script id="t3-data" type="application/json">__T3_DATA__</script>

<script id="t3-core">
// Math core shared by the page and ui/check_parity.mjs. Mirrors t3calib (geometry.py, edge_alignment.py, metrics.py).
const T3 = (() => {
  "use strict";
  const ALPHABET = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  const LOOKUP = new Uint8Array(256);
  for (let i = 0; i < ALPHABET.length; i++) LOOKUP[ALPHABET.charCodeAt(i)] = i;

  function decodeBase64(text) {
    let length = text.length;
    while (length > 0 && text.charCodeAt(length - 1) === 61) length--;
    const bytes = new Uint8Array(Math.floor(length * 3 / 4));
    let out = 0, i = 0;
    for (; i + 4 <= length; i += 4) {
      const n = (LOOKUP[text.charCodeAt(i)] << 18) | (LOOKUP[text.charCodeAt(i + 1)] << 12)
        | (LOOKUP[text.charCodeAt(i + 2)] << 6) | LOOKUP[text.charCodeAt(i + 3)];
      bytes[out++] = (n >> 16) & 255; bytes[out++] = (n >> 8) & 255; bytes[out++] = n & 255;
    }
    const rest = length - i;
    if (rest >= 2) {
      const n = (LOOKUP[text.charCodeAt(i)] << 18) | (LOOKUP[text.charCodeAt(i + 1)] << 12)
        | ((rest === 3 ? LOOKUP[text.charCodeAt(i + 2)] : 0) << 6);
      bytes[out++] = (n >> 16) & 255;
      if (rest === 3) bytes[out++] = (n >> 8) & 255;
    }
    return bytes;
  }
  const float32 = (text) => new Float32Array(decodeBase64(text).buffer);
  const float64 = (text) => new Float64Array(decodeBase64(text).buffer);

  function prepareFrame(raw) {
    const quantised = decodeBase64(raw.field);
    const field = new Float64Array(quantised.length);
    for (let i = 0; i < quantised.length; i++) field[i] = quantised[i] / 255;
    return {
      drive: raw.drive, index: raw.index, split: raw.split, width: raw.width, height: raw.height,
      points: float32(raw.points), edgePoints: float32(raw.edge_points), edgeWeights: float64(raw.edge_weights), field,
      objects: raw.objects.map((o) => ({ label: o.label, box: o.box, trueDepth: o.true_depth, distance: o.distance, points: float32(o.points) })),
    };
  }

  function matMul(a, b) {
    const rows = a.length, inner = b.length, cols = b[0].length, out = [];
    for (let r = 0; r < rows; r++) {
      const row = new Array(cols);
      for (let c = 0; c < cols; c++) {
        let sum = 0;
        for (let k = 0; k < inner; k++) sum += a[r][k] * b[k][c];
        row[c] = sum;
      }
      out.push(row);
    }
    return out;
  }

  function inverse4(m) {  // Gauss-Jordan with partial pivoting (KITTI's R is not exactly orthonormal, so no transpose shortcut)
    const a = m.map((row, r) => [...row, ...[0, 1, 2, 3].map((c) => (c === r ? 1 : 0))]);
    for (let col = 0; col < 4; col++) {
      let pivot = col;
      for (let r = col + 1; r < 4; r++) if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
      [a[col], a[pivot]] = [a[pivot], a[col]];
      const p = a[col][col];
      for (let c = 0; c < 8; c++) a[col][c] /= p;
      for (let r = 0; r < 4; r++) {
        if (r === col) continue;
        const f = a[r][col];
        for (let c = 0; c < 8; c++) a[r][c] -= f * a[col][c];
      }
    }
    return a.map((row) => row.slice(4));
  }

  const RAD = Math.PI / 180, DEG = 180 / Math.PI;
  function rotationFromEuler(rollDeg, pitchDeg, yawDeg) {  // Rz @ Ry @ Rx about the LiDAR axes (x fwd, y left, z up)
    const r = rollDeg * RAD, p = pitchDeg * RAD, y = yawDeg * RAD;
    const aboutX = [[1, 0, 0], [0, Math.cos(r), -Math.sin(r)], [0, Math.sin(r), Math.cos(r)]];
    const aboutY = [[Math.cos(p), 0, Math.sin(p)], [0, 1, 0], [-Math.sin(p), 0, Math.cos(p)]];
    const aboutZ = [[Math.cos(y), -Math.sin(y), 0], [Math.sin(y), Math.cos(y), 0], [0, 0, 1]];
    return matMul(matMul(aboutZ, aboutY), aboutX);
  }
  function driftMatrix(d) {  // degrees and metres
    const R = rotationFromEuler(d.roll || 0, d.pitch || 0, d.yaw || 0);
    return [[R[0][0], R[0][1], R[0][2], d.tx || 0], [R[1][0], R[1][1], R[1][2], d.ty || 0], [R[2][0], R[2][1], R[2][2], d.tz || 0], [0, 0, 0, 1]];
  }
  const applyDrift = (veloToCam, drift) => matMul(veloToCam, driftMatrix(drift));

  function lidarToImage(calib, veloToCam) {  // P_rect_02 @ R_rect_00 @ velo_to_cam, flattened 3x4
    const m = matMul(matMul(calib.projection, calib.rectification), veloToCam || calib.velo_to_cam);
    return [...m[0], ...m[1], ...m[2]];
  }

  function project(points, M, width, height, minDepth = 1.0) {
    const n = points.length / 3;
    const u = new Float64Array(n), v = new Float64Array(n), depth = new Float64Array(n), visible = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      const x = points[3 * i], y = points[3 * i + 1], z = points[3 * i + 2];
      const d = x * M[8] + y * M[9] + z * M[10] + M[11];
      depth[i] = d;
      if (d > minDepth) {
        const pu = (x * M[0] + y * M[1] + z * M[2] + M[3]) / d, pv = (x * M[4] + y * M[5] + z * M[6] + M[7]) / d;
        u[i] = pu; v[i] = pv;
        visible[i] = pu >= 0 && pu < width && pv >= 0 && pv < height ? 1 : 0;
      }
    }
    return { u, v, depth, visible };
  }

  // Edge score: sum(w * field[v, u]) / sum(w) over LiDAR edge points visible in the image.
  function score(frame, calib, veloToCam) {
    const M = lidarToImage(calib, veloToCam);
    const p = frame.edgePoints, w = frame.edgeWeights, field = frame.field, W = frame.width, H = frame.height;
    let total = 0, weightSum = 0;
    for (let i = 0, n = w.length; i < n; i++) {
      const x = p[3 * i], y = p[3 * i + 1], z = p[3 * i + 2];
      const d = x * M[8] + y * M[9] + z * M[10] + M[11];
      if (!(d > 1.0)) continue;
      const u = (x * M[0] + y * M[1] + z * M[2] + M[3]) / d, v = (x * M[4] + y * M[5] + z * M[6] + M[7]) / d;
      if (u >= 0 && u < W && v >= 0 && v < H) {
        total += w[i] * field[Math.trunc(v) * W + Math.trunc(u)];
        weightSum += w[i];
      }
    }
    return total / Math.max(weightSum, 1e-9);
  }

  // Health: share of the 26 rotation + 26 translation neighbours that score lower than the current extrinsic.
  function healthCheck(frame, calib, veloToCam, rotationStep = 0.5, translationStep = 0.1) {
    const current = score(frame, calib, veloToCam), neighbours = [], steps = [-1, 0, 1];
    for (const a of steps) for (const b of steps) for (const c of steps) {
      if (a === 0 && b === 0 && c === 0) continue;
      neighbours.push(score(frame, calib, applyDrift(veloToCam, { roll: a * rotationStep, pitch: b * rotationStep, yaw: c * rotationStep })));
    }
    for (const a of steps) for (const b of steps) for (const c of steps) {
      if (a === 0 && b === 0 && c === 0) continue;
      neighbours.push(score(frame, calib, applyDrift(veloToCam, { tx: a * translationStep, ty: b * translationStep, tz: c * translationStep })));
    }
    const worse = neighbours.filter((s) => s < current).length;
    return { score: current, fractionWorse: worse / neighbours.length, neighbours: neighbours.length, bestGain: Math.max(...neighbours) / current - 1 };
  }

  function arange(start, stop, step) {  // numpy.arange for floats: start, start+step, then start + i*delta
    const length = Math.max(0, Math.ceil((stop - start) / step));
    const out = [];
    if (length > 0) out.push(start);
    if (length > 1) out.push(start + step);
    const delta = (start + step) - start;
    for (let i = 2; i < length; i++) out.push(start + i * delta);
    return out;
  }

  // Coordinate grid search yaw -> pitch -> roll: +-3 deg at 0.1 deg (2 rounds), then +-0.3 deg at 0.02 deg (2 rounds).
  function* refineSteps(frame, calib, veloToCam, searchDeg = 3.0, rounds = 2) {
    let estimate = veloToCam;
    const stages = [[0.1, searchDeg], [0.02, 0.3]], axes = ["yaw", "pitch", "roll"];
    const total = stages.length * rounds * axes.length;
    let sweep = 0;
    for (let s = 0; s < stages.length; s++) {
      const [step, halfWidth] = stages[s];
      const offsets = arange(-halfWidth, halfWidth + step / 2, step);
      for (let round = 0; round < rounds; round++) {
        for (const axis of axes) {
          let best = -Infinity, bestCandidate = estimate;
          for (const offset of offsets) {
            const candidate = applyDrift(estimate, { [axis]: offset });
            const value = score(frame, calib, candidate);
            if (value > best) { best = value; bestCandidate = candidate; }  // first maximum, like np.argmax
          }
          estimate = bestCandidate;
          sweep++;
          yield { sweep, total, stage: s, step, round: round + 1, axis, estimate, score: best };
        }
      }
    }
    return estimate;
  }
  function refineRotation(frame, calib, veloToCam) {
    const steps = refineSteps(frame, calib, veloToCam);
    for (;;) { const next = steps.next(); if (next.done) return next.value; }
  }

  function driftBetween(reference, estimate) {  // remaining (rotation deg, translation m): inv(reference) @ estimate
    const rel = matMul(inverse4(reference), estimate);
    const cosine = Math.min(1, Math.max(-1, (rel[0][0] + rel[1][1] + rel[2][2] - 1) / 2));
    return { rotationDeg: Math.acos(cosine) * DEG, translationM: Math.sqrt(rel[0][3] ** 2 + rel[1][3] ** 2 + rel[2][3] ** 2) };
  }

  function median(values) {
    const sorted = Float64Array.from(values).sort(), n = sorted.length, half = n >> 1;
    if (!n) return NaN;
    return n % 2 ? sorted[half] : (sorted[half - 1] + sorted[half]) / 2;
  }
  const insideBox = (u, v, box) => u >= box[0] && u <= box[2] && v >= box[1] && v <= box[3];
  function shrinkBox(box, keep) {
    const du = (box[2] - box[0]) * (1 - keep) / 2, dv = (box[3] - box[1]) * (1 - keep) / 2;
    return [box[0] + du, box[1] + dv, box[2] - du, box[3] - dv];
  }

  function reprojectionMedian(frame, calib, veloToCam) {
    const W = frame.width, H = frame.height;
    const truth = project(frame.points, lidarToImage(calib), W, H), moved = project(frame.points, lidarToImage(calib, veloToCam), W, H);
    const shifts = [];
    for (let i = 0; i < truth.visible.length; i++) {
      if (truth.visible[i] && moved.visible[i]) {
        const du = moved.u[i] - truth.u[i], dv = moved.v[i] - truth.v[i];
        shifts.push(Math.sqrt(du * du + dv * dv));
      }
    }
    return median(shifts);
  }

  function associationAndRange(frame, calib, veloToCam, centreKeep = 0.5) {
    const M = lidarToImage(calib, veloToCam), W = frame.width, H = frame.height;
    const all = project(frame.points, M, W, H);
    const perObject = frame.objects.map((obj) => {
      const own = project(obj.points, M, W, H);
      let inside = 0;
      for (let i = 0; i < own.visible.length; i++) if (own.visible[i] && insideBox(own.u[i], own.v[i], obj.box)) inside++;
      const centre = shrinkBox(obj.box, centreKeep), depths = [];
      for (let i = 0; i < all.visible.length; i++) if (all.visible[i] && insideBox(all.u[i], all.v[i], centre)) depths.push(all.depth[i]);
      const measured = depths.length ? median(depths) : NaN;
      const miss = !depths.length || Math.abs(measured - obj.trueDepth) > Math.max(1.0, 0.1 * obj.trueDepth);
      return { label: obj.label, retention: inside / own.visible.length, miss, measured, trueDepth: obj.trueDepth };
    });
    const n = perObject.length;
    return {
      perObject,
      associationPct: n ? 100 * perObject.reduce((s, o) => s + o.retention, 0) / n : NaN,
      rangeMissPct: n ? 100 * perObject.filter((o) => o.miss).length / n : NaN,
      misses: perObject.filter((o) => o.miss).length, objects: n,
    };
  }

  return { decodeBase64, prepareFrame, matMul, inverse4, rotationFromEuler, driftMatrix, applyDrift, lidarToImage, project, score,
    healthCheck, arange, refineSteps, refineRotation, driftBetween, median, shrinkBox, reprojectionMedian, associationAndRange };
})();
</script>

<script>
(() => {
  "use strict";
  const DATA = JSON.parse(document.getElementById("t3-data").textContent);
  const CAL = DATA.calibration, MON = DATA.monitor, TRUE_T = CAL.velo_to_cam;
  const SCALE = 2, BINS = 48, NEAR_M = 3, FAR_M = 60;
  const LABELS = { Car: "xe con", Van: "xe van", Truck: "xe tải", Pedestrian: "người đi bộ", Person_sitting: "người ngồi",
    Cyclist: "xe đạp", Tram: "tàu điện", Misc: "vật khác" };
  const AXES = [
    { key: "roll", hint: "quanh trục x (tiến)", min: -3, max: 3, step: 0.05, unit: "°", toMetric: 1 },
    { key: "tx", hint: "dịch về trước", min: -20, max: 20, step: 1, unit: " cm", toMetric: 0.01 },
    { key: "pitch", hint: "quanh trục y (trái)", min: -3, max: 3, step: 0.05, unit: "°", toMetric: 1 },
    { key: "ty", hint: "dịch sang trái", min: -20, max: 20, step: 1, unit: " cm", toMetric: 0.01 },
    { key: "yaw", hint: "quanh trục z (lên)", min: -3, max: 3, step: 0.05, unit: "°", toMetric: 1 },
    { key: "tz", hint: "dịch lên trên", min: -20, max: 20, step: 1, unit: " cm", toMetric: 0.01 },
  ];
  const PRESETS = [
    { label: "Hiệu chuẩn đúng (KITTI)", drift: {} },
    { label: "Va nhẹ (yaw 0.5°)", drift: { yaw: 0.5 } },
    { label: "Va thực tế (roll 0.3°, pitch −0.8°, yaw 1.2°, tx 5 cm)", drift: { roll: 0.3, pitch: -0.8, yaw: 1.2, tx: 0.05 } },
    { label: "Va mạnh (yaw 2.5°)", drift: { yaw: 2.5 } },
  ];
  const STAGE_NAMES = ["thô (bước 0.1°, ±3°)", "mịn (bước 0.02°, ±0.3°)"];

  const $ = (id) => document.getElementById(id);
  const canvas = $("view"), ctx = canvas.getContext("2d");
  const state = { frame: 0, drift: {}, showTruth: false, view: "now", live: null, refined: null, refining: false, token: 0 };
  const prepared = new Map(), images = new Map();

  function turbo(x) {
    const r = 0.13572138 + x * (4.6153926 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))));
    const g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))));
    const b = 0.1066733 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))));
    const c = (t) => Math.round(255 * Math.min(1, Math.max(0, t)));
    return `rgb(${c(r)},${c(g)},${c(b)})`;
  }
  const COLOURS = Array.from({ length: BINS }, (_, b) => turbo(0.1 + 0.82 * b / (BINS - 1)));  // bin 0 = far (blue) ... near (red)
  $("ramp").style.background = `linear-gradient(90deg, ${[...COLOURS].reverse().join(",")})`;
  const LOG_NEAR = Math.log(NEAR_M), LOG_SPAN = Math.log(FAR_M) - Math.log(NEAR_M);
  const depthBin = (d) => Math.round((1 - Math.min(1, Math.max(0, (Math.log(d) - LOG_NEAR) / LOG_SPAN))) * (BINS - 1));

  const frameData = (i) => { if (!prepared.has(i)) prepared.set(i, T3.prepareFrame(DATA.frames[i])); return prepared.get(i); };
  function frameImage(i) {
    if (!images.has(i)) {
      const img = new Image();
      img.onload = () => schedule();
      img.src = "data:image/jpeg;base64," + DATA.frames[i].image;
      images.set(i, img);
    }
    return images.get(i);
  }

  const signed = (x, digits) => (x < -1e-9 ? "−" : "+") + Math.abs(x).toFixed(digits);
  const pct = (x) => (Number.isFinite(x) ? `${x.toFixed(0)} %` : "–");
  const currentExtrinsic = () => T3.applyDrift(TRUE_T, state.drift);
  function driftText(d) {
    const parts = ["roll", "pitch", "yaw"].filter((k) => Math.abs(d[k] || 0) > 1e-9).map((k) => `${k} ${signed(d[k], 2)}°`)
      .concat(["tx", "ty", "tz"].filter((k) => Math.abs(d[k] || 0) > 1e-9).map((k) => `${k} ${signed(100 * d[k], 0)} cm`));
    return parts.length ? parts.join(", ") : "không lệch";
  }
  function displayed() {
    if (state.view === "after" && (state.refined || state.live)) return { T: state.refined ? state.refined.estimate : state.live, label: state.refined ? "Đang hiển thị: sau tự hiệu chuẩn lại" : "Đang tự hiệu chuẩn lại…" };
    return { T: currentExtrinsic(), label: `Đang hiển thị: hiệu chuẩn hiện tại (${driftText(state.drift)})` };
  }

  // ---------- canvas ----------
  function draw() {
    const f = frameData(state.frame), img = frameImage(state.frame);
    const W = f.width, H = f.height;
    if (canvas.width !== W * SCALE || canvas.height !== H * SCALE) { canvas.width = W * SCALE; canvas.height = H * SCALE; }
    ctx.fillStyle = "#d8d6cf"; ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (img.complete && img.naturalWidth) ctx.drawImage(img, 0, 0, canvas.width, canvas.height);

    const shown = displayed();
    $("showing").textContent = shown.label;
    const proj = T3.project(f.points, T3.lidarToImage(CAL, shown.T), W, H);
    const bins = Array.from({ length: BINS }, () => []);
    for (let i = 0; i < proj.visible.length; i++) if (proj.visible[i]) bins[depthBin(proj.depth[i])].push(i);
    const r = 1.6;
    for (let b = 0; b < BINS; b++) {  // far first, near on top
      if (!bins[b].length) continue;
      ctx.fillStyle = COLOURS[b];
      ctx.beginPath();
      for (const i of bins[b]) ctx.rect(proj.u[i] * SCALE - r, proj.v[i] * SCALE - r, 2 * r, 2 * r);
      ctx.fill();
    }
    if (state.showTruth) {
      const truth = T3.project(f.points, T3.lidarToImage(CAL), W, H);
      ctx.fillStyle = "rgba(17,197,224,0.9)";
      ctx.beginPath();
      for (let i = 0; i < truth.visible.length; i++) if (truth.visible[i]) ctx.rect(truth.u[i] * SCALE - 1.2, truth.v[i] * SCALE - 1.2, 2.4, 2.4);
      ctx.fill();
    }
    ctx.lineWidth = 2.5 * SCALE / 2;
    ctx.font = `600 ${11 * SCALE}px "Segoe UI", system-ui, sans-serif`;
    ctx.textBaseline = "bottom";
    for (const obj of f.objects) {
      const [u0, v0, u1, v1] = obj.box.map((x) => x * SCALE);
      ctx.strokeStyle = "#1f9d48";
      ctx.strokeRect(u0, v0, u1 - u0, v1 - v0);
      const text = `${LABELS[obj.label] || obj.label} · ${obj.trueDepth.toFixed(0)} m`;
      const tw = ctx.measureText(text).width + 10, th = 15 * SCALE;
      const ty = v0 > th ? v0 : v1 + th;
      ctx.fillStyle = "#1f9d48"; ctx.fillRect(u0 - 1, ty - th, tw, th);
      ctx.fillStyle = "#fff"; ctx.fillText(text, u0 + 4, ty - 3);
    }
  }

  // ---------- panels ----------
  function setVerdict(health) {
    const alarm = health.fractionWorse < MON.trigger_below;
    $("badge").textContent = alarm ? "BÁO ĐỘNG" : "OK";
    $("badge").classList.toggle("alarm", alarm);
    $("action").textContent = alarm ? "Giảm trọng số ghép camera–LiDAR, gọi hiệu chuẩn lại" : "Hiệu chuẩn hiện tại là đỉnh khớp cạnh — giữ nguyên";
    $("action").classList.toggle("alarm", alarm);
    $("score").textContent = health.score.toFixed(4);
    $("health").textContent = pct(100 * health.fractionWorse);
    $("healthBar").style.width = `${100 * health.fractionWorse}%`;
    $("healthBar").classList.toggle("alarm", alarm);
  }
  function fillB(suffix, f, T) {
    const fused = T3.associationAndRange(f, CAL, T), err = T3.driftBetween(TRUE_T, T);
    $("bReproj" + suffix).innerHTML = `<b>${T3.reprojectionMedian(f, CAL, T).toFixed(1)}</b> px`;
    $("bAssoc" + suffix).innerHTML = `<b>${pct(fused.associationPct)}</b>`;
    $("bRange" + suffix).innerHTML = `<b>${pct(fused.rangeMissPct)}</b> (${fused.misses}/${fused.objects} vật)`;
    $("bRot" + suffix).innerHTML = `<b>${err.rotationDeg.toFixed(2)}</b>°`;
    $("bTrans" + suffix).innerHTML = `<b>${(100 * err.translationM).toFixed(1)}</b> cm`;
  }
  function clearAfter() {
    for (const k of ["bReproj", "bAssoc", "bRange", "bRot", "bTrans"]) $(k + "After").textContent = "–";
    $("afterA").textContent = "";
    $("progressText").textContent = "";
    $("progressBar").style.width = "0";
  }

  function updatePanels() {
    const f = frameData(state.frame), T = currentExtrinsic();
    setVerdict(T3.healthCheck(f, CAL, T, MON.rotation_step_deg, MON.translation_step_m));
    fillB("Now", f, T);
    $("viewToggle").hidden = !state.refined;
    for (const b of $("viewToggle").querySelectorAll("button")) b.setAttribute("aria-pressed", String(b.dataset.view === state.view));
  }

  let pending = false, panelsDirty = true;
  function schedule(panels = false) {
    panelsDirty = panelsDirty || panels;
    if (pending) return;
    pending = true;
    requestAnimationFrame(() => {
      pending = false;
      if (panelsDirty) { panelsDirty = false; updatePanels(); }
      draw();
    });
  }

  function invalidateRefinement() {
    state.token++;
    state.refining = false; state.refined = null; state.live = null; state.view = "now";
    $("refine").disabled = false;
    clearAfter();
  }

  // ---------- refinement (no ground truth used) ----------
  const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
  async function runRefinement() {
    if (state.refining) return;
    invalidateRefinement();
    const token = state.token, f = frameData(state.frame), start = currentExtrinsic();
    state.refining = true; state.view = "after";
    $("refine").disabled = true;
    const started = performance.now();
    const steps = T3.refineSteps(f, CAL, start);
    for (;;) {
      const next = steps.next();
      if (token !== state.token) return;
      if (next.done) {
        const estimate = next.value, health = T3.healthCheck(f, CAL, estimate, MON.rotation_step_deg, MON.translation_step_m);
        state.refined = { estimate, health }; state.live = null; state.refining = false;
        $("refine").disabled = false;
        const ok = health.fractionWorse >= MON.trigger_below;
        $("progressText").textContent = `Xong: 552 lần chấm điểm, ${((performance.now() - started) / 1000).toFixed(1)} s (có làm chậm để dễ xem).`;
        $("afterA").innerHTML = `Sau tự hiệu chuẩn: điểm <b>${health.score.toFixed(4)}</b>, sức khỏe <b>${pct(100 * health.fractionWorse)}</b> → <b>${ok ? "OK" : "vẫn BÁO ĐỘNG"}</b>. Chỉ sửa góc quay.`;
        fillB("After", f, estimate);
        schedule(true);
        return;
      }
      const s = next.value;
      state.live = s.estimate;
      $("progressBar").style.width = `${(100 * s.sweep) / s.total}%`;
      $("progressText").textContent = `Dò lưới ${STAGE_NAMES[s.stage]}, vòng ${s.round}/2, trục ${s.axis} — điểm ${s.score.toFixed(4)}`;
      draw();
      await sleep(130);
    }
  }

  // ---------- controls ----------
  const sliderInputs = {};
  function setDrift(drift) {
    state.drift = { ...drift };
    for (const a of AXES) {
      const value = (state.drift[a.key] || 0) / a.toMetric;
      sliderInputs[a.key].value = String(+value.toFixed(4));
      sliderInputs[a.key].output.textContent = signed(value, a.unit === "°" ? 2 : 0) + a.unit;
    }
    for (const [i, b] of [...$("presets").children].entries()) b.setAttribute("aria-pressed", String(sameDrift(PRESETS[i].drift, state.drift)));
    invalidateRefinement();
    schedule(true);
  }
  const sameDrift = (a, b) => AXES.every((x) => Math.abs((a[x.key] || 0) - (b[x.key] || 0)) < 1e-9);

  for (const a of AXES) {
    const wrap = document.createElement("div");
    wrap.className = "slider";
    wrap.innerHTML = `<label for="s-${a.key}"><span>${a.key} <em>${a.hint}</em></span><output></output></label>
      <input type="range" id="s-${a.key}" min="${a.min}" max="${a.max}" step="${a.step}" value="0">`;
    const input = wrap.querySelector("input");
    input.output = wrap.querySelector("output");
    input.addEventListener("input", () => {
      const drift = { ...state.drift, [a.key]: parseFloat(input.value) * a.toMetric };
      setDrift(drift);
    });
    sliderInputs[a.key] = input;
    $("sliders").appendChild(wrap);
  }
  PRESETS.forEach((p) => {
    const b = document.createElement("button");
    b.type = "button"; b.textContent = p.label;
    b.addEventListener("click", () => setDrift(p.drift));
    $("presets").appendChild(b);
  });
  DATA.frames.forEach((f, i) => {
    const b = document.createElement("button");
    b.type = "button";
    b.innerHTML = `drive ${f.drive} · khung ${f.index}<small>${f.split === "dev" ? "dev (dùng khi chỉnh tham số)" : "held-out (chưa từng dùng)"} · ${f.objects.length} vật</small>`;
    b.addEventListener("click", () => { state.frame = i; markFrame(); invalidateRefinement(); schedule(true); });
    $("frames").appendChild(b);
  });
  const markFrame = () => [...$("frames").children].forEach((b, i) => b.setAttribute("aria-pressed", String(i === state.frame)));
  $("showTruth").addEventListener("change", (e) => { state.showTruth = e.target.checked; schedule(); });
  $("refine").addEventListener("click", runRefinement);
  for (const b of $("viewToggle").querySelectorAll("button")) b.addEventListener("click", () => { state.view = b.dataset.view; schedule(true); });
  $("tick").style.left = `${100 * MON.trigger_below}%`;
  $("tickLabel").style.left = `${100 * MON.trigger_below}%`;
  $("tickLabel").textContent = `${(100 * MON.trigger_below).toFixed(0)} %`;

  markFrame();
  setDrift({});
})();
</script>
</body>
</html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--parity-reference", metavar="DRIVE:INDEX", help="print Python reference values as JSON and exit")
    args = parser.parse_args()
    if args.parity_reference:
        parity_reference(args.parity_reference)
    else:
        build()


if __name__ == "__main__":
    main()
