# T3 — Camera–LiDAR calibration drift on an ADAS car (KITTI raw)

VinUni AI20K, Track 4, Day 4 "Sensor Reality Sprint", topic T3 "Calibration drift impact".

This is a small, self-contained benchmark. It asks one question: when the bracket that holds the
LiDAR and the front camera gets knocked a little, how badly does camera–LiDAR fusion break, and can
we notice it without a calibration target?

- **Platform:** ADAS car (KITTI recording car).
- **Sensors:** front left colour camera (cam 2) + Velodyne HDL-64E LiDAR.
- **Ground truth:** the KITTI calibration of 2011_09_26. We perturb it on purpose and measure the damage.

It does three things:

1. **Drift sweep.** Tilt the LiDAR-to-camera transform (the *extrinsic*) by 0–3° about each axis and
   shift it by 0–20 cm along each axis, then measure how far LiDAR points move in the image, how many
   of an object's own LiDAR points still fall inside its 2D box, and how often a simple late-fusion
   range estimate goes wrong.
2. **Health monitor.** A targetless check after Levinson & Thrun (RSS 2013): LiDAR depth edges should
   land on image edges. If the current extrinsic is not a local best, the monitor asks for re-calibration.
3. **Rotation refinement.** Climb the same edge score from a realistic drift and measure what is recovered.

The written report (Vietnamese, one page plus figures) is [REPORT.md](REPORT.md).

## Requirements

- Python 3.13 (tested with 3.13.14), CPU only. No GPU, no ROS, no training, no pretrained weights.
- Packages actually imported by the code: `numpy`, `pandas`, `matplotlib`, `pillow`, `opencv-python`.
  `scipy` is listed in the course template but the current code does not import it.

```bash
pip install numpy pandas matplotlib pillow opencv-python
```

Versions used for the committed results: numpy 2.2.6, pandas 3.0.5, matplotlib 3.11.1,
Pillow 12.3.0, OpenCV 5.0.0.

## 1. Get the data

```bash
python get_data.py                  # drives 0048 and 0005 into ./data
python get_data.py --drives 0048    # only the small drive
python get_data.py --drives 0052    # held-out drive, used only by validate_holdout.py
```

| What | Frames | Download |
|---|---|---|
| `2011_09_26_drive_0048_sync` | 22 | 84 MB |
| `2011_09_26_drive_0005_sync` | 154 | 646 MB |
| `2011_09_26_drive_0052_sync` (held-out) | 78 | 285 MB |
| `2011_09_26_calib` + both `_tracklets` zips | — | < 1 MB |

- Source: the public KITTI raw S3 bucket (`s3.eu-central-1.amazonaws.com/avg-kitti/raw_data`).
  These direct links need no login.
- Why the script is not a plain `wget`: one connection to that bucket runs at only ~50–80 KB/s
  from Vietnam. The script asks for many 4 MB byte ranges at once (24 parallel HTTP range requests
  by default, `--connections N` to change) and writes them into place.
- Only what the benchmark reads is extracted: `image_02/`, `velodyne_points/`,
  `tracklet_labels.xml` and the `calib_*.txt` files. The zips stay in `data/` with a `.done`
  marker, so a re-run skips the download. Expect about 1.2 GB on disk including the zips for 0048 + 0005; drive 0052 adds about 0.5 GB.
- **License:** KITTI is CC BY-NC-SA 3.0 — non-commercial use only. `data/` is not part of this repo.

## 2. Run the benchmark

```bash
python run_benchmark.py                                 # full run: both drives
python run_benchmark.py --drives 0048 --out results_quick   # quick check on the 22-frame drive
```

The full run takes about 1–2 minutes on a laptop CPU (the committed run finished in 74.8 s).
It uses every 2nd frame: 11 frames of 0048 + 77 frames of 0005 = 88 frames, 241 labelled object views.

Without `--out`, the quick mode **overwrites** `results/` with numbers from 11 frames only.

Command-line options:

| Flag | Default | Meaning |
|---|---|---|
| `--drives` | `0048 0005` | which drives to load |
| `--frame-step` | `2` | use every n-th frame |
| `--monitor-frames` | `24` | frames pooled by the edge-alignment monitor |
| `--flag-below` | `0.9` | trigger re-calibration when fewer than this share of the 52 neighbours score lower |
| `--data-dir`, `--out` | `data/`, `results/` | input and output folders |

### Held-out check

The edge-score settings were tuned on 0048 and 0005. `validate_holdout.py` re-runs the monitor and the
rotation refinement with the same fixed settings (copied from `run_benchmark.py`, nothing tuned) on
any drive list: false alarm at the true calibration, single-axis detection, and 20 random drifts
(seed 7; roll/pitch/yaw within ±1.5°, tx/ty/tz within ±5 cm).

```bash
python validate_holdout.py --drives 0052 --label holdout        # drive never used for tuning (~2.3 min)
python validate_holdout.py --drives 0048 0005 --label dev       # same checks on the tuning drives (~2.7 min)
```

Result in short: on 0052 the improved variant is better in median (0.44° vs 0.55° left) but less
reliable (90th percentile 1.75° vs 0.70°, three refinements stuck at a roll–pitch false peak).
See REPORT.md, section 3.

## 3. Outputs

All written to `results/` by one run.

| File | What it is |
|---|---|
| `run_log.txt` | full console log of the run: data counts, every table below, timings |
| `rotation_sweep.csv` | roll / pitch / yaw 0–3°: reprojection error by depth band, association retention (all / near / far), objects lost, late-fusion range miss |
| `translation_sweep.csv` | tx / ty / tz 0–20 cm: same metrics |
| `monitor_sweep.csv` | health monitor on single-axis drifts, both edge variants (`paper`, `improved`): score, share of neighbours worse, best neighbour gain, triggered or not |
| `refinement.json` | realistic drift (roll 0.3°, pitch −0.8°, yaw 1.2°, tx 5 cm) before and after rotation refinement with each variant, plus where each score peaks relative to KITTI |
| `figures/01_overlay.png` | LiDAR points on the camera image: KITTI calibration, realistic drift, after refinement; zoom on a near car and a far van |
| `figures/02_rotation_sweep.png` | rotation drift vs reprojection error, association retention, range miss |
| `figures/03_translation_depth.png` | sideways shift ty: pixel error by depth vs yaw 1°; association of near objects for tx / ty / tz |
| `figures/04_monitor.png` | the edge score around KITTI for yaw and pitch; health check vs rotation and translation drift |

Written to `results/holdout/` by `validate_holdout.py`, one set per `--label` (`dev`, `holdout`):

| File | What it is |
|---|---|
| `<label>_summary.json` | drives and frame counts, fixed settings, single-axis detection sweep, and per variant: false alarm at truth, detection rates, residual rotation (median, p90, max, per axis), score peak vs KITTI |
| `<label>_random_drifts.csv` | one row per random drift: injected roll/pitch/yaw/tx/ty/tz, health and trigger, residual rotation (total and per axis) for each variant |
| `<label>_log.txt` | full console log of the run, ending in a paper-vs-improved summary table |

Metric definitions (all against the KITTI calibration):

- **Reprojection error** — pixel distance between where a LiDAR point lands with the true and the
  drifted extrinsic; median per depth band (0–10, 10–20, 20–40, 40–80 m).
- **Association retention** — share of an object's own LiDAR points (inside its tracklet 3D box)
  that still fall inside its true 2D box. The 2D box is the projected tracklet box, i.e. a perfect
  2D detector. An object is "lost" below 50 %.
- **Range miss** — late fusion "camera box + median LiDAR depth in the central half of the box";
  a miss is an error above max(1 m, 10 %) or no points. Note this recipe already misses 3.3 % of
  objects (all of them far) with the perfect calibration.

## Repo layout

```
get_data.py            parallel-range downloader + selective unzip for KITTI raw
run_benchmark.py       the whole pipeline: sweeps, monitor, refinement, tables, figures
validate_holdout.py    held-out check of monitor + refinement with fixed settings
t3calib/
  kitti.py             calibration, Velodyne scans, cam-2 images, tracklet boxes
  geometry.py          Drift (roll/pitch/yaw/tx/ty/tz in LiDAR axes), projection, box helpers
  metrics.py           reprojection error by depth, association retention, late-fusion range miss
  edge_alignment.py    targetless edge score, health check, rotation refinement
results/               outputs of the committed run (see above)
results/holdout/       outputs of validate_holdout.py (dev and holdout)
data/                  KITTI files, created by get_data.py (not committed)
REPORT.md              the one-page report (Vietnamese)
```

## Changing parameters

The sweep and monitor settings are constants at the top of `run_benchmark.py`:

| Constant | Default | Controls |
|---|---|---|
| `ROTATION_DEG` | 0, 0.1, 0.25, 0.5, 0.75, 1, 1.5, 2, 3 | rotation drifts in the sweep (degrees) |
| `TRANSLATION_M` | 0, 0.02, 0.05, 0.10, 0.20 | translation drifts in the sweep (metres) |
| `MONITOR_ROTATION_DEG` | 0 … 2.0 | rotation drifts fed to the monitor |
| `MONITOR_TRANSLATION_M` | 0 … 0.20 | translation drifts fed to the monitor |
| `MONITOR_STEP_DEG`, `MONITOR_STEP_M` | 0.5°, 0.10 m | neighbour spacing of the health check (3×3×3 rotation grid + 3×3×3 translation grid = 52 neighbours) |
| `REALISTIC_DRIFT` | roll 0.3°, pitch −0.8°, yaw 1.2°, tx 5 cm | the "bracket knock" case used for refinement and figure 01 |

The edge-score settings are the keyword defaults in `t3calib/edge_alignment.py`
(`image_edge_field`: `decay=0.8`, `spread_px=15`; `lidar_depth_edges`: `min_jump_m=0.5`,
`min_height_m=-1.3`). They were tuned on the same KITTI frames that are evaluated, so treat any
change as a new experiment, not a fix, and re-check it on drive 0052 with `validate_holdout.py`.

## References

See the "Nguồn" section of [REPORT.md](REPORT.md). Main method: J. Levinson and S. Thrun,
"Automatic Online Calibration of Cameras and Lasers", RSS 2013. Data: A. Geiger et al.,
"Vision meets Robotics: The KITTI Dataset", IJRR 2013.
