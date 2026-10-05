# SoftCorr setup and run (Windows, native)

SoftCorr is the "estimate and correct the drifted extrinsic" step of T3. It is a public,
MIT-licensed, training-free LiDAR-camera calibrator. It uses Microsoft MoGe-2 for
single-image depth. This page gives the exact commands used on this machine. Results go
to `results/softcorr/`.

## Hardware and software used

| item | value |
|---|---|
| OS | Windows 11 Pro 10.0.26200, Git Bash; no WSL |
| GPU | NVIDIA GeForce RTX 4060 Laptop, 8 GB, driver 596.49 |
| Python | 3.13.14 (the only interpreter on the machine) |
| torch | 2.11.0+cu128 (CUDA 12.8 wheel) |
| SoftCorr | https://github.com/yuhyun00/SoftCorr @ `331c6191195718b36cb795285c6dcb7b1610984a` (2026-09-14) |
| MoGe | https://github.com/microsoft/MoGe @ `0286b495230a074aadf1c76cc5c679e943e5d1c6` (v2.0.0, pinned by SoftCorr), checkpoint `Ruicheng/moge-2-vitl-normal` |

## Install

Run from the project root:

```bash
git clone https://github.com/yuhyun00/SoftCorr third_party/SoftCorr
git -C third_party/SoftCorr checkout 331c6191195718b36cb795285c6dcb7b1610984a

python -m venv .venv-softcorr
.venv-softcorr/Scripts/python.exe -m pip install --upgrade pip
.venv-softcorr/Scripts/python.exe -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
.venv-softcorr/Scripts/python.exe -m pip install -r third_party/SoftCorr/requirements.txt
.venv-softcorr/Scripts/python.exe -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

Install torch from the CUDA index **first**. If you skip that step, `requirements.txt`
(`torch>=2.3`) installs the CPU-only wheel from PyPI. The torch wheel is 2.75 GB.

`requirements.txt` installs MoGe from git at the pinned commit. On this machine, MoGe was
installed from a local clone of that same commit, to avoid downloading the repository
twice. The result is the same package. On first use, MoGe downloads the MoGe-2 checkpoint
(about 1.3 GB) into `~/.cache/huggingface/`. That folder is outside the project.

`.venv-softcorr/`, `third_party/` and `.cache/` (the per-frame MoGe-2 depth cache) are
listed in `.gitignore`.

## Run

SoftCorr's own example, exactly as its README gives it (run from `third_party/SoftCorr`):

```bash
cd third_party/SoftCorr
../../.venv-softcorr/Scripts/python.exe calibrate.py --image example_dataset/kitti_00_002887.png \
    --cloud example_dataset/kitti_00_002887.pcd --intrinsics example_dataset/K.txt \
    --init example_dataset/T_init.txt --gt example_dataset/T_gt.txt
```

The T3 drift harness (the same 20 drifts as `validate_holdout.py`), run from the project root:

```bash
.venv-softcorr/Scripts/python.exe softcorr_eval.py --drives 0052 --label holdout
.venv-softcorr/Scripts/python.exe softcorr_eval.py --drives 0048 0005 --label dev
```

## Measured

SoftCorr runs natively on Windows: no WSL, no code changes, and no Application Control block. pandas 3.0.6, opencv 5.0 and MoGe all import.

| run | result | time | peak VRAM |
|---|---|---|---|
| README example (KITTI odometry 00/002887, nominal start) | 0.084 deg / 19.3 mm from `T_gt.txt` (README expects about 0.08 deg / 20 mm) | 698 s wall clock, of which about 10 min was the first MoGe-2 checkpoint download; about 75 s of GPU time | 2.95 GB on the whole card (nvidia-smi, `results/softcorr/example_nvidia_smi.csv`) |
| Harness, drive 0052, 1 frame (0052/39), 5 of 20 drifts | 0.243 deg / 1.85 cm residual (median) | 87 s per drift (MoGe-2 inference 5.8 s per frame, cached after that) | 2.22 GB for MoGe-2, 0.73 GB for the optimisation (`torch.cuda.max_memory_allocated`) |

Conversion check: SoftCorr's own projection and `t3calib.geometry.project` agree to 0.0002 px on
16 605 points. The tolerance is 0.5 px.

Started at the KITTI calibration, SoftCorr moves to 0.241 deg / 1.8 cm away from it. From every
drifted start, it lands at the same point. That point is where this frame's SoftCorr score
peaks, not an error in the start. Full numbers are in `results/softcorr/holdout_summary.json`.

To run all 20 drifts with 3 frames each, budget about 90 minutes on this GPU (about 87 s
per frame for each drift). Use `--max-drifts`, `--frames-per-drive` and `--time-budget-min`
to trim the run.
