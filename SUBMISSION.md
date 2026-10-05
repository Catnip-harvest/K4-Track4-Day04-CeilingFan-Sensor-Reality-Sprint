# Hồ sơ nộp — T3: Lệch hiệu chuẩn camera–LiDAR trên xe ADAS

Nhóm **Ceiling Fan** · VinUni AI20K, Track 4, Day 4 "Sensor Reality Sprint", đề T3 "Calibration drift impact".

Repo: https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint · Thành viên: [TEAMMATES.md](TEAMMATES.md) · Báo cáo cá nhân: [reports/](reports/) · Việc còn lại: [TEAM_TASKS.md](TEAM_TASKS.md)

Đọc theo thứ tự: [REPORT_1PAGE.md](REPORT_1PAGE.md) (1 trang) → [slides](https://claude.ai/artifact/MohVqPy4yyGuHUUeV81r94) → [REPORT.md](REPORT.md) (bản đầy đủ) → [README.md](README.md) (cách chạy).

Slides là link riêng tư: người chấm chỉ mở được sau khi nhóm bấm chia sẻ.

## Bước 1 · Chốt bài toán

| Nền tảng / tính năng / sensor | Failure case | Claim ban đầu | Metric và đơn vị | Baseline và điều kiện lỗi | Phân công 5 thành viên |
|---|---|---|---|---|---|
| Xe ADAS (xe ghi KITTI) / late fusion camera–LiDAR: box 2D lấy khoảng cách từ điểm LiDAR / camera màu trước cam 2 + LiDAR Velodyne HDL-64E | Extrinsic camera–LiDAR lệch sau rung lắc hay va vào giá đỡ; không cảm biến nào tự báo. Ca phân tích: đỉnh giả roll–pitch trên 0052 (bản improved, 3/20 lần chỉnh kẹt ở 1.74–2.19°) | Lệch xoay khoảng 1° đã làm late fusion đo sai khoảng cách; monitor căn cạnh không cần bảng (Levinson & Thrun 2013) phát hiện được lệch từ 0.25° / 10 cm | Lỗi chiếu (px); giữ điểm trong box 2D thật (%); range miss (%); health (% trong 52 hàng xóm có điểm thấp hơn); tỉ lệ phát hiện (%); sai số xoay (°) và dịch (cm) còn lại sau khi chỉnh | Baseline: hiệu chuẩn KITTI 2011_09_26 (range miss nền 3.3 %). Lỗi: lệch một trục (xoay 0–3°, dịch 0–20 cm) trên dev 0048 + 0005; 20 lệch ngẫu nhiên (xoay ±1.5°, dịch ±5 cm, seed 7) trên hold-out 0052 | **Hoàng Quốc Việt:** benchmark lõi, monitor, kiểm định hold-out, báo cáo (kèm kiểm tra phát hiện vật thể và SoftCorr).<br>**Nguyễn Minh Dương:** requirements.txt + script một lệnh ([#3](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/3)); kiểm tra độ nghiêng mặt đường cho roll/pitch ([#4](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/4)).<br>**Hoàng Trung Khải:** unit test pytest cho t3calib ([#5](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/5)); kiểm định thêm drive 0001 ([#6](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/6)).<br>**Nguyễn Hoàng Sơn:** association theo loại vật ([#7](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/7)); chọn ngưỡng kích hoạt hiệu chuẩn lại ([#8](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/8)).<br>**Dương Dương:** bộ hỏi–đáp ([#9](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/9)); kịch bản nói 3–5 phút cho 5 người ([#10](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/10)). |

## Bước 2 · Ghi chép nguồn

### Levinson & Thrun, "Automatic Online Calibration of Cameras and Lasers", RSS 2013

- **Input → output:** ảnh camera + scan LiDAR + extrinsic hiện tại → điểm căn cạnh (độ sâu nhảy của LiDAR có rơi lên cạnh ảnh không), phán quyết hiệu chuẩn còn đúng hay không (so với các phương án hàng xóm), và extrinsic được chỉnh.
- **Metric / dữ liệu / điều kiện của nguồn:** dữ liệu riêng của tác giả, không phải KITTI. Paper cho biết: phát hiện lệch trên 0.25° hoặc 10 cm trong vòng một giây, chính xác 100 %; theo dõi lệch xoay với sai số trung bình 0.10°.
- **Chạy được trên lớp:** tác giả không công bố code; nhóm tự cài bằng NumPy ([t3calib/edge_alignment.py](t3calib/edge_alignment.py)). `python run_benchmark.py` 74.8 s, `python validate_holdout.py --drives 0052 --label holdout` 135.6 s, CPU laptop, không GPU; dữ liệu KITTI 0048 + 0005 (730 MB), 0052 (285 MB).
- **Nhóm tái lập gì:** benchmark mô phỏng (không có demo gốc để chạy): dữ liệu thật, lệch bơm vào extrinsic (proxy). Metric thật so với đáp án KITTI: lỗi chiếu, association, range miss, sai số xoay còn lại; health theo đúng ý của paper (so với 52 hàng xóm).
- **Link, version, dataset, lệnh:** https://www.roboticsproceedings.org/rss09/p29.pdf · code của nhóm, nhánh `main` của repo này · KITTI raw 2011_09_26 drive 0048, 0005, 0052 · các lệnh ở mục (d).

### SoftCorr (hiệu chuẩn LiDAR–camera 6 bậc tự do, không cần huấn luyện, giấy phép MIT)

- **Input → output:** một ảnh RGB + một scan LiDAR + ma trận K + extrinsic khởi tạo → extrinsic 6 bậc tự do (xoay và dịch) đã chỉnh, bằng cách tối đa tương quan cục bộ giữa độ sâu đơn ảnh MoGe-2 và range LiDAR chiếu lên ảnh.
- **Metric / dữ liệu / điều kiện của nguồn:** README của repo cho biết ví dụ KITTI odometry 00/002887 hội tụ khoảng 0.08° / 20 mm so với `T_gt.txt`; cần GPU CUDA.
- **Chạy được trên lớp:** có, chạy thẳng trên Windows (không WSL, không sửa code). RTX 4060 Laptop 8 GB, torch 2.11.0+cu128. Ví dụ gốc 698 s (khoảng 10 phút là tải checkpoint MoGe-2 1.3 GB, khoảng 75 s GPU). Harness T3: khoảng 87 s mỗi lệch mỗi frame; VRAM đỉnh 2.22 GB (MoGe-2) và 0.73 GB (tối ưu).
- **Nhóm tái lập gì:** cả demo gốc (nhóm đo được 0.084° / 19.3 mm, khớp README) lẫn benchmark mô phỏng trên harness T3: 5 trong 20 lệch của `validate_holdout.py`, 1 frame (0052/39) mỗi lệch. Metric thật so với hiệu chuẩn KITTI: sai số xoay (°) và dịch (cm) còn lại.
- **Link, version, dataset, lệnh:** https://github.com/yuhyun00/SoftCorr @ `331c6191195718b36cb795285c6dcb7b1610984a`; MoGe https://github.com/microsoft/MoGe @ `0286b495230a074aadf1c76cc5c679e943e5d1c6` (v2.0.0), checkpoint `Ruicheng/moge-2-vitl-normal` · KITTI raw 0052 · `.venv-softcorr/Scripts/python.exe softcorr_eval.py --drives 0052 --label holdout --frames-per-drive 1 --time-budget-min 10` (cài đặt: [docs/SOFTCORR_SETUP.md](docs/SOFTCORR_SETUP.md)).

### Kiểm tra nhất quán phát hiện vật thể (Ultralytics YOLO26n, gợi ý của giảng viên)

- **Input → output:** ảnh cam 2 → box 2D của YOLO26n (car, truck, bus, person, bicycle, motorcycle; độ tin cậy ≥ 0.4; ảnh 1280 px). So với box 3D chiếu bằng extrinsic hiện tại → IoU, độ lệch tâm Δu, Δv (px), hai luật cảnh báo, ước lượng yaw / pitch.
- **Metric / dữ liệu / điều kiện của nguồn:** YOLO26n là mô hình COCO huấn luyện sẵn của Ultralytics; nhóm không dùng số mAP của họ, chỉ dùng mô hình như một bộ phát hiện camera có sẵn.
- **Chạy được trên lớp:** CPU, venv riêng `.venv-yolo` (ultralytics 8.4.173, torch 2.10.0+cpu). Lần đầu tải `yolo26n.pt` về `.cache/ultralytics/`; khi đã có bộ đệm phát hiện, cả lần chạy mất 3.1 s.
- **Nhóm tái lập gì:** benchmark mô phỏng: phía camera là bộ phát hiện thật, phía LiDAR là proxy (tracklet KITTI thay bộ phát hiện LiDAR, nên là cận trên). Metric: tỉ lệ phát hiện, sai số xoay còn lại so với hiệu chuẩn KITTI. Chi tiết: [docs/DETECTION_CHECK.md](docs/DETECTION_CHECK.md).
- **Link, version, dataset, lệnh:** https://github.com/ultralytics/ultralytics (8.4.173, trọng số `yolo26n.pt`) · KITTI raw 0048 + 0005 (dev), 0052 (hold-out) · `.venv-yolo/Scripts/python detection_check.py` → [results/detection_check/](results/detection_check/).

### KITTI raw (Geiger et al., IJRR 2013)

- **Input → output:** bộ dữ liệu: ảnh cam 2, scan Velodyne, hộp 3D tracklet, tệp hiệu chuẩn → nhóm dùng hiệu chuẩn làm đáp án, tracklet làm box 2D thật (bộ phát hiện hoàn hảo) và làm proxy bộ phát hiện LiDAR.
- **Metric / dữ liệu / điều kiện của nguồn:** bản ghi 2011_09_26, ban ngày, đường phố; hiệu chuẩn camera–LiDAR do KITTI cung cấp; giấy phép CC BY-NC-SA 3.0, phi thương mại.
- **Chạy được trên lớp:** `python get_data.py --drives 0048 0005 0052` tải 84 + 646 + 285 MB qua 24 kết nối song song; không cần đăng nhập.
- **Nhóm tái lập gì:** dữ liệu thật; nhóm không tái lập phép hiệu chuẩn của KITTI, chỉ coi nó là đáp án đúng.
- **Link, version, dataset, lệnh:** https://www.cvlibs.net/datasets/kitti/raw_data.php (bucket `s3.eu-central-1.amazonaws.com/avg-kitti/raw_data`) · `2011_09_26_drive_0048_sync`, `_0005_sync`, `_0052_sync`, `2011_09_26_calib`, các zip `_tracklets` · `python get_data.py --drives 0048 0005 0052`.

## (a) Yêu cầu → bằng chứng

| Loại | Yêu cầu của đề | Bằng chứng trong repo |
|---|---|---|
| Đầu ra | 1 trang / slides ngắn | [REPORT_1PAGE.md](REPORT_1PAGE.md); [slides](https://claude.ai/artifact/MohVqPy4yyGuHUUeV81r94) (riêng tư cho tới khi được chia sẻ); bản đầy đủ [REPORT.md](REPORT.md) |
| Đầu ra | Log / ảnh chạy code | Log: [results/run_log.txt](results/run_log.txt), [results/holdout/dev_log.txt](results/holdout/dev_log.txt), [results/holdout/holdout_log.txt](results/holdout/holdout_log.txt), [results/demo/demo_log.txt](results/demo/demo_log.txt), [results/demo/demo_log_yaw2.5.txt](results/demo/demo_log_yaw2.5.txt). Ảnh: [01_overlay.png](results/figures/01_overlay.png), [02_rotation_sweep.png](results/figures/02_rotation_sweep.png), [03_translation_depth.png](results/figures/03_translation_depth.png), [04_monitor.png](results/figures/04_monitor.png), [demo_before_after.png](results/demo/demo_before_after.png), [demo_before_after_yaw2.5.png](results/demo/demo_before_after_yaw2.5.png) |
| Đầu ra | 1 metric benchmark | Lỗi chiếu (px), association, late-fusion range miss theo độ lệch: [REPORT_1PAGE.md § 3](REPORT_1PAGE.md#3-benchmark--kết-quả), [REPORT.md § 3](REPORT.md#3-benchmark--kết-quả); số gốc: [results/rotation_sweep.csv](results/rotation_sweep.csv), [results/translation_sweep.csv](results/translation_sweep.csv), [results/monitor_sweep.csv](results/monitor_sweep.csv), [results/refinement.json](results/refinement.json) |
| Đầu ra | 1 failure case | Đỉnh giả roll–pitch trên drive 0052: [REPORT_1PAGE.md § 4](REPORT_1PAGE.md#4-failure-case--trường-hợp-hỏng), [REPORT.md § 4](REPORT.md#4-failure-case--trường-hợp-hỏng); số gốc: [results/holdout/holdout_random_drifts.csv](results/holdout/holdout_random_drifts.csv) (drift_id 1, 15, 19), [results/holdout/holdout_summary.json](results/holdout/holdout_summary.json) |
| Đầu ra | 1 đề xuất cải tiến | Tìm thô đến tinh + ràng buộc mặt đường như Galibr, triển khai hai tầng (monitor rẻ → hiệu chuẩn lại nặng): [REPORT_1PAGE.md § 5](REPORT_1PAGE.md#5-engineering-decision--quyết-định-kỹ-thuật); "Cách sửa đề xuất" ở [REPORT.md § 4](REPORT.md#4-failure-case--trường-hợp-hỏng) và "Làm tiếp" ở [REPORT.md § 5](REPORT.md#5-engineering-decision--quyết-định-kỹ-thuật) |
| Rubric 40 % | Benchmark / demo chạy được (code, log, ảnh, plot) | Code: [get_data.py](get_data.py), [run_benchmark.py](run_benchmark.py), [validate_holdout.py](validate_holdout.py), [demo.py](demo.py), [t3calib/](t3calib/); log và ảnh như hàng "Log / ảnh" ở trên; kết quả kiểm định: [results/holdout/](results/holdout/); demo: [results/demo/](results/demo/) ([demo_summary.json](results/demo/demo_summary.json)); lệnh và thời gian chạy: mục (d) dưới đây |
| Rubric 25 % | Hiểu failure thực tế | [REPORT.md § 4](REPORT.md#4-failure-case--trường-hợp-hỏng): ca chính (mù roll), ca thứ hai (đỉnh giả roll–pitch, giải thích qua quầng cạnh 15 px ≈ 1.2°), các ca phụ (tx, hàm điểm số không lồi, không chỉnh tịnh tiến); hình [04_monitor.png](results/figures/04_monitor.png) |
| Rubric 20 % | Giải thích thuật toán: input, output, metric, limitation | [REPORT_1PAGE.md § 2](REPORT_1PAGE.md#2-method--cách-làm) (input / output / hạn chế), [REPORT.md § 2](REPORT.md#2-method--cách-làm) (bảng định nghĩa metric, health 52 hàng xóm, tìm lưới từng trục, hai thay đổi so với paper); code [t3calib/edge_alignment.py](t3calib/edge_alignment.py), [t3calib/metrics.py](t3calib/metrics.py); định nghĩa metric: [README.md § 3](README.md#3-outputs) |
| Rubric 15 % | Trade-off: khi nào nên / không nên dùng (ADAS, robot, drone) | [REPORT_1PAGE.md § 5](REPORT_1PAGE.md#5-engineering-decision--quyết-định-kỹ-thuật); [REPORT.md § 5](REPORT.md#5-engineering-decision--quyết-định-kỹ-thuật): luật kích hoạt, dự phòng, "Khi KHÔNG dùng phương pháp này", bảng "Đánh đổi theo nền tảng" |
| Checklist | Nói rõ ADAS car / ground robot / drone | Xe ADAS: [REPORT_1PAGE.md § 1](REPORT_1PAGE.md#1-problem--vấn-đề), [REPORT.md § 1](REPORT.md#1-problem--vấn-đề); robot và drone: [REPORT_1PAGE.md § 5](REPORT_1PAGE.md#5-engineering-decision--quyết-định-kỹ-thuật), bảng nền tảng ở [REPORT.md § 5](REPORT.md#5-engineering-decision--quyết-định-kỹ-thuật) |
| Checklist | Nói rõ sensor | Camera trước cam 2 + LiDAR Velodyne HDL-64E: [REPORT_1PAGE.md § 1](REPORT_1PAGE.md#1-problem--vấn-đề), [REPORT.md § 1](REPORT.md#1-problem--vấn-đề), [README.md](README.md) |
| Checklist | Một metric định lượng | Bảng ở [REPORT_1PAGE.md § 3](REPORT_1PAGE.md#3-benchmark--kết-quả) (ví dụ yaw 1° → 14.5 px, vật xa còn 84.7 % điểm); kiểm định 0052: sai số xoay còn lại trung vị 0.55° (paper) / 0.44° (improved), phân vị 90 0.70° / 1.75° ([holdout_summary.json](results/holdout/holdout_summary.json)) |
| Checklist | Một failure case cụ thể | 3/20 lần chỉnh trên 0052 kẹt ở 1.74–2.19°: [REPORT_1PAGE.md § 4](REPORT_1PAGE.md#4-failure-case--trường-hợp-hỏng), bảng ca 1 / 19 / 15 ở [REPORT.md § 4](REPORT.md#4-failure-case--trường-hợp-hỏng) |
| Checklist | Tách kết luận paper và kết luận nhóm tự benchmark | Hai câu "Nhóm quan sát được …" / "Paper/repo cho biết …" ở [REPORT_1PAGE.md § 3](REPORT_1PAGE.md#3-benchmark--kết-quả) và trong từng báo cáo cá nhân ở [reports/](reports/); "Kết luận từ paper" và "Kết luận nhóm tự benchmark" (7 ý) ở [REPORT.md § 3](REPORT.md#3-benchmark--kết-quả) |
| Checklist | Proxy / mô phỏng phải giải thích vì sao hợp lý | "Vì sao proxy hợp lý" ở [REPORT_1PAGE.md § 2](REPORT_1PAGE.md#2-method--cách-làm): bơm lệch vào extrinsic đã lưu ≡ LiDAR bị xoay/dịch trên giá đỡ, ảnh và scan là dữ liệu thật; hiệu chuẩn KITTI làm đáp án, đỉnh điểm số cách nó 0.15–0.50° (`score_peak_vs_kitti_deg_*` trong [refinement.json](results/refinement.json); `score_peak_vs_kitti_deg` trong [dev_summary.json](results/holdout/dev_summary.json) và [holdout_summary.json](results/holdout/holdout_summary.json)) |
| Checklist | Paper / repo mới: link, commit / version, dataset, lệnh chạy | Mục Bước 2 ở trên và mục (b) dưới đây; cài đặt SoftCorr: [docs/SOFTCORR_SETUP.md](docs/SOFTCORR_SETUP.md); danh sách nguồn: [REPORT.md, Nguồn](REPORT.md#nguồn) |
| Bổ sung | Hiệu chuẩn lại bằng AI (SoftCorr), kiểm tra nhất quán phát hiện vật thể | SoftCorr: [softcorr_eval.py](softcorr_eval.py), [results/softcorr/holdout_summary.json](results/softcorr/holdout_summary.json) (5/20 lệch, 1 frame mỗi lệch); phát hiện vật thể: [detection_check.py](detection_check.py), [results/detection_check/summary.json](results/detection_check/summary.json), [detection_check.png](results/detection_check/detection_check.png), [docs/DETECTION_CHECK.md](docs/DETECTION_CHECK.md); tóm tắt ở [REPORT_1PAGE.md](REPORT_1PAGE.md) § 3, § 5 và [REPORT.md § 3](REPORT.md#3-benchmark--kết-quả) |
| Nộp bài | TEAMMATES.md, 5 báo cáo cá nhân | [TEAMMATES.md](TEAMMATES.md); [reports/](reports/) (một tệp mỗi thành viên, xem [reports/README.md](reports/README.md)); việc còn lại: [TEAM_TASKS.md](TEAM_TASKS.md) |

## (b) Nguồn đã dùng

### Đã chạy

| Nguồn | Link | Version / commit | Dữ liệu | Lệnh chạy chính xác |
|---|---|---|---|---|
| J. Levinson, S. Thrun, "Automatic Online Calibration of Cameras and Lasers", RSS 2013 — phương pháp được cài đặt | https://www.roboticsproceedings.org/rss09/p29.pdf | Code do nhóm tự viết bằng NumPy ([t3calib/edge_alignment.py](t3calib/edge_alignment.py)), không dùng code của tác giả; commit = nhánh `main` của repo này | KITTI raw 2011_09_26: dev 0048 + 0005 (88 frame), kiểm định 0052 (39 frame) | `python run_benchmark.py`; `python validate_holdout.py --drives 0048 0005 --label dev`; `python validate_holdout.py --drives 0052 --label holdout`; `python demo.py` |
| KITTI raw — A. Geiger et al., "Vision meets Robotics: The KITTI Dataset", IJRR 2013 | https://www.cvlibs.net/datasets/kitti/raw_data.php (tải qua bucket công khai `s3.eu-central-1.amazonaws.com/avg-kitti/raw_data`) | Bản ghi 2011_09_26: `drive_0048_sync` (22 frame, 84 MB), `drive_0005_sync` (154 frame, 646 MB), `drive_0052_sync` (78 frame, 285 MB), `2011_09_26_calib`, các zip `_tracklets`. Giấy phép CC BY-NC-SA 3.0, phi thương mại | cam 2 (`image_02/`), `velodyne_points/`, `tracklet_labels.xml`, `calib_*.txt` | `python get_data.py --drives 0048 0005 0052` |
| SoftCorr — hiệu chuẩn LiDAR–camera không cần huấn luyện (giấy phép MIT) | https://github.com/yuhyun00/SoftCorr | commit `331c6191195718b36cb795285c6dcb7b1610984a` (2026-09-14); MoGe `0286b495230a074aadf1c76cc5c679e943e5d1c6` (v2.0.0), checkpoint `Ruicheng/moge-2-vitl-normal`; torch 2.11.0+cu128, RTX 4060 Laptop | KITTI raw 0052, frame 39; 5 trong 20 lệch của harness T3 (seed 7, xem docstring [softcorr_eval.py](softcorr_eval.py)) | `.venv-softcorr/Scripts/python.exe softcorr_eval.py --drives 0052 --label holdout --frames-per-drive 1 --time-budget-min 10`; cài đặt ở [docs/SOFTCORR_SETUP.md](docs/SOFTCORR_SETUP.md) |
| Ultralytics YOLO26n — bộ phát hiện camera cho kiểm tra nhất quán phát hiện | https://github.com/ultralytics/ultralytics | ultralytics 8.4.173, trọng số `yolo26n.pt`, torch 2.10.0+cpu (venv `.venv-yolo`) | KITTI raw 0048 + 0005 (dev), 0052 (hold-out); tracklet KITTI thay bộ phát hiện LiDAR (cận trên) | `.venv-yolo/Scripts/python detection_check.py` |

Môi trường của các lần chạy đã nộp: Python 3.13.14, numpy 2.2.6, pandas 3.0.5, matplotlib 3.11.1, Pillow 12.3.0, OpenCV 5.0.0; CPU laptop, không GPU. Riêng SoftCorr chạy trên GPU trong `.venv-softcorr`; kiểm tra phát hiện chạy trên CPU trong `.venv-yolo` (numpy 2.5.2).

### Đã đọc, không chạy

| Nguồn | Link | Vì sao không chạy | Nhóm dùng gì từ nó |
|---|---|---|---|
| J. Moravec, R. Šára, "Online Camera-LiDAR Calibration Monitoring and Rotational Drift Tracking", IEEE T-RO 2024 | DOI [10.1109/TRO.2023.3347130](https://doi.org/10.1109/TRO.2023.3347130); bản thảo http://hdl.handle.net/10467/113999; code https://github.com/moravecj/OCaMo (MATLAB) | Code OCaMo chỉ chạy trên dữ liệu CARLA | Cách đặt bài toán: giám sát liên tục và theo dõi lệch xoay |
| Song et al., "Galibr: Targetless LiDAR-Camera Extrinsic Calibration Method via Ground Plane Initialization" | https://arxiv.org/abs/2406.11599 | Không có code | Ràng buộc mặt đường cho roll, pitch trong đề xuất cải tiến |
| Cheng et al., "CalibRefine", arXiv 2025 | https://arxiv.org/abs/2502.17648; code https://github.com/radar-lab/Lidar_Camera_Automatic_Calibration | Đánh giá trên dữ liệu giao thông không phải KITTI | Tham khảo, không dùng số |
| Han et al., "DF-Calib / UniCalib: Targetless LiDAR-Camera Calibration via Depth Flow", arXiv 2025 | https://arxiv.org/abs/2504.01416 | Không có trọng số công khai | Chỉ trích số họ báo cáo trên KITTI (0.045° xoay, 0.635 cm dịch); nhóm không chạy lại |
| Tahiraj et al., "Cal or No Cal? Real-Time Miscalibration Detection of LiDAR and Camera Sensors" | https://arxiv.org/abs/2504.01040; code https://github.com/TUMFTM/MiscalibrationDetection | Chỉ phát hiện, không chỉnh; không có trọng số. Danh sách Eigen của repo có 0005 và 0048, nên kết quả học máy trên hai drive này sẽ lạc quan | Lý do nhóm kiểm định thêm trên drive 0052 |

## (c) Phân công

| Thành viên | Phần việc | Issue |
|---|---|---|
| Hoàng Quốc Việt | Benchmark lõi ([run_benchmark.py](run_benchmark.py), [t3calib/](t3calib/)), monitor sức khỏe, kiểm định hold-out ([validate_holdout.py](validate_holdout.py)), kiểm tra phát hiện vật thể ([detection_check.py](detection_check.py)), SoftCorr ([softcorr_eval.py](softcorr_eval.py)), báo cáo ([REPORT.md](REPORT.md), [REPORT_1PAGE.md](REPORT_1PAGE.md), [reports/HoangQuocViet.md](reports/HoangQuocViet.md)) | — |
| Nguyễn Minh Dương | Thêm requirements.txt và script chạy một lệnh; kiểm tra độ nghiêng mặt đường để bắt lệch roll/pitch | [#3](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/3), [#4](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/4) |
| Hoàng Trung Khải | Unit test cho t3calib bằng pytest; kiểm định thêm trên một drive KITTI chưa dùng (0001) | [#5](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/5), [#6](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/6) |
| Nguyễn Hoàng Sơn | Phân tích association theo loại vật thể; chọn ngưỡng kích hoạt hiệu chuẩn lại (độ nhạy và báo động giả) | [#7](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/7), [#8](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/8) |
| Dương Dương | Bộ câu hỏi và trả lời cho buổi thuyết trình; kịch bản nói 3–5 phút chia đều cho 5 thành viên | [#9](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/9), [#10](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/10) |

Phần việc của bốn thành viên lấy theo tiêu đề issue trên GitHub; các bước còn lại của từng người ở [TEAM_TASKS.md](TEAM_TASKS.md).

## (d) Lệnh tái lập

Chạy từ thư mục gốc của repo. Thời gian lấy từ log của các lần chạy đã nộp (CPU laptop, không GPU).

```bash
pip install numpy pandas matplotlib pillow opencv-python
python get_data.py --drives 0048 0005 0052
python run_benchmark.py
python validate_holdout.py --drives 0048 0005 --label dev
python validate_holdout.py --drives 0052 --label holdout
python demo.py
python demo.py --yaw 2.5 --pitch 0 --roll 0 --tx 0
.venv-yolo/Scripts/python detection_check.py
.venv-softcorr/Scripts/python.exe softcorr_eval.py --drives 0052 --label holdout --frames-per-drive 1 --time-budget-min 10
```

| Lệnh | Thời gian | Ghi ra |
|---|---|---|
| `python get_data.py --drives 0048 0005 0052` | tùy mạng; tải 84 + 646 + 285 MB, 24 kết nối song song | `data/`: khoảng 1.2 GB cho 0048 + 0005 kể cả zip, 0052 thêm khoảng 0.5 GB |
| `python run_benchmark.py` | 74.8 s | [results/run_log.txt](results/run_log.txt), `rotation_sweep.csv`, `translation_sweep.csv`, `monitor_sweep.csv`, `refinement.json`, `results/figures/01–04*.png` |
| `python validate_holdout.py --drives 0048 0005 --label dev` | 162.8 s | `results/holdout/dev_summary.json`, `dev_random_drifts.csv`, `dev_log.txt` |
| `python validate_holdout.py --drives 0052 --label holdout` | 135.6 s | `results/holdout/holdout_summary.json`, `holdout_random_drifts.csv`, `holdout_log.txt` |
| `python demo.py` | 4 s (`runtime_s` 3.8 trong `demo_summary.json`) | `results/demo/demo_log.txt`, `demo_summary.json`, `demo_before_after.png` |
| `.venv-yolo/Scripts/python detection_check.py` | 3.1 s khi đã có bộ đệm phát hiện; lần đầu còn tải `yolo26n.pt` và chạy YOLO26n trên CPU | `results/detection_check/`: `summary.json`, `sweep.csv`, `random_drifts_0052.csv`, `detection_check.png`, `log.txt` |
| `.venv-softcorr/Scripts/python.exe softcorr_eval.py --drives 0052 --label holdout --frames-per-drive 1 --time-budget-min 10` | 526.5 s (dừng ở giới hạn 10 phút sau 5 lệch; khoảng 87 s mỗi lệch, RTX 4060 Laptop) | `results/softcorr/holdout_summary.json`, `holdout_random_drifts.csv`, `holdout_log.txt` |
| `python demo.py --yaw 2.5 --pitch 0 --roll 0 --tx 0` | 3.6 s | cùng ba tệp trong `results/demo/`, ghi đè bản trên; bản trong repo đã đổi tên thành `*_yaw2.5.*`. Thêm `--out <thư mục khác>` để giữ cả hai |

- Kiểm tra nhanh trên drive nhỏ: `python run_benchmark.py --drives 0048 --out results_quick`. Thiếu `--out` thì lệnh này ghi đè `results/` bằng số của 11 frame.
- SoftCorr và kiểm tra nhất quán phát hiện cần venv riêng: cài theo [docs/SOFTCORR_SETUP.md](docs/SOFTCORR_SETUP.md) và mục "Detection consistency check" của [README.md](README.md); phương pháp ở docstring đầu [detection_check.py](detection_check.py) và [docs/DETECTION_CHECK.md](docs/DETECTION_CHECK.md).
