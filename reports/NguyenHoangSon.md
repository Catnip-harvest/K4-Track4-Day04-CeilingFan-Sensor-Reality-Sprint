# Báo cáo cá nhân — Nguyễn Hoàng Sơn

**MSSV:** 2A202602457 · **Nhóm:** Ceiling Fan ([TEAMMATES.md](../TEAMMATES.md)) · **Repo:** https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint

**Phần tôi phụ trách:**
- Issue #7: association theo loại vật ([per_class_analysis.py](../per_class_analysis.py), [docs/PER_CLASS.md](../docs/PER_CLASS.md)).
- Issue #8: chọn ngưỡng kích hoạt hiệu chuẩn lại ([threshold_sweep.py](../threshold_sweep.py), [docs/THRESHOLD.md](../docs/THRESHOLD.md)).
- Chạy lại benchmark chung để kiểm tra dữ liệu: `python run_benchmark.py` trên bản KITTI tải về máy tôi cho ra `rotation_sweep.csv` và `translation_sweep.csv` trùng từng số với bản trong repo.

## 1. Problem — vấn đề

- **Nền tảng:** xe ADAS.
- **Tính năng:** đo khoảng cách bằng late fusion. Camera đưa ra box 2D và loại vật; khoảng cách lấy từ trung vị độ sâu của các điểm LiDAR rơi vào tâm box.
- **Sensor:** camera màu phía trước (cam 2) và LiDAR Velodyne HDL-64E của KITTI.
- **Lỗi thực tế:** giá đỡ bị rung hoặc va làm extrinsic camera–LiDAR lệch một chút, chủ yếu là lệch xoay. Từng cảm biến vẫn bình thường nên không cái nào tự báo lỗi.

Báo cáo chung đã cho thấy lệch khoảng 1° là bắt đầu nguy hiểm. Tôi muốn hỏi tiếp hai câu:
1. Mốc 1° đó có đúng cho **mọi loại vật** không?
2. Monitor tự kiểm tra nên báo ở ngưỡng nào để vừa bắt được lệch vừa không báo nhầm?

## 2. Method — cách làm

- **Nguồn thuật toán monitor:** J. Levinson, S. Thrun, "Automatic Online Calibration of Cameras and Lasers", RSS 2013 — https://www.roboticsproceedings.org/rss09/p29.pdf. Nhóm tự cài lại bằng NumPy trong [t3calib/edge_alignment.py](../t3calib/edge_alignment.py).
  - *Input:* ảnh, scan LiDAR, extrinsic hiện tại.
  - *Output:* health = tỉ lệ trong 52 hàng xóm có điểm căn cạnh thấp hơn.
  - *Giả định:* chỗ độ sâu LiDAR nhảy trùng với cạnh ảnh.
- **Issue #7:** dùng lại nguyên `collect_objects` và `association_and_range` trong [t3calib/metrics.py](../t3calib/metrics.py), không sửa gì. Chỉ chia danh sách vật theo nhãn KITTI và theo gần / xa 20 m, rồi tính cho từng nhóm.
  - *Input:* 88 frame của drive 0048 + 0005, 241 lượt vật, lệch yaw / pitch 0.5–2°.
  - *Output:* association retention (%) và range miss (%) theo nhóm.
  - *Giả định:* box 2D là box 3D thật chiếu lên ảnh, tức một detector hoàn hảo.
- **Issue #8:**
  - *Báo động giả:* health ở hiệu chuẩn đúng trên các cửa sổ 12 và 24 frame liên tiếp của 0048, 0005, 0052.
  - *Độ nhạy:* health của 60 lệch ngẫu nhiên và 45 lệch một trục đã lưu trong [results/holdout/](../results/holdout/), gồm cả drive 0001 Khải đã chạy.
  - Quét ngưỡng 0.80–0.98.
- **Dataset và lệnh** (commit trên nhánh `main` của repo nhóm). KITTI raw 2011_09_26, tải bằng `python get_data.py --drives 0048 0005 0052`. Các script nhận `--data-dir` nếu dữ liệu nằm ngoài repo.
  - `python per_class_analysis.py` (≈ 20–40 s, CPU)
  - `python threshold_sweep.py` (≈ 60–75 s, CPU)

## 3. Benchmark — kết quả

Dữ liệu thật, lệch mô phỏng. Baseline là hiệu chuẩn KITTI; điều kiện lỗi là lệch một trục. Mọi nhóm dùng cùng metric.

**#7 — association retention (%)** ([per_class.csv](../results/per_class/per_class.csv), hình [per_class.png](../results/per_class/per_class.png)):

| Nhóm vật (số lượt) | baseline | yaw 1° | yaw 2° | pitch 1° | pitch 2° |
|---|---|---|---|---|---|
| Car gần < 20 m (54) | 100 | 97.3 | 89.8 | 99.1 | 86.7 |
| Car xa ≥ 20 m (34) | 100 | 88.1 | 64.9 | 71.8 | **38.4** |
| Van xa ≥ 20 m (55) | 100 | 82.6 | 59.4 | 82.0 | 43.0 |
| Cyclist (66 lượt, **1 vật**) | 100 | 97.6 | 76.1 | 99.9 | 94.9 |

Range miss của Cyclist: 0 % ở baseline, **16.7 %** ở yaw 1°, **100 %** ở yaw 2°. Pedestrian chỉ có 3 lượt nên tôi không kết luận gì.

**#8 — ngưỡng** ([threshold.csv](../results/threshold/threshold.csv), hình [threshold.png](../results/threshold/threshold.png)), bản improved, gộp 24 frame:

| Ngưỡng | Báo giả, 16 cửa sổ 24 frame | Mẫu gộp báo giả (trên 3) | Bắt 60 lệch ngẫu nhiên | Bắt lệch 1 trục ≥ 0.5° / 10 cm |
|---|---|---|---|---|
| 0.85 | 0 % | 0 | 93.3 % | 71.1 % |
| **0.90** | 0 % | 0 | 93.3 % | 77.8 % |
| 0.91 | 0 % | 1 (drive 0001: 0.904) | 95.0 % | 80.0 % |
| 0.95 | 0 % | 1 | 100.0 % | 84.4 % |

Kiểm tra chéo: ở ngưỡng 0.90, script cho lại đúng 95 % (paper) và 90 % (improved) trên 0052 như CSV đã lưu.

**Nhóm quan sát được:**
- Với cùng độ lệch, vật xa mất điểm trước vật gần, bất kể loại.
- Vật hẹp như người đi xe đạp vẫn giữ gần đủ điểm nhưng đo **sai khoảng cách** ngay từ yaw 1°.
- Ngưỡng 0.90 cho bản improved là ngưỡng cao nhất chưa báo nhầm trên mọi mẫu sạch 24 frame.
- Bản paper báo nhầm 18.8 % cửa sổ 24 frame liên tiếp ở ngưỡng 0.90, đều ở drive 0005.

**Paper/repo cho biết:** Levinson & Thrun phát hiện lệch > 0.25° hoặc 10 cm trong vòng một giây với độ chính xác 100 %, trên dữ liệu riêng của tác giả, không phải KITTI. Paper không tách kết quả theo loại vật. Với monitor của nhóm trên KITTI, tôi không đạt mức 0.25° đó.

## 4. Failure case — người đi xe đạp đo sai khoảng cách dù điểm LiDAR vẫn còn

**Cấu hình:** drive 0005, người đi xe đạp duy nhất (66 lượt, cách trung vị 11.4 m, box rộng trung vị 73 px), lệch yaw 1° và 2°. Bằng chứng ở [per_class.csv](../results/per_class/per_class.csv), dòng `Cyclist`.

**Quan sát:**

| Lệch yaw | Điểm còn trong box | Range miss |
|---|---|---|
| 1° | 97.6 % | 16.7 % |
| 2° | 76.1 % | 100 % |
| 0° (baseline) | 100 % | 0 % |

Với Car gần, yaw 2° vẫn cho range miss 0 %. Nếu chỉ nhìn association, ta sẽ nghĩ xe đạp còn an toàn tới gần 1.5°. Thực tế late fusion đã đưa ra khoảng cách sai cho chính vật dễ tổn thương nhất.

**Hệ quả:** tính năng khoảng cách (cảnh báo va chạm, giữ khoảng cách) nhận sai độ sâu cho xe đạp, trong khi monitor ở yaw 1° có thể vẫn chưa báo. Tôi suy ra điều này từ số đo; nhóm chưa chạy tính năng cảnh báo thật.

*Giả thuyết (chưa kiểm chứng riêng):* yaw 2° dời điểm khoảng 29 px theo chiều ngang (số từ [rotation_sweep.csv](../results/rotation_sweep.csv)). Vùng tâm box mà late fusion dùng chỉ rộng khoảng 37 px, nên phần lớn rơi ra nền phía sau và trung vị lấy nhầm độ sâu của nền. Pitch dời theo chiều dọc mà xe đạp thì cao, nên gần như không ảnh hưởng.

**Giới hạn:**
- Chỉ có **một** người đi xe đạp, nên đây là một ca, không phải thống kê cho cả loại.
- Box 2D là box hoàn hảo; detector thật còn lệch thêm.
- Lệch là mô phỏng.

## 5. Engineering decision — quyết định kỹ thuật

- **Ngưỡng:** giữ health < 0.90 cho bản improved, gộp ≥ 24 frame. Nhưng chỉ kích hoạt khi **hai cửa sổ liên tiếp** cùng dưới ngưỡng, vì biên an toàn chỉ còn một hàng xóm: drive 0001 đúng hiệu chuẩn mà chỉ đạt 0.904. Health 0.90–0.95 thì ghi log mức "theo dõi". Không dùng bản paper và không gộp 12 frame cho monitor (xem [THRESHOLD.md](../docs/THRESHOLD.md)).
- **Dự phòng theo loại vật:** khi đang ở mức "theo dõi" hoặc đã báo, với box hẹp (Cyclist, Pedestrian, box < khoảng 100 px) thì không lấy khoảng cách từ điểm rơi vào tâm box 2D, mà lấy từ box 3D của LiDAR thuần. Lý do: các vật này đo sai khoảng cách sớm nhất.
- **Ghi log:** health theo cửa sổ, range miss proxy theo loại vật, bề rộng box, và số điểm LiDAR trong tâm box so với trong cả box. Tỉ lệ này tụt là dấu hiệu sớm của ca xe đạp ở trên.
- **Dữ liệu cần thêm:** nhiều Cyclist và Pedestrian hơn, ví dụ các drive KITTI trong khu dân cư. Thêm frame ban đêm và mưa để đo báo động giả thật.
- **Cải tiến gắn với failure case:** thay "trung vị độ sâu trong một nửa tâm box" bằng "cụm độ sâu gần nhất chiếm đa số trong cả box". Kiểm chứng bằng `python per_class_analysis.py`, so cột `range_miss_pct`. Đạt khi:
  - range miss của Cyclist ở yaw 1° giảm từ 16.7 % về 0 %;
  - ở yaw 2° giảm rõ so với 100 %;
  - range miss nền của Car xa (đang 23.5 %) không tăng.
- **Trạng thái issue:**
  - #7 xong: script, CSV, hình, tài liệu.
  - #8 xong: script, CSV, hình, đề xuất ngưỡng.
  - Cải tiến cụm độ sâu và xác nhận hai cửa sổ **chưa làm**.
