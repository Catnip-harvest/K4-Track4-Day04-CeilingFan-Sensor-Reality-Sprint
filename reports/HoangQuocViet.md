# Báo cáo cá nhân — Hoàng Quốc Việt

**MSSV:** 2A202602563 · **Nhóm:** Ceiling Fan ([TEAMMATES.md](../TEAMMATES.md)) · **Repo:** https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint

**Phần tôi phụ trách:** lõi benchmark ([run_benchmark.py](../run_benchmark.py), [t3calib/](../t3calib/)), monitor sức khỏe hiệu chuẩn, kiểm định hold-out ([validate_holdout.py](../validate_holdout.py)), báo cáo nhóm ([REPORT.md](../REPORT.md), [REPORT_1PAGE.md](../REPORT_1PAGE.md)).

## 1. Problem — vấn đề

Nền tảng là xe ADAS. Tính năng tôi xét là late fusion camera–LiDAR: box 2D của camera lấy khoảng cách từ các điểm LiDAR rơi vào nó. Sensor: camera màu phía trước (cam 2) và LiDAR Velodyne HDL-64E của KITTI.
Lỗi thực tế: sau rung lắc hay một cú va vào giá đỡ, tham số ngoài (extrinsic) lệch vài phần độ. Ảnh vẫn nét, scan vẫn đúng hình, nên không cảm biến nào tự báo lỗi; nhưng điểm LiDAR chiếu sai chỗ và box lấy nhầm độ sâu. Tôi muốn biết lệch bao nhiêu thì ghép hỏng, và một monitor không cần bảng hiệu chuẩn có bắt được không.

## 2. Method — cách làm

- **Nguồn:** J. Levinson, S. Thrun, "Automatic Online Calibration of Cameras and Lasers", RSS 2013 — https://www.roboticsproceedings.org/rss09/p29.pdf. Tác giả không công bố code; tôi tự cài bằng NumPy trong [t3calib/edge_alignment.py](../t3calib/edge_alignment.py) (version = nhánh `main` của repo nhóm).
- **Input → output:** ảnh cam 2 + scan LiDAR + extrinsic hiện tại → *health* = phần trăm trong 52 phương án hàng xóm có điểm căn cạnh thấp hơn (< 90 % → đề nghị hiệu chuẩn lại), và góc xoay đã chỉnh (tìm lưới từng trục yaw → pitch → roll, ±3°).
- **Giả định:** chỗ độ sâu LiDAR nhảy đột ngột trùng với cạnh ảnh; cảnh có đủ cạnh; hiệu chuẩn KITTI là đáp án; bơm lệch vào extrinsic đã lưu tương đương LiDAR bị xoay/dịch trên giá đỡ. Chỉ chỉnh xoay, không chỉnh dịch.
- **Dataset và lệnh:** KITTI raw 2011_09_26 — dev 0048 + 0005 (88 frame, 241 lượt vật), hold-out 0052 (39 frame). `python get_data.py --drives 0048 0005 0052`; `python run_benchmark.py` (74.8 s, CPU laptop); `python validate_holdout.py --drives 0052 --label holdout` (135.6 s).

## 3. Benchmark — kết quả

Dữ liệu thật, lệch mô phỏng (proxy). Metric: lỗi chiếu (px), tỉ lệ giữ điểm trong box 2D thật (%), late-fusion range miss (%). Baseline là hiệu chuẩn KITTI; điều kiện lỗi là lệch một trục. Bảng đầy đủ ở [REPORT.md § 3](../REPORT.md#3-benchmark--kết-quả), số gốc [results/rotation_sweep.csv](../results/rotation_sweep.csv), hình [02_rotation_sweep.png](../results/figures/02_rotation_sweep.png), log [results/run_log.txt](../results/run_log.txt).

| Điều kiện | Lỗi chiếu (px) | Giữ điểm, vật xa (%) | Sai khoảng cách (%) |
|---|---|---|---|
| Baseline: hiệu chuẩn KITTI | 0.0 | 100.0 | 3.3 (mức nền) |
| yaw 1° / 2° | 14.5 / 28.9 | 84.7 / 61.5 | 5.4 / 44.4 |
| pitch 1° / 2° | 13.0 / 26.0 | 78.1 / 41.2 | 4.6 / 27.0 |

Kiểm định trên 0052, 20 lệch ngẫu nhiên (seed 7, xoay ±1.5°, dịch ±5 cm; [holdout_summary.json](../results/holdout/holdout_summary.json)): không báo nhầm ở hiệu chuẩn đúng; bản paper bắt 95 %, bản improved 90 %; sai số xoay còn lại trung vị 0.55° / 0.44°, phân vị 90 0.70° / 1.75°.

**Nhóm quan sát được** xoay hại hơn dịch (yaw 1° dời 14.5 px ở mọi độ sâu, ty 10 cm chỉ 2.7 px ở 20–40 m), mốc nguy hiểm khoảng 1°, và monitor bắt yaw, pitch từ 0.5° nhưng không bắt roll tới 2°.
**Paper/repo cho biết** phương pháp phát hiện lệch trên 0.25° hoặc 10 cm trong vòng một giây với độ chính xác 100 %, trên dữ liệu riêng của tác giả, không phải KITTI. Tôi không đạt mức 0.25° đó trên KITTI.

## 4. Failure case — đỉnh giả roll–pitch trên 0052

Bản improved chỉnh hỏng 3/20 lần: drift_id 1, 19, 15 trong [holdout_random_drifts.csv](../results/holdout/holdout_random_drifts.csv) còn 2.19°, 1.84°, 1.74°. Cả ba bắt đầu với roll ≈ −1.4°, pitch ≈ +1°. Monitor báo đúng (health 76.9 %, 46.2 %, 59.6 %), nhưng sau khi chỉnh roll và pitch gần như không đổi; bản paper đưa cả ba về 0.44–0.75°.
*Giả thuyết (chưa kiểm chứng riêng):* tìm theo từng trục dừng ở chỗ roll và pitch bù trừ nhau; lệch tổng 1.79–1.99° vượt quầng cạnh 15 px (≈ 1.2° ở f = 721.5 px) nên điểm số gần như phẳng. Phân tích đầy đủ: [REPORT.md § 4](../REPORT.md#4-failure-case--trường-hợp-hỏng), hình [04_monitor.png](../results/figures/04_monitor.png).

## 5. Engineering decision — quyết định kỹ thuật

- **Log:** mỗi cửa sổ 24 frame ghi health, điểm số, mức cải thiện của hàng xóm tốt nhất, số điểm "mép" LiDAR trong khung nhìn, độ lệch xoay ước lượng, tốc độ xe, sự kiện xóc từ IMU.
- **Dự phòng:** health < 90 % → giảm trọng số ghép, lấy khoảng cách từ LiDAR thuần, loại vật từ camera thuần; chỉ nhận extrinsic mới khi health ≥ 90 % ở cửa sổ kế tiếp.
- **Dữ liệu tiếp theo:** ảnh và scan quanh mỗi lần báo; đo lại bằng bảng hiệu chuẩn mỗi lần bảo dưỡng để có đáp án thật thay cho proxy.
- **Cải tiến gắn với failure case:** tìm thô đến tinh (quầng rộng rồi hẹp) cộng ràng buộc mặt đường cho roll, pitch như Galibr (https://arxiv.org/abs/2406.11599). Kiểm chứng bằng đúng lệnh `python validate_holdout.py --drives 0052 --label holdout`: đạt khi ba ca 1, 15, 19 xuống dưới 1°, phân vị 90 của improved ≤ 0.70° (mức bản paper) và vẫn không báo nhầm.
- **Tầng nặng nhóm đã đo:** SoftCorr trên 0052 (mới 5/20 lệch, mỗi lệch 1 frame) còn ≈ 0.24° / 1.8 cm từ mọi điểm xuất phát, ≈ 87 s mỗi lệch trên GPU ([results/softcorr/holdout_summary.json](../results/softcorr/holdout_summary.json)). Vì vậy tôi giữ monitor rẻ chạy liên tục trên CPU và chỉ gọi tầng nặng khi monitor báo.
