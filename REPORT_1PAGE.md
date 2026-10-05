# T3 — Lệch hiệu chuẩn camera–LiDAR trên xe ADAS

Nhóm Ceiling Fan ([TEAMMATES.md](TEAMMATES.md)) · KITTI raw 2011_09_26 · Bản đầy đủ: [REPORT.md](REPORT.md) · Nguồn, lệnh: [SUBMISSION.md](SUBMISSION.md) · Báo cáo cá nhân: [reports/](reports/)

### 1. Problem — vấn đề

**Nền tảng:** xe ADAS. **Cảm biến:** camera trước (cam 2) và LiDAR Velodyne HDL-64E.
Cú va vào giá đỡ làm lệch tham số ngoài (extrinsic). Ảnh và scan vẫn bình thường, nhưng điểm LiDAR chiếu sai chỗ nên box 2D lấy nhầm độ sâu. Lệch bao nhiêu thì ghép hỏng, và có tự phát hiện được không?

### 2. Method — cách làm

- **Đo thiệt hại:** lệch từng trục (xoay 0–3°, dịch 0–20 cm); đo lỗi chiếu (px), tỉ lệ điểm còn trong box 2D thật (association), tỉ lệ late fusion đo sai khoảng cách.
- **Thuật toán** (Levinson & Thrun, RSS 2013): *input* ảnh, scan, extrinsic hiện tại; chỗ độ sâu nhảy phải rơi lên cạnh ảnh. *Output* health = % trong 52 hàng xóm có điểm thấp hơn (< 90 % → báo) và góc xoay chỉnh lại. *Hạn chế:* chỉ chỉnh xoay, cần nhiều cạnh. Bản *improved*: quầng cạnh hẹp hơn, tìm độ sâu nhảy cả giữa hai vòng quét.
- **Vì sao proxy hợp lý:** bơm lệch vào extrinsic đã lưu ≡ LiDAR bị xoay/dịch trên giá đỡ; ảnh và scan vẫn thật. Hiệu chuẩn KITTI làm đáp án; đỉnh điểm số nằm cách nó 0.15–0.50°.

### 3. Benchmark — kết quả

Dev 0048 + 0005: 88 frame, 241 vật. `python run_benchmark.py`: 74.8 s, CPU. [Log](results/run_log.txt) · [Hình: đúng / lệch / sau khi chỉnh](results/figures/01_overlay.png)

| Lệch | Lỗi chiếu (px) | Giữ điểm, vật xa (%) | Sai khoảng cách (%) |
|---|---|---|---|
| không lệch | 0.0 | 100.0 | 3.3 (nền) |
| yaw 1° / 2° | 14.5 / 28.9 | 84.7 / 61.5 | 5.4 / 44.4 |
| pitch 1° / 2° | 13.0 / 26.0 | 78.1 / 41.2 | 4.6 / 27.0 |
| roll 2° | 9.5 | 93.4 | 3.7 |
| ty 10 cm | 9.5 (0–10 m), 2.7 (20–40 m) | 99.2 | 2.9 |

**Kiểm định drive 0052** (chưa dùng để tinh chỉnh, 20 lệch ngẫu nhiên; `python validate_holdout.py --drives 0052 --label holdout`, 135.6 s): không báo nhầm; paper bắt 95 %, improved 90 %. Sai số xoay còn lại, trung vị: paper 0.55°, improved 0.44°; phân vị 90: 0.70° và **1.75°**.
**Kiểm tra bằng phát hiện vật thể** (gợi ý của giảng viên; `.venv-yolo/Scripts/python detection_check.py`, [summary.json](results/detection_check/summary.json)): so box YOLO26n của camera với box 3D chiếu từ LiDAR, dùng tracklet KITTI thay bộ phát hiện LiDAR nên là **cận trên**. Trên 0052: bắt 95 % (luật độ lệch tâm), sai số xoay còn lại trung vị 0.45°, phân vị 90 1.46°; pitch / yaw chỉ còn ≈ 0.09–0.10° nhưng không thấy roll.

**Nhóm quan sát được** (KITTI, tự đo): xoay hại hơn dịch; mốc nguy hiểm khoảng 1°; monitor bắt yaw, pitch từ 0.5°, không đạt mức 0.25° và mù roll.
**Paper/repo cho biết** (Levinson & Thrun, dữ liệu riêng, không phải KITTI): bắt lệch > 0.25° hoặc 10 cm trong một giây, chính xác 100 %.

### 4. Failure case — trường hợp hỏng

**Đỉnh giả roll–pitch trên 0052 (improved):** 3/20 lần chỉnh bị kẹt ở 1.74–2.19° sai số xoay. Cả ba có roll ≈ −1.4°, pitch ≈ +1°; monitor báo đúng nhưng bộ chỉnh gần như không đổi roll, pitch; bản paper đưa về 0.44–0.75°. *Giả thuyết:* tìm từng trục kẹt ở nơi roll, pitch bù trừ nhau; lệch vượt quầng cạnh 15 px (≈ 1.2°) nên điểm số phẳng.

### 5. Engineering decision — quyết định kỹ thuật

**Đề xuất cải tiến:** tìm thô đến tinh (quầng rộng rồi hẹp) cộng ràng buộc mặt đường cho roll, pitch như Galibr; đạt khi ba ca kẹt xuống dưới 1° trong `validate_holdout.py` trên 0052. Triển khai hai tầng: monitor rẻ chạy liên tục trên CPU (health < 90 % → giảm trọng số ghép, lấy khoảng cách từ LiDAR thuần), rồi mới gọi hiệu chuẩn lại nặng; chỉ nhận extrinsic mới khi health ≥ 90 % trở lại.
**Tầng nặng đã đo — SoftCorr** (commit `331c619`, [summary.json](results/softcorr/holdout_summary.json)): mới chạy **5/20 lệch, mỗi lệch 1 frame** của 0052. Sai số xoay còn lại trung vị 0.243°, tệ nhất 0.245°, dịch còn 1.85 cm (bản cạnh để nguyên 5.05 cm); cùng 5 lệch, paper 0.605°, improved 0.445° (tệ nhất 2.19°). Xuất phát từ hiệu chuẩn KITTI nó cũng dời 0.241° / 1.8 cm, nên mọi lệch hội tụ về cùng một điểm: độ lệch cố định của đỉnh điểm số frame này. ≈ 87 s mỗi lệch trên RTX 4060 Laptop, nên hợp làm tầng hiệu chuẩn lại, không làm monitor.
**Nên dùng:** xe ADAS trong phố, làm monitor rẻ. **Hạn chế:** robot tầm gần (dịch 10 cm đã 9.5 px, mà không chỉnh dịch). **Không dùng một mình:** drone (nhìn xuống đất, ít độ sâu nhảy); đêm, mưa, lệch trên 3°.
