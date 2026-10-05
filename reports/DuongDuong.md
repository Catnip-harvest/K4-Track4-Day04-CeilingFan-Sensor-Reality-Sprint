# Báo cáo cá nhân — Dương Dương

**MSSV:** 2A2026202498 · **Nhóm:** Ceiling Fan ([TEAMMATES.md](../TEAMMATES.md)) · **Repo:** https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint

**Phần tôi phụ trách:** chuẩn bị bộ câu hỏi–trả lời cho phần phản biện ([docs/QA.md](../docs/QA.md), issue #9) và kịch bản thuyết trình 3–5 phút chia đều cho năm thành viên ([docs/SCRIPT.md](../docs/SCRIPT.md), issue #10). Khi làm hai phần này, tôi kiểm tra lại các kết luận trình bày với số gốc trong `results/` để tránh nhầm số của paper với số nhóm tự đo.

## 1. Problem — vấn đề

Nhóm xét một xe ADAS dùng camera màu phía trước (KITTI cam 2) và LiDAR Velodyne HDL-64E. Tính năng cụ thể là late fusion: hộp 2D từ camera nhận khoảng cách từ các điểm LiDAR được chiếu vào hộp. Nếu giá đỡ bị rung hoặc va chạm, extrinsic camera–LiDAR có thể lệch dù ảnh và đám mây điểm riêng lẻ vẫn trông bình thường. Khi đó hệ thống có thể ghép điểm của nền hoặc của vật khác vào hộp, làm sai khoảng cách mà không có sensor nào tự báo hỏng.

Câu hỏi của tôi khi đọc kết quả là: mức lệch nhỏ có gây hậu quả đo được hay không, monitor căn cạnh có phát hiện đáng tin cậy trên một drive chưa dùng để tinh chỉnh hay không, và nhóm nên nói rõ giới hạn nào khi bảo vệ bài.

## 2. Method — cách làm

- **Phương pháp chính:** nhóm tự cài lại ý tưởng targetless của Levinson và Thrun, “Automatic Online Calibration of Cameras and Lasers”, RSS 2013 (https://www.roboticsproceedings.org/rss09/p29.pdf), bằng NumPy trong [t3calib/edge_alignment.py](../t3calib/edge_alignment.py). Ảnh camera, scan LiDAR và extrinsic hiện tại được biến thành điểm căn cạnh; *health* là tỉ lệ trong 52 phương án lân cận có điểm thấp hơn. Health dưới 90% thì đề nghị hiệu chuẩn lại.
- **Input → output:** ảnh cam 2 + scan Velodyne + extrinsic hiện tại → health, quyết định cảnh báo và extrinsic đã được chỉnh xoay. Bộ chỉnh tìm lần lượt yaw, pitch và roll; phiên bản lõi chưa chỉnh tịnh tiến.
- **Proxy và giả định:** nhóm lấy ảnh và scan thật của KITTI, sau đó chủ động nhân một phép xoay/dịch vào extrinsic. Đây là proxy hợp lý cho việc cảm biến dịch trên giá đỡ vì quan hệ hình học tương đối thay đổi theo cùng cách, trong khi hiệu chuẩn KITTI cho một mốc đối chiếu. Giả định quan trọng là cảnh có đủ cạnh ảnh trùng với bước nhảy độ sâu LiDAR và hiệu chuẩn KITTI được coi là đáp án.
- **Dữ liệu và lệnh:** KITTI raw ngày 2011_09_26; drive 0048 + 0005 để phát triển, drive 0052 để kiểm định. Lệnh tạo số liệu chính là `python run_benchmark.py` và `python validate_holdout.py --drives 0052 --label holdout`; dữ liệu được tải bằng `python get_data.py --drives 0048 0005 0052`.

Ngoài monitor căn cạnh, nhóm còn kiểm tra chéo bằng YOLO26n so với tracklet KITTI ([docs/DETECTION_CHECK.md](../docs/DETECTION_CHECK.md)) và thử SoftCorr commit `331c6191195718b36cb795285c6dcb7b1610984a` trên một phần hold-out ([results/softcorr/holdout_summary.json](../results/softcorr/holdout_summary.json)). Tracklet thay bộ phát hiện LiDAR nên kết quả detection check chỉ là cận trên, không phải một pipeline phát hiện hai phía hoàn chỉnh.

## 3. Benchmark — kết quả

Benchmark dùng dữ liệu cảm biến thật nhưng lỗi extrinsic được mô phỏng có kiểm soát. Baseline là hiệu chuẩn KITTI. Các metric gồm lỗi chiếu trung vị (px), tỉ lệ điểm LiDAR của vật còn nằm trong hộp 2D (%), tỉ lệ late fusion đo sai khoảng cách (%) và sai số xoay còn lại sau hiệu chỉnh (°). Hình tổng hợp được tạo bởi `python run_benchmark.py`: [02_rotation_sweep.png](../results/figures/02_rotation_sweep.png) và [04_monitor.png](../results/figures/04_monitor.png); số gốc nằm trong [rotation_sweep.csv](../results/rotation_sweep.csv) và [holdout_summary.json](../results/holdout/holdout_summary.json).

| Điều kiện | Kết quả chính |
|---|---|
| Baseline KITTI | 0 px lỗi chiếu; 100% điểm được giữ; range miss nền 3,3% |
| Yaw 1° | 14,5 px; vật xa còn 84,7% điểm; range miss 5,4% |
| Pitch 1° | 13,0 px; vật xa còn 78,1% điểm; range miss 4,6% |
| Hold-out 0052, paper | phát hiện 95%; residual trung vị 0,55°, p90 0,70° |
| Hold-out 0052, improved | phát hiện 90%; residual trung vị 0,44°, p90 1,75° |

**Nhóm quan sát được** mốc khoảng 1° đã đáng chú ý cho late fusion: vật xa mất điểm sớm hơn, pitch 1° chỉ còn giữ 78,1% điểm của vật xa, còn yaw 1° làm điểm dời 14,5 px. Trên 0052, bản improved tốt hơn về trung vị nhưng có đuôi lỗi xấu hơn bản paper, vì ba ca hiệu chỉnh bị kẹt.

**Paper/repo cho biết** phương pháp gốc phát hiện sai lệch lớn hơn 0,25° hoặc 10 cm trong vòng một giây với độ chính xác 100% trên dữ liệu riêng của tác giả. Nhóm không tái lập được ngưỡng 0,25° trên KITTI và không dùng con số của paper như kết quả benchmark của nhóm.

Kiểm tra phát hiện vật thể trên 0052 bắt 95% lệch ngẫu nhiên theo luật độ lệch tâm, nhưng gần như không quan sát roll; phía LiDAR dùng tracklet thật nên đây là cận trên. SoftCorr cho residual trung vị khoảng 0,243° và 1,85 cm, nhưng mới chạy 5/20 lệch, một frame mỗi lệch, mất khoảng 87 giây/ca trên RTX 4060 Laptop.

## 4. Failure case — đỉnh giả roll–pitch trên drive 0052

Failure case tôi chọn là ba lần bản improved báo đúng nhưng chỉnh sai trên drive chưa dùng để tinh chỉnh. Trong [holdout_random_drifts.csv](../results/holdout/holdout_random_drifts.csv), các drift 1, 15 và 19 bắt đầu với tổng lệch xoay lần lượt 1,99°, 1,79° và 1,91°. Sau chỉnh, residual vẫn là 2,19°, 1,74° và 1,84°; trong khi bản paper đưa cùng ba ca về 0,44–0,75°.

| Drift | Roll / pitch ban đầu | Health improved | Residual improved | Residual paper |
|---|---:|---:|---:|---:|
| 1 | −1,48° / +0,96° | 76,9% | 2,19° | 0,44° |
| 15 | −1,39° / +1,13° | 59,6% | 1,74° | 0,75° |
| 19 | −1,48° / +0,76° | 46,2% | 1,84° | 0,51° |

*Giả thuyết chưa được kiểm chứng riêng:* bộ tìm theo từng trục dừng tại một đỉnh giả, nơi thay đổi riêng roll hoặc pitch đều làm điểm số xấu đi dù thay đổi đồng thời có thể tốt hơn. Tổng lệch 1,79–1,99° cũng vượt vùng quầng cạnh 15 px, tương đương khoảng 1,2° ở tiêu cự 721,5 px; khi cạnh đúng đã ra ngoài quầng, bề mặt điểm số dễ phẳng và tạo cực trị cục bộ. Vì vậy “monitor đã báo” không đồng nghĩa “kết quả hiệu chuẩn mới an toàn để áp dụng”.

## 5. Engineering decision — quyết định kỹ thuật

- **Luật vận hành:** chạy monitor rẻ liên tục; health < 90% chỉ tạo yêu cầu hiệu chuẩn lại, không tự động ghi đè extrinsic. Chỉ nhận extrinsic mới nếu cửa sổ kiểm tra kế tiếp đạt health ≥ 90% và metric association/range không xấu hơn baseline vận hành.
- **Dự phòng:** trong lúc chưa có extrinsic tin cậy, giảm trọng số late fusion, dùng khoảng cách LiDAR thuần và giữ nhãn camera ở chế độ degraded thay vì ghép hai sensor bằng một ma trận nghi ngờ.
- **Log cần giữ:** health, điểm số hiện tại và hàng xóm tốt nhất, số cạnh LiDAR trong ảnh, extrinsic cũ/mới, residual ước lượng, tốc độ xe và sự kiện xóc IMU. Lưu ảnh + scan quanh cảnh báo để kiểm tra lại bằng bảng hiệu chuẩn khi bảo dưỡng.
- **Cải tiến gắn trực tiếp với failure case:** thay tìm từng trục một bằng tìm thô đến tinh trên roll–pitch kết hợp; dùng quầng rộng để vào vùng bắt rồi quầng hẹp để chốt. Có thể thêm pháp tuyến mặt đường LiDAR làm ràng buộc roll/pitch như Galibr (https://arxiv.org/abs/2406.11599).
- **Tiêu chí kiểm chứng:** chạy lại `python validate_holdout.py --drives 0052 --label holdout`; cải tiến chỉ được nhận nếu drift 1, 15, 19 đều còn dưới 1°, p90 improved giảm từ 1,75° xuống không quá 0,70° và hiệu chuẩn đúng vẫn không báo nhầm. Cần kiểm tra thêm trên drive khác vì 20 lệch của một drive chưa đủ để kết luận tổng quát.

Trong issue #9, tôi đã chuyển các điểm dễ bị hỏi—proxy, mù roll, đỉnh giả, cận trên của tracklet, phạm vi 5/20 của SoftCorr—thành bộ hỏi–đáp có link tới số gốc. Trong issue #10, tôi chia kịch bản thành năm đoạn theo đúng năm mục báo cáo, mỗi thành viên khoảng 35–45 giây và chỉ dùng hình/bảng đã commit, không phụ thuộc demo trực tiếp.
