# Kịch bản thuyết trình 3–5 phút — nhóm Ceiling Fan

Tài liệu cho issue #10. Tổng thời lượng mục tiêu: **3 phút 40 giây**; năm thành viên nói khoảng **40–45 giây/người**. Nhóm không demo trực tiếp: người điều khiển chỉ chuyển giữa các hình và bảng đã commit trong repo.

## Chuẩn bị màn hình

1. [01_overlay.png](../results/figures/01_overlay.png) — hiệu chuẩn đúng, lệch thực tế và sau hiệu chỉnh.
2. [04_monitor.png](../results/figures/04_monitor.png) — score căn cạnh và health theo độ lệch.
3. Bảng benchmark trong [REPORT_1PAGE.md § 3](../REPORT_1PAGE.md#3-benchmark--kết-quả).
4. Bảng ba ca đỉnh giả trong [REPORT.md § 4](../REPORT.md#4-failure-case--trường-hợp-hỏng).
5. [detection_check.png](../results/detection_check/detection_check.png) — kiểm tra chéo bằng phát hiện vật thể.

## 1. Dương Dương — Problem, 0:00–0:42

**Màn hình:** `01_overlay.png`, ưu tiên ba ô toàn cảnh.

> Nhóm em xét một xe ADAS dùng camera màu phía trước và LiDAR Velodyne. Tính năng là late fusion: box 2D từ camera lấy khoảng cách từ các điểm LiDAR được chiếu vào box. Sau rung hoặc va vào giá đỡ, extrinsic giữa hai sensor có thể lệch dù ảnh vẫn nét và scan LiDAR vẫn đúng hình. Khi đó điểm LiDAR rơi sai vật hoặc rơi xuống nền, làm hệ thống đo khoảng cách sai mà không sensor nào tự báo. Câu hỏi của nhóm là: lệch nhỏ đến mức nào đã nguy hiểm, có phát hiện được mà không cần bảng chuẩn hay không, và hiệu chỉnh lại có đáng tin cậy trên dữ liệu chưa dùng để tinh chỉnh không?

**Chuyển:** “Minh Dương sẽ trình bày cách nhóm biến câu hỏi đó thành benchmark.”

## 2. Nguyễn Minh Dương — Method, 0:42–1:25

**Màn hình:** `04_monitor.png`, chỉ vào đồ thị score và health.

> Nhóm dùng ảnh và scan thật của KITTI, coi hiệu chuẩn KITTI là mốc, rồi chủ động bơm lệch xoay từ 0 đến 3 độ và dịch từ 0 đến 20 xen-ti-mét vào extrinsic. Đây là proxy cho cảm biến bị xê dịch trên giá đỡ, vì quan hệ hình học camera–LiDAR thay đổi theo cùng cách. Monitor được cài theo ý tưởng Levinson và Thrun: bước nhảy độ sâu LiDAR nên trùng với cạnh ảnh. Ta so điểm hiện tại với 52 extrinsic lân cận; nếu dưới 90 phần trăm hàng xóm kém hơn thì phát cảnh báo. Khi báo, bộ chỉnh tìm lại yaw, pitch và roll; phiên bản lõi chưa chỉnh tịnh tiến.

**Chuyển:** “Sau đây Khải sẽ cho thấy lỗi này tác động lên late fusion như thế nào.”

## 3. Hoàng Trung Khải — Benchmark, 1:25–2:10

**Màn hình:** bảng ở `REPORT_1PAGE.md § 3`.

> Baseline hiệu chuẩn đúng có lỗi chiếu bằng 0, giữ 100 phần trăm điểm và vẫn có range miss nền 3,3 phần trăm do vật xa thưa điểm. Chỉ với yaw 1 độ, điểm đã dời 14,5 pixel và vật xa còn 84,7 phần trăm điểm; pitch 1 độ làm vật xa chỉ còn 78,1 phần trăm. Trên drive giữ lại 0052 với 20 lệch ngẫu nhiên, bản paper phát hiện 95 phần trăm và còn 0,55 độ trung vị. Bản improved có trung vị tốt hơn, 0,44 độ, nhưng chỉ bắt 90 phần trăm và phân vị 90 tăng tới 1,75 độ. Vì vậy không thể chỉ nhìn trung vị để kết luận bản improved tốt hơn.

**Chuyển:** “Sơn sẽ giải thích ba trường hợp làm phần đuôi sai số tăng mạnh.”

## 4. Nguyễn Hoàng Sơn — Failure case, 2:10–2:52

**Màn hình:** bảng drift 1, 15, 19 trong `REPORT.md § 4`.

> Failure case chính nằm trên drive 0052: bản improved báo đúng nhưng hiệu chỉnh kẹt ở ba ca có roll khoảng âm 1,4 độ và pitch dương khoảng 1 độ. Sai số sau chỉnh vẫn là 2,19; 1,74 và 1,84 độ, trong khi bản paper đưa cùng các ca về 0,44 đến 0,75 độ. Giả thuyết của nhóm là tìm lần lượt từng trục mắc ở một đỉnh giả, nơi roll và pitch bù nhau; tổng lệch ban đầu cũng lớn hơn vùng quầng cạnh khoảng 1,2 độ. Ca này cho thấy cảnh báo đúng chưa đủ: extrinsic mới phải được kiểm tra lại trước khi áp dụng.

**Chuyển:** “Cuối cùng, Việt chốt quyết định kỹ thuật và hướng cải tiến.”

## 5. Hoàng Quốc Việt — Engineering decision, 2:52–3:40

**Màn hình:** `detection_check.png`, sau đó chuyển về phần Engineering decision trong `REPORT_1PAGE.md`.

> Nhóm chọn kiến trúc hai tầng. Monitor cạnh rẻ chạy liên tục; health dưới 90 phần trăm chỉ yêu cầu hiệu chuẩn lại, chưa tự động ghi đè extrinsic. Detection check bằng YOLO bắt 95 phần trăm lệch nhưng mù roll và còn lạc quan vì phía LiDAR dùng tracklet thật. SoftCorr chỉnh được cả dịch, còn khoảng 0,243 độ và 1,85 xen-ti-mét, nhưng mới đo 5 trên 20 ca và mất khoảng 87 giây mỗi ca trên GPU, nên chỉ phù hợp làm tầng nặng. Hướng sửa trực tiếp là tìm thô đến tinh trên roll–pitch và thêm ràng buộc mặt đường. Nhóm chỉ nhận cải tiến nếu ba ca lỗi đều dưới 1 độ, p90 không quá 0,70 độ và không tạo báo nhầm.

> Tóm lại, lệch khoảng 1 độ đã đủ ảnh hưởng late fusion; monitor có ích để báo sớm, nhưng cần cơ chế dự phòng và bước xác nhận độc lập trước khi dùng extrinsic mới.

## Ghi chú điều phối

- Người điều khiển mở sẵn năm tài liệu/hình theo thứ tự trên; không chạy Python, tải dữ liệu hoặc mở notebook trong lúc nói.
- Khi còn 30 giây, bỏ câu giải thích SoftCorr hội tụ; vẫn giữ ba ý kết: monitor liên tục, hiệu chuẩn tầng nặng, kiểm tra trước khi áp dụng.
- Nếu được thêm 30 giây, Khải nêu rõ “paper/repo báo 0,25° trên dữ liệu riêng; nhóm không đạt ngưỡng đó trên KITTI” để tách kết quả nguồn và kết quả nhóm.
- Số liệu dùng trong lời nói được đối chiếu từ [rotation_sweep.csv](../results/rotation_sweep.csv), [holdout_summary.json](../results/holdout/holdout_summary.json), [detection summary](../results/detection_check/summary.json) và [SoftCorr summary](../results/softcorr/holdout_summary.json).
