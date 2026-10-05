# Báo cáo cá nhân — Hoàng Trung Khải

**MSSV:** 2A202602947 · **Nhóm:** Ceiling Fan ([TEAMMATES.md](../TEAMMATES.md)) · **Repo:** https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint

**Phần tôi phụ trách:** kiểm thử hình học và metric của `t3calib` (issue [#5](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/5)); kiểm định độc lập trên drive KITTI raw 0001 (issue [#6](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/6)).

## 1. Problem — vấn đề

Tôi xét xe ADAS dùng camera màu trước cam 2 và LiDAR Velodyne HDL-64E. Khi gá lắp bị xê dịch, ma trận extrinsic camera–LiDAR không còn đúng: điểm LiDAR bị chiếu lệch khỏi vật thể trong ảnh, khiến bước late fusion lấy nhầm khoảng cách. Vì thế ngoài chất lượng benchmark, các phép biến đổi tọa độ và metric cần có kiểm thử nhỏ, xác định được kết quả đúng mà không cần tải bộ dữ liệu lớn.

## 2. Method — cách làm

- **Nguồn và phiên bản:** ý tưởng căn cạnh theo J. Levinson và S. Thrun, “Automatic Online Calibration of Cameras and Lasers”, RSS 2013 ([paper](https://www.roboticsproceedings.org/rss09/p29.pdf)). Phần triển khai của nhóm nằm trong [`t3calib/`](../t3calib/) trên nhánh `main`; mã nguồn tác giả không được dùng.
- **Input → output:** `Drift` tạo phép xoay/tịnh tiến 4×4, ghép với extrinsic; `project` chiếu điểm LiDAR ra pixel; metric tính sai số chiếu, tỉ lệ giữ điểm trong box và tỉ lệ sai khoảng cách. Tôi bổ sung [`tests/test_t3calib.py`](../tests/test_t3calib.py) dùng dữ liệu tổng hợp để kiểm tra nghịch đảo drift, phép chiếu đối chiếu bằng tay và các metric ở trường hợp không drift.
- **Kiểm tra trên dữ liệu:** KITTI raw thuộc ngày `2011_09_26`; drive 0001 có 108 frame, lấy mẫu cách 2 frame. Lệnh chạy là `python3 get_data.py --drives 0001`, rồi `python3 validate_holdout.py --drives 0001 --label holdout_0001`. Bộ kiểm định dùng 24 frame monitor, 20 drift ngẫu nhiên cùng seed 7 và giữ nguyên cấu hình của benchmark nhóm.

## 3. Benchmark — kết quả

Kiểm thử issue #5 là kiểm thử tổng hợp, không dùng KITTI. Lệnh tái lập từ gốc repo: `python3 -m pytest -q` (3 passed; dùng Python 3.10 trong workspace này). Ba kiểm thử đặt các kỳ vọng cụ thể: ma trận extrinsic phục hồi sau khi nhân drift rồi nghịch đảo; pixel khớp phép nhân ma trận viết tay; không drift cho lỗi chiếu 0 px, association 100% và range miss 0%. Đây là kiểm tra tính đúng đắn cục bộ, không thay thế benchmark trên dữ liệu đường thật.

Kết quả bổ sung drive 0001 nằm trong [`results/holdout/holdout_0001_summary.json`](../results/holdout/holdout_0001_summary.json), tạo bằng `python3 validate_holdout.py --drives 0001 --label holdout_0001`. Bảng gốc của kiểm định trước đó trên drive 0052 là [`results/holdout/holdout_summary.json`](../results/holdout/holdout_summary.json), từ `python validate_holdout.py --drives 0052 --label holdout`.

**Nhóm quan sát được** trên drive 0001, monitor không báo nhầm ở hiệu chuẩn KITTI; bản paper phát hiện 100% drift ngẫu nhiên, improved cũng 100%, còn detection sweep một trục lần lượt đạt 67% và 86%. Sai số xoay còn lại trung vị của paper/improved là 2.02°/4.77°, P90 là 2.06°/7.81°. Trên drive 0052, các số tương ứng là 95%/90% phát hiện drift ngẫu nhiên, median 0.55°/0.44° và P90 0.70°/1.75%. Hai tập cho thấy kết quả refine phụ thuộc drive; đây là số tự đo của nhóm. Lưu ý môi trường chạy khác nhau: 0001 dùng Python 3.10.12 / NumPy 1.23.5, còn lần chạy 0052 dùng Python 3.13.14 / NumPy 2.2.6, nên so sánh giữa drive không cô lập hoàn toàn ảnh hưởng phiên bản thư viện.

**Paper/repo cho biết** Levinson–Thrun báo cáo phát hiện lệch lớn hơn 0.25° hoặc 10 cm trong vòng một giây với độ chính xác 100% trên dữ liệu thí nghiệm của họ; cấu hình KITTI của nhóm là triển khai và đánh giá riêng, không tái lập con số đó.

## 4. Failure case — đỉnh điểm số lệch trên drive 0001

Trong [bảng random drift của drive 0001](../results/holdout/holdout_0001_random_drifts.csv), ca 11 được monitor đánh dấu ở cả hai bản. Sau refine, sai số xoay còn lại là 2.05° với paper và 8.53° với improved; đây là một ca hiệu chỉnh improved đi xa khỏi đáp án KITTI dù đã kích hoạt. Cùng drive, đỉnh score của improved lệch KITTI 3.39°. **Giả thuyết (chưa kiểm chứng riêng):** cảnh của drive 0001 tạo đỉnh căn cạnh thiên lệch, khiến tối ưu score cục bộ ưu tiên một extrinsic sai.

## 5. Engineering decision — quyết định kỹ thuật

- **Log cần giữ:** drive/frame, extrinsic đầu vào và đầu ra, health, điểm trước/sau refine, drift đã tiêm trong benchmark và sai số xoay/tịnh tiến còn lại. Khi chạy kiểm định cần lưu cả lệnh, seed và nhãn đầu ra để so sánh được giữa drive.
- **Dự phòng:** nếu health kích hoạt nhưng refine không cải thiện đủ, không áp dụng extrinsic mới tự động; giữ cấu hình gần nhất đã xác thực và đánh dấu cần kiểm tra lại.
- **Cải tiến gắn với failure case:** kiểm tra tính nhất quán của đỉnh score trên các nhóm frame khác nhau, rồi chỉ chấp nhận hiệu chỉnh nếu các nhóm frame đồng thuận; nếu không thì giữ extrinsic trước đó và yêu cầu thu thêm frame. Đo lại trên 20 drift cùng seed 7: mục tiêu ca 11 dưới 1° sai số xoay, P90 improved không quá 2.06° (mức paper trên drive này), và không báo nhầm tại hiệu chuẩn đúng.
- **Issue #5:** đã thêm ba kiểm thử tổng hợp vào [`tests/test_t3calib.py`](../tests/test_t3calib.py); `python3 -m pytest -q` cho kết quả 3 passed. **Issue #6:** đã kiểm định drive 0001; lệnh `python3 validate_holdout.py --drives 0001 --label holdout_0001`. Số liệu hai bản, gồm detection rate và median/P90 residual rotation, nằm trong [`holdout_0001_summary.json`](../results/holdout/holdout_0001_summary.json); kết quả này được so sánh với [`holdout_summary.json`](../results/holdout/holdout_summary.json) của drive 0052.
