# Báo cáo cá nhân — Nguyễn Minh Dương

**MSSV:** 2A202602920 · **Nhóm:** Ceiling Fan ([TEAMMATES.md](../TEAMMATES.md)) · **Repo:** https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint

**Phần tôi phụ trách:** đóng gói môi trường và pipeline chạy một lệnh ([requirements.txt](../requirements.txt),
[run_all.ps1](../run_all.ps1), [run_all.sh](../run_all.sh), issue #3); kiểm tra độ nghiêng LiDAR bằng
mặt đường ([ground_plane_check.py](../ground_plane_check.py), issue #4).

## 1. Problem — vấn đề

Nền tảng là xe ADAS dùng camera màu phía trước và LiDAR Velodyne HDL-64E. Tính năng được kiểm tra là
late fusion camera–LiDAR: box 2D từ camera nhận khoảng cách từ các điểm LiDAR chiếu vào box. Sau rung
lắc hoặc va chạm vào giá đỡ, extrinsic có thể lệch dù ảnh và point cloud riêng lẻ vẫn trông bình
thường. Điểm LiDAR khi đó rơi sai vị trí trên ảnh, làm hệ thống ghép nhầm vật hoặc đo sai khoảng cách.

Một khó khăn kỹ thuật khác là khả năng tái lập: nếu phiên bản thư viện và thứ tự lệnh không cố định,
người mới clone repo có thể không tạo lại được số liệu. Vì vậy phần việc của tôi vừa bổ sung cách chạy
lặp lại toàn pipeline, vừa xử lý failure case mà monitor căn cạnh gần như mù với roll.

## 2. Method — cách làm

- **Monitor chính của nhóm:** cài lại ý tưởng targetless của Levinson và Thrun, “Automatic Online
  Calibration of Cameras and Lasers”, RSS 2013
  (https://www.roboticsproceedings.org/rss09/p29.pdf). Input là ảnh, scan LiDAR và extrinsic hiện tại;
  output là health score, cảnh báo và extrinsic được chỉnh xoay.
- **Ground-plane check của tôi:** lấy điểm có khả năng thuộc mặt đường trong vùng `2 < x < 40 m`,
  `|y| < 10 m`, `-3 < z < -0,5 m`; fit mặt phẳng bằng RANSAC NumPy và least-squares trên inlier.
  Pháp tuyến cho biết roll/pitch của LiDAR. Tôi chuyển pháp tuyến sang hệ camera bằng phần quay của
  extrinsic KITTI và đo tương đối với mặt phẳng gốc của cùng frame.
- **Mô phỏng:** dùng đúng 20 drift seed 7 từ `validate_holdout.py`; scan được xoay bằng nghịch đảo
  phần quay của `Drift`. Tịnh tiến không làm đổi pháp tuyến, còn yaw không thể suy ra riêng từ một
  mặt phẳng.
- **Giả định:** hiệu chuẩn KITTI là đáp án; mặt đường chiếm ưu thế trong ROI; độ dốc của cùng frame
  không đổi giữa baseline và scan mô phỏng. Đây là phép thử lỗi có kiểm soát, chưa phải drift vật lý
  được đo bằng bảng hiệu chuẩn ngoài hiện trường.
- **Lệnh:** `python get_data.py --drives 0048 0005 0052`, sau đó `python ground_plane_check.py`.
  Toàn benchmark có thể chạy bằng `.\run_all.ps1` trên Windows hoặc `./run_all.sh` trên Linux/macOS.

## 3. Benchmark — kết quả

Dữ liệu cảm biến là KITTI raw thật; lỗi extrinsic là mô phỏng. Benchmark chung dùng drive 0048 và
0005 để phát triển, drive 0052 để hold-out. Metric chung gồm lỗi chiếu, association retention, range
miss và residual rotation. Số gốc của nhóm nằm trong [rotation_sweep.csv](../results/rotation_sweep.csv)
và [holdout_summary.json](../results/holdout/holdout_summary.json), được tạo bởi `python run_benchmark.py`
và `python validate_holdout.py --drives 0052 --label holdout`.

| Điều kiện | Kết quả chính |
|---|---|
| Baseline KITTI | 0 px lỗi chiếu; giữ 100% điểm; range miss nền 3,3% |
| Yaw 1° | 14,5 px; vật xa giữ 84,7%; range miss 5,4% |
| Pitch 1° | 13,0 px; vật xa giữ 78,1%; range miss 4,6% |
| Hold-out 0052, paper | phát hiện 95%; residual trung vị 0,55°, p90 0,70° |
| Hold-out 0052, improved | phát hiện 90%; residual trung vị 0,44°, p90 1,75° |

Kết quả phần của tôi được tạo bằng `python ground_plane_check.py` và lưu tại
[ground_check.csv](../results/ground_check/ground_check.csv),
[ground_check_log.txt](../results/ground_check/ground_check_log.txt) cùng hình
[ground_check.png](../results/ground_check/ground_check.png). Trên 20 drift của drive 0052, sai số
tuyệt đối trung vị là 0,0081° cho roll và 0,0075° cho pitch.

**Nhóm quan sát được** edge monitor bắt yaw và pitch từ khoảng 0,5° nhưng không bắt roll tới 2° trong
single-axis sweep; riêng random drift 14 có roll +1,1522° mà cả hai biến thể đều không cảnh báo
(`neighbours_worse = 0,9423`). Ground-plane check ước lượng drift này thành +1,1437°, sai số −0,0085°,
nên phát hiện được với ngưỡng `|roll| >= 0,5°`.

**Paper/repo cho biết** phương pháp căn cạnh gốc phát hiện sai lệch trên 0,25° hoặc 10 cm trong vòng
một giây với độ chính xác 100% trên dữ liệu của tác giả. Nhóm không dùng con số đó làm kết quả KITTI;
trên hold-out của nhóm, độ nhạy phụ thuộc trục và roll là điểm yếu rõ ràng.

## 4. Failure case — monitor bỏ sót roll

Failure case tôi chọn là drift 14 trong [holdout_random_drifts.csv](../results/holdout/holdout_random_drifts.csv).
Lệch được tiêm gồm roll +1,1522°, pitch +0,4247° và yaw +0,2091°. Health của cả paper và improved
đều là 94,23%, cao hơn ngưỡng cảnh báo 90%, nên monitor coi extrinsic này là khỏe. Sau refinement,
residual của paper vẫn 0,7174° và improved còn 0,8766°.

*Giả thuyết:* roll chủ yếu làm các điểm xoay quanh hướng nhìn về phía trước; trong nhiều vùng ảnh,
độ dịch vuông góc với cạnh nhỏ hơn yaw hoặc pitch, nên edge score ít thay đổi. Ngược lại, pháp tuyến
mặt đường phản ứng trực tiếp với roll: RANSAC cho +1,1437° và đánh dấu phát hiện. Kết quả này cho thấy
hai tín hiệu bổ sung cho nhau, nhưng chưa chứng minh ground check sẽ ổn định trên đường dốc thật.

## 5. Engineering decision — quyết định kỹ thuật

- **Đóng gói:** pin đúng các phiên bản đã tạo kết quả trong `requirements.txt`; runner dừng ngay khi
  bất kỳ bước tải dữ liệu, benchmark, hold-out hoặc demo trả mã lỗi khác 0.
- **Luật vận hành:** chạy edge monitor cùng ground-plane check. Nếu một trong hai báo lệch, chưa ghi
  đè extrinsic ngay; chuyển fusion sang chế độ giảm tin cậy và thu một cửa sổ dữ liệu mới để xác nhận.
- **Dự phòng:** khi extrinsic chưa đáng tin, giảm trọng số late fusion, giữ nhãn camera và ưu tiên
  khoảng cách LiDAR thuần thay vì tiếp tục ghép bằng ma trận nghi ngờ.
- **Log cần giữ:** health score, roll/pitch từ mặt đường, số inlier RANSAC, pháp tuyến trong hệ camera,
  extrinsic cũ/mới, tốc độ xe và sự kiện xóc IMU; lưu ảnh và scan quanh mỗi cảnh báo.
- **Cải tiến gắn với failure case:** thêm ground-plane check làm ràng buộc roll/pitch cho monitor căn
  cạnh. Tiêu chí kiểm chứng là drift 14 phải được phát hiện, median lỗi roll/pitch dưới 0,1° trên 20
  drift, trong khi baseline trên đường phẳng không báo nhầm. Kết quả hiện tại đạt tiêu chí mô phỏng,
  nhưng bước tiếp theo phải kiểm tra đường dốc/nghiêng và dùng IMU hoặc nhiều đoạn đường để phân biệt
  độ dốc thật với cảm biến bị lệch.
