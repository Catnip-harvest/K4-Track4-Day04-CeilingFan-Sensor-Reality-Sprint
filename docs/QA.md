# Bộ câu hỏi–trả lời bảo vệ — T3 Camera–LiDAR calibration drift

Tài liệu cho issue #9. Các câu trả lời cố ý ngắn để dùng khi phản biện; con số đều trỏ tới artefact đã commit trong repo.

## 1. Vì sao bơm lỗi vào extrinsic là một proxy hợp lý cho cảm biến bị xê dịch thật?

Ảnh camera và scan LiDAR vẫn là dữ liệu KITTI thật; nhóm chỉ thay đổi phép biến đổi tương đối dùng để chiếu LiDAR sang camera, đúng đại lượng sẽ đổi khi giá đỡ bị xoay hoặc tịnh tiến. Proxy này đo được tác động hình học với đáp án biết trước, nhưng không mô phỏng rung động, rolling shutter hoặc thay đổi cảnh; xem định nghĩa phép lệch trong [geometry.py](../t3calib/geometry.py) và thiết kế benchmark trong [README.md](../README.md#2-run-the-benchmark).

## 2. Vì sao nhóm chọn late fusion “box camera + khoảng cách LiDAR” làm tính năng cần kiểm tra?

Đây là một đường truyền lỗi dễ giải thích: extrinsic sai làm điểm LiDAR rơi khỏi box đúng hoặc lấy nhầm điểm nền, từ đó khoảng cách vật bị sai dù từng sensor vẫn hoạt động. [rotation_sweep.csv](../results/rotation_sweep.csv) và [02_rotation_sweep.png](../results/figures/02_rotation_sweep.png) cùng đo lỗi chiếu, lượng điểm còn trong box và range miss nên nối được lỗi hiệu chuẩn với tác động chức năng.

## 3. Health và 52 hàng xóm có nghĩa gì?

Từ extrinsic hiện tại, nhóm thử 26 hàng xóm xoay trong lưới 3×3×3 và 26 hàng xóm dịch trong một lưới khác, bỏ tâm của mỗi lưới; health là phần trăm hàng xóm có điểm căn cạnh thấp hơn điểm hiện tại. Ngưỡng 90% nghĩa là nếu hơn 10% phương án gần đó tốt hơn thì extrinsic hiện tại không còn giống một cực đại cục bộ; thiết lập được ghi trong [holdout_summary.json](../results/holdout/holdout_summary.json).

## 4. Nhóm có tái lập được tuyên bố phát hiện từ 0,25° hoặc 10 cm của paper không?

Không: trên KITTI 0052, cả hai biến thể đều không báo ở roll, pitch hoặc yaw 0,25°, và độ nhạy còn phụ thuộc trục. Tuyên bố 0,25°/10 cm, dưới một giây và chính xác 100% thuộc dữ liệu riêng của Levinson & Thrun; kết quả nhóm nằm trong mảng `detection_sweep` của [holdout_summary.json](../results/holdout/holdout_summary.json), nên không được nhập hai nguồn số liệu làm một.

## 5. Vì sao monitor khó nhìn thấy roll?

Roll chủ yếu xoay ảnh quanh trục nhìn, nên điểm gần tâm ảnh dịch ít và score căn cạnh gộp toàn ảnh thay đổi yếu; yaw/pitch tạo dịch chuyển ngang/dọc rõ hơn. Drift 14 có roll 1,15° trên tổng 1,24° nhưng cả paper lẫn improved đều không báo, thể hiện trực tiếp trong [holdout_random_drifts.csv](../results/holdout/holdout_random_drifts.csv).

## 6. Vì sao bản improved tốt hơn ở trung vị nhưng lại kém tin cậy hơn trên 0052?

Bản improved còn 0,44° trung vị, tốt hơn 0,55° của paper, nhưng ba ca kẹt ở 1,74–2,19° đẩy phân vị 90 lên 1,75° thay vì 0,70°. Các ca 1, 15 và 19 đều có roll khoảng −1,4° cùng pitch dương khoảng 1°, gợi ý một đỉnh giả roll–pitch; số tổng hợp ở [holdout_summary.json](../results/holdout/holdout_summary.json), số từng ca ở [holdout_random_drifts.csv](../results/holdout/holdout_random_drifts.csv).

## 7. Tại sao pitch có thể làm association hỏng hơn yaw dù lỗi pixel nhỏ hơn?

Hộp xe thường rộng hơn cao, nên cùng độ dịch chuyển, dịch dọc do pitch dễ đẩy điểm ra khỏi hộp hơn dịch ngang do yaw. Ở 2°, pitch còn 41,2% điểm của vật xa trong khi yaw còn 61,5%, dù lỗi chiếu tương ứng là 26,0 và 28,9 px; xem [rotation_sweep.csv](../results/rotation_sweep.csv).

## 8. Vì sao baseline đã có range miss 3,3% dù hiệu chuẩn được coi là đúng?

Metric range dùng trung vị độ sâu LiDAR trong nửa giữa của box và đánh lỗi khi sai quá `max(1 m, 10%)` hoặc không có điểm, nên vật xa thưa điểm vẫn có thể trượt ngay tại hiệu chuẩn KITTI. Giá trị nền 3,3195% được lưu trong trường `kitti_calibration.range_miss_pct` của [refinement.json](../results/refinement.json); mọi điều kiện lỗi được so với cùng công thức này.

## 9. Vì sao bộ chỉnh cạnh chưa xử lý tịnh tiến?

Phiên bản hiện tại tối ưu yaw, pitch và roll, còn phần dịch được giữ nguyên; do đó ca thực tế có tx 5 cm vẫn còn sai 5 cm sau chỉnh dù residual xoay giảm. Bằng chứng nằm trong [refinement.json](../results/refinement.json): bản improved đưa xoay từ 1,47° xuống 0,116° nhưng `translation_error_cm` vẫn là 5,0 cm.

## 10. Detection consistency check có phải kiểm tra hoàn toàn thực tế không?

Không: phía camera dùng YOLO26n thật, nhưng phía LiDAR dùng tracklet 3D KITTI và ghép cặp tại hiệu chuẩn đúng, nên đây là cận trên so với một detector LiDAR có bỏ sót và nhiễu hộp. Trên 0052 luật độ lệch tâm bắt 95% lệch ngẫu nhiên nhưng gần như không sửa được roll; điều kiện và kết quả đầy đủ ở [DETECTION_CHECK.md](DETECTION_CHECK.md) và [summary.json](../results/detection_check/summary.json).

## 11. Tại sao không dùng detection check thay hẳn monitor căn cạnh?

Detection check cần đủ vật thể được phát hiện và phụ thuộc chất lượng detector/association; cảnh đường vắng hoặc lớp ngoài COCO sẽ làm tín hiệu yếu. Monitor cạnh không cần nhãn hay detector, còn detection check bổ sung tín hiệu yaw/pitch tốt hơn, vì vậy hai phương pháp phù hợp để kiểm tra chéo hơn là thay thế nhau; xem cách kết hợp trong [DETECTION_CHECK.md](DETECTION_CHECK.md#khi-nào-kết-hợp-cả-hai).

## 12. Kết quả SoftCorr 0,243° có đủ để kết luận tốt hơn không?

Chưa: SoftCorr mới chạy 5/20 lệch, chỉ một frame của 0052 cho mỗi lệch, nên mẫu quá nhỏ để kết luận độ tin cậy tổng quát. Nó cho residual xoay trung vị 0,243° và dịch 1,85 cm nhưng mất khoảng 87 giây/ca trên GPU; phạm vi, thời gian và cảnh báo được ghi trong [holdout_summary.json](../results/softcorr/holdout_summary.json).

## 13. Vì sao SoftCorr hội tụ gần như cùng một điểm từ nhiều khởi tạo?

Ngay cả khi bắt đầu từ hiệu chuẩn KITTI, SoftCorr cũng dời khoảng 0,241° và 1,81 cm; năm lệch đều hội tụ quanh điểm đó, nên 0,243° có thể phản ánh độ lệch cố định của cực đại score trên frame 0052/39 chứ không phải độ phân tán ngẫu nhiên. Đây mới là một giả thuyết; cần nhiều frame và drive khác để tách bias của mục tiêu SoftCorr khỏi sai khác của mốc KITTI, như dữ liệu `from_truth` trong [holdout_summary.json](../results/softcorr/holdout_summary.json).

## 14. Nhóm chọn ngưỡng cảnh báo 90% dựa trên điều gì, và có báo nhầm không?

Ngưỡng 90% là thiết lập cố định dùng trên dev rồi giữ nguyên khi sang 0052; tại hiệu chuẩn đúng, health là 100% và cả hai bản đều không báo nhầm trên hai tập đã đo. Tuy nhiên chưa có đủ cảnh bình thường đa dạng để ước lượng false-alarm rate ngoài KITTI, nên trước triển khai phải quét ngưỡng trên nhiều drive; số hiện tại ở [dev_summary.json](../results/holdout/dev_summary.json) và [holdout_summary.json](../results/holdout/holdout_summary.json).

## 15. Cải tiến nào xử lý trực tiếp failure case và tiêu chí nhận là gì?

Nhóm đề xuất tìm thô đến tinh trên roll–pitch kết hợp, rồi thêm ràng buộc pháp tuyến mặt đường để tránh việc hai trục bù nhau tại một đỉnh giả. Chạy lại `python validate_holdout.py --drives 0052 --label holdout`; chỉ nhận cải tiến nếu các drift 1, 15, 19 đều dưới 1°, p90 improved không quá 0,70° và hiệu chuẩn đúng vẫn không báo nhầm, với baseline hiện tại từ [holdout_random_drifts.csv](../results/holdout/holdout_random_drifts.csv) và [holdout_summary.json](../results/holdout/holdout_summary.json).
