# Kiểm tra lệch hiệu chỉnh bằng phát hiện vật thể

Script: `detection_check.py` (chạy trong `.venv-yolo`). Kết quả: `results/detection_check/`
(`summary.json`, `sweep.csv`, `random_drifts_0052.csv`, `detection_check.png`, `log.txt`).

## Ý tưởng

Khi xe chạy thật, không có đám mây điểm "đúng" để so với ảnh. Thứ đáng tin là phát hiện vật thể:
LiDAR phát hiện vật thể 3D, chiếu hộp 3D vào ảnh bằng ngoại tham số hiện tại, rồi so với hộp 2D mà
camera tự phát hiện. Hiệu chỉnh đúng thì hai hộp trùng nhau; LiDAR xoay trên giá thì mọi hộp chiếu
trượt cùng một hướng, và độ lệch đó chính là độ trôi.

## Cách làm

1. **Phía camera (thật):** YOLO26n (COCO, Ultralytics) chạy trên `image_02`, ảnh 1280 px, giữ car,
   truck, bus, person, bicycle, motorcycle với độ tin cậy ≥ 0,4. Lấy mỗi 2 khung của 0048, 0005, 0052.
   Kết quả phát hiện được lưu đệm vì không phụ thuộc hiệu chỉnh.
2. **Phía LiDAR (thay thế):** hộp 3D tracklet của KITTI đóng vai bộ phát hiện LiDAR. Đây là **cận trên**:
   PointPillars hay CenterPoint thật sẽ bỏ sót vật và có nhiễu hộp riêng, nên số liệu thật sẽ kém hơn.
   8 góc hộp được chiếu thành hộp 2D (cắt theo ảnh; bỏ hộp có góc nằm sau camera hoặc nhỏ hơn 12 px).
3. **Ghép cặp:** ghép tham lam theo IoU (≥ 0,3, cùng nhóm lớp) **tại hiệu chỉnh đúng**, rồi giữ nguyên
   các cặp đó cho mọi ngoại tham số bị lệch. Như vậy tín hiệu là độ lệch chứ không phải chuyện ghép lại.
   Cặp tại hiệu chỉnh đúng: 0048 có 42, 0005 có 244, 0052 có 118.
4. **Chỉ số:** IoU trung bình, trung vị độ lệch tâm Δu, Δv (hộp chiếu trừ hộp camera, px), tỉ lệ cặp có
   IoU < 0,5.
5. **Hiệu chỉnh lại ở mức vật thể:** yaw ≈ −atan(Δu / f), pitch ≈ +atan(Δv / f), f = 721,5 px. Dấu được
   đo trên drift +1° đã biết ở drive phát triển (yaw +1° cho Δu = −13,6 px; pitch +1° cho Δv = +13,7 px).
   Bản lặp áp dụng công thức 4 lần, mỗi lần chiếu lại rồi đo lại, vì atan(Δu / f) chỉ đúng cho vật nằm
   giữa ảnh. **Roll không ước lượng được bằng cách này**: roll xoay hộp quanh tâm ảnh chứ không làm
   chúng trượt, nên trung vị độ lệch gần như không thấy nó.

Drive 0048 + 0005 là drive phát triển (dấu, độ lệch nền và ngưỡng chỉ đọc tại hiệu chỉnh đúng);
0052 là drive giữ lại để kiểm tra, giống `validate_holdout.py`.

**Hai luật cảnh báo, đều chỉ lấy từ phân bố tại hiệu chỉnh đúng:**
- `flag_iou`: IoU trung bình < IoU lúc hiệu chỉnh đúng trên cùng tuyến − 0,05. Luật này cần một giá trị
  tham chiếu ghi lại ngay sau khi hiệu chỉnh. Độ dao động bootstrap của IoU chỉ khoảng 0,017.
- `flag_offset`: |Δu − 0,19| hoặc |Δv − 0,82| > 3,0 px. Độ lệch nền và dung sai lấy từ drive phát triển;
  dung sai là max(3 px, 4 × độ lệch chuẩn bootstrap). Luật này dùng được ngay khi triển khai.

## Kết quả trên drive giữ lại 0052

IoU trung bình của các cặp cố định:

| Trục | 0° | 0,25° | 0,5° | 1° | 2° |
|---|---|---|---|---|---|
| yaw | 0,720 | 0,718 | 0,687 | 0,603 | 0,450 |
| pitch | 0,720 | 0,708 | 0,662 | 0,569 | 0,412 |
| roll | 0,720 | 0,722 | 0,719 | 0,703 | 0,662 |

Một độ yaw làm hộp trượt Δu = −14,7 px; một độ pitch cho Δv = +12,2 px. Roll 2° chỉ làm IoU giảm 0,06.

Bảng so sánh với bộ theo dõi căn cạnh trên cùng 0052 (`results/holdout/holdout_summary.json`):

| | Phát hiện vật thể (`flag_offset` / `flag_iou`) | Căn cạnh, bản bài báo | Căn cạnh, bản cải tiến |
|---|---|---|---|
| Báo động giả tại hiệu chỉnh đúng | không / không | không | không |
| Phát hiện drift một trục (chỉ xoay, 12 ca) | 75 % / 50 % | 50 % | 58 % |
| Phát hiện 20 drift ngẫu nhiên | **95 %** / 85 % | 95 % | 90 % |
| Sai lệch xoay còn lại, trung vị | 0,45° | 0,55° | 0,44° |
| Sai lệch xoay còn lại, phân vị 90 | 1,46° | 0,70° | 1,75° |
| Trung vị \|roll\| còn lại | 0,41° | 0,30° | 0,13° |
| Trung vị \|pitch\| / \|yaw\| còn lại | **0,09° / 0,10°** | 0,21° / 0,15° | 0,18° / 0,15° |
| Mức sàn tại hiệu chỉnh đúng | 0,11° | 0,18° | 0,21° |

Các con số của phương pháp phát hiện vật thể là bản lặp; bản một bước cho 0,45° / 1,45°. Luật `flag_offset`
chỉ bỏ sót drift số 4, là drift gần như chỉ có roll (−1,39°). Luật `flag_iou` bỏ sót thêm drift 2 và 14,
cũng là hai drift do roll chi phối.

**Drift thực tế** (roll 0,3°, pitch −0,8°, yaw 1,2°, tx 5 cm): IoU giảm từ 0,72 xuống 0,48, Δu = −17,1 px,
Δv = −11,3 px, cả hai luật đều cảnh báo. Một bước ước lượng yaw 1,36°, pitch −0,90°, còn lại 0,35°;
bản lặp còn lại 0,32°, gần như toàn bộ là phần roll 0,3° không nhìn thấy. Trên drive phát triển, bản lặp
còn lại 0,30°, còn bộ căn cạnh cải tiến còn 0,12° (`results/refinement.json`).

## Đánh đổi

- **Cần hai bộ phát hiện và đủ vật thể.** 0052 cho khoảng 3 cặp mỗi khung. Đường vắng thì không có tín
  hiệu, trong khi căn cạnh chỉ cần cảnh có cấu trúc.
- **Không thấy roll.** Hộp xoay quanh tâm ảnh nên trung vị Δu, Δv không đổi. Ở 0052, roll 1° vẫn cho
  Δv = −3 px và kéo pitch ước lượng lệch −0,24°, nhưng đó là do vật tập trung lệch một bên ảnh, không phải
  phép đo roll. Hồi quy Δv theo vị trí ngang có thể thấy roll nếu vật trải đều khắp bề ngang ảnh, nhưng
  ở đây chưa làm. Tịnh tiến (tx 5 cm) cũng gần như vô hình với hộp.
- **Nhiễu hộp của bộ phát hiện tạo mức sàn.** Độ lệch nền khác nhau giữa các drive (+0,2 / +0,8 px ở drive
  phát triển, −1,3 / −0,7 px ở 0052). Trừ độ lệch nền của drive phát triển không giúp gì
  (trung vị 0,48° so với 0,45°), nên mức sàn khoảng 0,1°. Bộ phát hiện LiDAR thật sẽ nâng mức sàn này lên.
- **Ghép cặp tại hiệu chỉnh đúng là lạc quan.** Khi triển khai, việc ghép diễn ra với ngoại tham số đang
  bị lệch. Ở 2° hộp trượt khoảng 28 px, nên vật nhỏ ở xa cần ghép theo khoảng cách tâm có cổng giới hạn.
- **Hệ thống end-to-end:** chỉ dùng được nếu mô hình có một đầu ra phát hiện vật thể (hộp 2D và 3D).
  Không có hộp thì không có gì để so.

## Khi nào kết hợp cả hai

Dùng kiểm tra bằng phát hiện vật thể làm **chuông báo luôn bật**: gần như miễn phí vì tận dụng đầu ra sẵn
có, nhạy với yaw và pitch từ 0,25 đến 0,5°, và sửa yaw cùng pitch trong một đến bốn bước. Khi chuông kêu,
hoặc định kỳ, chạy **tinh chỉnh căn cạnh** để bắt roll (bản cải tiến còn 0,13° roll) và một phần tịnh
tiến, xuất phát từ ngoại tham số đã được sửa yaw và pitch. Khi hai phương pháp không đồng ý, hoặc khi
đường vắng vật thể, giữ cảnh báo và xếp lịch hiệu chỉnh lại có mục tiêu.

Chạy lại: `.venv-yolo/Scripts/python detection_check.py`. Lần đầu tải `yolo26n.pt` về `.cache/ultralytics/`;
khi đã có kết quả phát hiện trong bộ đệm, cả lần chạy mất khoảng 4 giây.
