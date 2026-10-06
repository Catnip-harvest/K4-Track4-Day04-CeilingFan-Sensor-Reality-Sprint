# Association theo loại vật thể (issue #7)

**Lệnh:** `python per_class_analysis.py` (thêm `--data-dir D:/datasets/KITTI` nếu dữ liệu nằm ngoài repo), khoảng 20–40 s trên CPU.
**Kết quả:** [per_class.csv](../results/per_class/per_class.csv) · [per_class.png](../results/per_class/per_class.png) · [per_class_log.txt](../results/per_class/per_class_log.txt)

**Cách đo:** dùng cùng frame, cùng danh sách vật và cùng hàm `association_and_range` như `run_benchmark.py` (drive 0048 + 0005, mỗi frame thứ 2: 88 frame, 241 lượt vật). Danh sách vật được chia theo nhãn KITTI và theo khoảng cách (gần < 20 m, xa ≥ 20 m), rồi gọi lại hàm cho từng nhóm. Lệch: yaw và pitch 0.5 / 1 / 2°. Box 2D là box 3D thật chiếu lên ảnh, tức là một detector hoàn hảo.

## Số mẫu (đọc trước khi đọc số)

| Nhãn | Lượt vật | Số vật thật (tracklet) | Khoảng cách trung vị | Bề rộng box trung vị |
|---|---|---|---|---|
| Car | 88 (gần 54, xa 34) | 16 | 18.0 m | 117 px |
| Van | 84 (gần 29, xa 55) | 4 | 28.9 m | 100 px |
| Cyclist | 66 (đều gần) | **1** | 11.4 m | 73 px |
| Pedestrian | **3** | 2 | 10.8 m | 116 px |

Pedestrian chỉ có 3 lượt: **không kết luận**. Cyclist có 66 lượt nhưng tất cả là **một** người đi xe đạp trong drive 0005: số của Cyclist mô tả vật đó, không mô tả cả loại.

## Association retention (%): điểm LiDAR của vật còn nằm trong box 2D

| Nhãn | yaw 0.5° | yaw 1° | yaw 2° | pitch 0.5° | pitch 1° | pitch 2° |
|---|---|---|---|---|---|---|
| Car, gần | 99.6 | 97.3 | 89.8 | 100.0 | 99.1 | 86.7 |
| Car, xa | 96.6 | 88.1 | **64.9** | 94.2 | 71.8 | **38.4** |
| Van, gần (29 lượt, ít) | 98.3 | 93.4 | 82.4 | 100.0 | 98.8 | 87.1 |
| Van, xa | 92.8 | 82.6 | **59.4** | 98.1 | 82.0 | **43.0** |
| Cyclist (1 vật, gần) | 99.9 | 97.6 | 76.1 | 100.0 | 99.9 | 94.9 |

Baseline (không lệch) là 100 % cho mọi nhóm.

## Late-fusion range miss (%): khoảng cách lấy từ tâm box sai > max(1 m, 10 %)

| Nhãn | baseline | yaw 1° | yaw 2° | pitch 1° | pitch 2° |
|---|---|---|---|---|---|
| Car, xa | 23.5 | 0.0 | 50.0 | 29.4 | **94.1** |
| Van, xa | 0.0 | 1.8 | 41.8 | 1.8 | 40.0 |
| Cyclist (1 vật) | 0.0 | 16.7 | **100.0** | 0.0 | 0.0 |

Car xa có range miss nền 23.5 % ngay khi chưa lệch, nên chỉ so được độ tăng chứ không so được mức tuyệt đối.

## Kết luận

**Nhóm quan sát được:**

- Với cùng một độ lệch, **khoảng cách quyết định trước loại vật**. Ở pitch 2°, Car xa chỉ giữ 38.4 % điểm, còn Car gần giữ 86.7 %. Với Van, hai con số là 43.0 % và 87.1 %.
- Van trông tệ nhất khi gộp mọi khoảng cách (yaw 2°: 67.3 %, Car: 80.2 %), nhưng đó là vì Van trong hai drive này ở xa hơn (trung vị 28.9 m, Car 18.0 m). So cùng nhóm xa thì Van và Car gần nhau (yaw 2°: 59.4 % và 64.9 %).
- Vật hẹp hỏng ở **khoảng cách đo** trước khi hỏng ở association. Người đi xe đạp duy nhất (box rộng 73 px) vẫn giữ 97.6 % điểm ở yaw 1°, nhưng range miss đã là 16.7 %. Ở yaw 2°, vật vẫn giữ 76.1 % điểm, vậy mà range miss là 100 %. Pitch 2° gần như không ảnh hưởng tới vật này (94.9 %, range miss 0 %).

*Giả thuyết (chưa kiểm chứng riêng):* yaw 2° dời điểm khoảng 29 px theo chiều ngang. Vùng tâm box mà late fusion dùng (một nửa bề rộng, khoảng 37 px) khi đó phần lớn rơi ra nền phía sau, nên trung vị độ sâu lấy nhầm độ sâu của nền. Vật hẹp và cao như xe đạp thì nhạy với yaw (lệch ngang), ít nhạy với pitch (lệch dọc). Cần thêm Cyclist và Pedestrian từ drive khác để kiểm chứng.

**Hệ quả cho ngưỡng:** nếu tính năng phải giữ đúng khoảng cách cho vật hẹp như xe đạp hoặc người, mốc an toàn của yaw thấp hơn mốc 1° mà báo cáo chung dùng cho mọi vật.
