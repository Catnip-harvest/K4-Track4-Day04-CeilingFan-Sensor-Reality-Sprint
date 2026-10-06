# Kiểm tra roll/pitch bằng mặt đường

## Cách đo

Chạy:

```bash
python ground_plane_check.py --drives 0048 0005 0052
```

Với mỗi scan, script giữ vùng có khả năng là mặt đường (`2 < x < 40 m`, `|y| < 10 m`,
`-3 < z < -0.5 m`), fit `z = ax + by + c` bằng `numpy.linalg.lstsq`, rồi loại outlier
lặp bằng ngưỡng MAD. Scan được mô phỏng nghiêng bằng nghịch đảo phần quay của `Drift` với
roll hoặc pitch thuộc `{0, 0.5, 1, 2}°`. Góc ước lượng được trừ góc mặt đường của frame gốc,
sau đó lấy trung vị qua các frame.

Kết quả đầy đủ nằm trong [`results/ground_plane/ground_plane.csv`](../results/ground_plane/ground_plane.csv)
và hình trong [`results/ground_plane/ground_plane.png`](../results/ground_plane/ground_plane.png).

## Kết quả

Bảng dưới là trung vị sai số tuyệt đối của đúng trục được tiêm, đơn vị độ. Dùng mỗi frame thứ hai:
11 frame ở 0048, 77 frame ở 0005 và 39 frame ở 0052.

| Drive | Trục | 0° | 0.5° | 1° | 2° |
|---|---|---:|---:|---:|---:|
| 0048 | roll | 0.000 | 0.014 | 0.031 | 0.121 |
| 0048 | pitch | 0.000 | 0.002 | 0.005 | 0.020 |
| 0005 | roll | 0.000 | 0.002 | 0.005 | 0.012 |
| 0005 | pitch | 0.000 | 0.019 | 0.042 | 0.068 |
| 0052 | roll | 0.000 | 0.006 | 0.007 | 0.010 |
| 0052 | pitch | 0.000 | 0.002 | 0.010 | 0.106 |

**Kết luận:** cách này bắt được roll 1° trên cả ba drive. Ước lượng trung vị lần lượt là
0.969°, 0.995° và 0.993°; sai số tuyệt đối trung vị lớn nhất chỉ 0.031°. Pitch 1° cũng được
ước lượng trong khoảng 0.966–0.997°.

## Giới hạn

- Mặt đường dốc hoặc nghiêng ngang làm đổi pháp tuyến. Phép thử mô phỏng ở đây trừ baseline của
  cùng frame nên cô lập được drift; hệ thống thực tế cần baseline đã biết hoặc gom nhiều đoạn đường
  để không báo nhầm độ dốc thành lỗi cảm biến.
- Lọc theo vùng hình học và least-squares vẫn có thể bị ảnh hưởng bởi lề đường, vật cản hoặc quá ít
  điểm đường; MAD chỉ giảm chứ không loại bỏ hoàn toàn rủi ro này.
- Pháp tuyến mặt đường chỉ đo hướng LiDAR so với xe/mặt đường. Nó không đo trực tiếp LiDAR so với
  camera, không bắt yaw hay tịnh tiến, nên phải dùng bổ sung cho health check camera–LiDAR hiện có.
