# Kiểm tra roll/pitch bằng mặt đường

## Cách đo

Lệnh chạy:

```bash
python ground_plane_check.py
```

Script dùng drive hold-out 0052 và đúng 20 drift sinh bởi `draw_random_drifts()` trong
`validate_holdout.py` (seed 7). Với mỗi scan, script giữ vùng có khả năng là mặt đường
(`2 < x < 40 m`, `|y| < 10 m`, `-3 < z < -0.5 m`), fit mặt phẳng bằng RANSAC NumPy rồi
least-squares trên inlier. Scan được xoay bằng nghịch đảo phần quay của `Drift`; tịnh tiến và yaw
không thể suy ra riêng từ một pháp tuyến mặt đường.

Roll/pitch được đo tương đối với mặt phẳng gốc của cùng frame, sau đó lấy trung vị qua 39 frame.
Pháp tuyến còn được chuyển từ hệ LiDAR sang hệ camera bằng phần quay của extrinsic KITTI; CSV lưu
ba thành phần pháp tuyến camera và góc thay đổi của nó. Quy tắc phát hiện roll dùng trong phép thử là
`|roll ước lượng| >= 0,5°`.

Kết quả: [ground_check.csv](../results/ground_check/ground_check.csv),
[ground_check_log.txt](../results/ground_check/ground_check_log.txt) và
[ground_check.png](../results/ground_check/ground_check.png).

## Kết quả

Trên 20 drift, trung vị sai số tuyệt đối là **0,0081° cho roll** và **0,0075° cho pitch**.
Một số ca đại diện:

| Drift | Roll tiêm → ước lượng | Pitch tiêm → ước lượng | Roll được phát hiện? |
|---:|---:|---:|:---:|
| 1 | −1,484° → −1,488° | +0,964° → +0,961° | Có |
| 6 | −0,391° → −0,384° | −1,489° → −1,487° | Không, vì roll < 0,5° |
| 12 | +1,122° → +1,106° | +0,487° → +0,474° | Có |
| **14** | **+1,152° → +1,144°** | **+0,425° → +0,432°** | **Có** |
| 19 | −1,484° → −1,488° | +0,759° → +0,760° | Có |

**Kết luận nghiệm thu:** ground check bắt được case 14 mà edge monitor bỏ qua. Case này có roll
+1,1522°, ground check ước lượng +1,1437° (sai số −0,0085°) và vượt ngưỡng phát hiện 0,5°.
Trong kết quả hold-out hiện có, cả monitor paper và improved đều không kích hoạt ở case 14
(`neighbours_worse = 0,9423`).

## Giới hạn

- Mặt đường dốc hoặc nghiêng ngang làm đổi pháp tuyến. Phép mô phỏng này trừ baseline của cùng frame
  để cô lập drift; hệ thống thật cần baseline đã biết hoặc gom nhiều đoạn đường để tránh báo độ dốc
  thành lỗi cảm biến.
- RANSAC vẫn có thể fit nhầm lề đường hoặc vật cản khi vùng quan sát có quá ít điểm mặt đường.
- Pháp tuyến chỉ đo LiDAR so với xe/mặt đường, không đo trực tiếp LiDAR so với camera. Nó không quan
  sát được yaw hay tịnh tiến, nên chỉ là ràng buộc bổ sung cho health monitor camera–LiDAR.
