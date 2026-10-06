# Chọn ngưỡng kích hoạt hiệu chuẩn lại (issue #8)

**Lệnh:** `python threshold_sweep.py` (thêm `--data-dir D:/datasets/KITTI` nếu dữ liệu nằm ngoài repo), khoảng 60–75 s trên CPU.
**Kết quả:** [threshold.csv](../results/threshold/threshold.csv) · [threshold.png](../results/threshold/threshold.png) · [false_alarm_windows.csv](../results/threshold/false_alarm_windows.csv) · [threshold_log.txt](../results/threshold/threshold_log.txt)

## Cách đo

Monitor báo health = tỉ lệ trong 52 phương án hàng xóm có điểm căn cạnh thấp hơn extrinsic hiện tại. Luật đang dùng là "báo khi health < 0.90", nhưng con số 0.90 chưa được chọn bằng dữ liệu. Script quét ngưỡng từ 0.80 đến 0.98, bước 0.01, cho cả hai bản (paper, improved). Thiết lập monitor giữ nguyên như `validate_holdout.py`: bước hàng xóm 0.5° / 10 cm.

- **Báo động giả:** health ở **hiệu chuẩn đúng** của KITTI, đo trên các cửa sổ frame liên tiếp (dùng mọi frame, các cửa sổ chồng nhau một nửa) của drive 0048, 0005 và 0052. Có hai cỡ cửa sổ:
  - 12 frame: issue yêu cầu; 38 cửa sổ.
  - 24 frame: đúng số frame monitor gộp trong `run_benchmark.py` và `validate_holdout.py`; 16 cửa sổ. Drive 0048 chỉ có 22 frame nên không có cửa sổ 24.

  Thêm 3 mẫu 24 frame rải đều trên cả drive đã lưu trong `results/holdout/*_summary.json`: dev (0048 + 0005), 0052 và **0001**.
- **Độ nhạy:** health của 60 lệch ngẫu nhiên đã lưu, gồm 20 lệch × 3 bộ dev / 0052 / 0001, cùng seed 7, xoay ±1.5° mỗi trục, dịch ±5 cm. Thêm 45 lệch một trục có độ lớn ≥ 0.5° hoặc ≥ 10 cm, lấy từ `detection_sweep`. Các giá trị này lấy lại từ file, không tính lại.
- **Kiểm tra chéo:** ở ngưỡng 0.90, script cho lại đúng cột `triggered_*` trong CSV: paper / improved = 85 / 90 % trên dev, **95 / 90 % trên 0052**, 100 / 100 % trên 0001. Lệch một giá trị là script dừng với lỗi `assert`.

## Kết quả

Bản **improved** (cửa sổ 24 frame là cấu hình đang chạy):

| Ngưỡng | Báo giả, cửa sổ 12 frame | Báo giả, cửa sổ 24 frame | Mẫu gộp báo giả (trên 3) | Bắt 60 lệch ngẫu nhiên | Bắt lệch 1 trục |
|---|---|---|---|---|---|
| 0.85 | 0.0 % | 0.0 % | 0 | 93.3 % | 71.1 % |
| **0.90** | 5.3 % | **0.0 %** | **0** | **93.3 %** | **77.8 %** |
| 0.91 | 7.9 % | 0.0 % | **1 (drive 0001)** | 95.0 % | 80.0 % |
| 0.95 | 13.2 % | 0.0 % | 1 | 100.0 % | 84.4 % |

Bản **paper**:

| Ngưỡng | Báo giả, cửa sổ 12 frame | Báo giả, cửa sổ 24 frame | Bắt 60 lệch ngẫu nhiên |
|---|---|---|---|
| 0.80 | 7.9 % | 6.2 % | 81.7 % |
| 0.90 | 21.1 % | **18.8 %** | 93.3 % |
| 0.95 | 34.2 % | 37.5 % | 100.0 % |

**Nhóm quan sát được:**

1. Ở hiệu chuẩn đúng, health của bản improved trên cửa sổ 24 frame luôn là 0.981 (16/16 cửa sổ). Riêng mẫu gộp của drive 0001 chỉ được **0.904**. Đây là mẫu sạch thấp nhất, và nó chặn trần ngưỡng: từ 0.91 trở lên, drive 0001 bị báo nhầm.
2. Hạ ngưỡng từ 0.90 xuống 0.85 không giảm báo động giả nào (đã là 0 %) nhưng mất độ nhạy lệch một trục (77.8 → 71.1 %).
3. 4 lệch ngẫu nhiên bản improved bỏ sót ở 0.90 đều là lệch **chủ yếu theo roll**:
   - dev #4: roll −1.39°, health 0.904
   - dev #14 và 0052 #14: roll +1.15°, health 0.942
   - 0052 #2: roll −0.74°, health 0.923

   Phải nâng ngưỡng lên 0.95 mới bắt hết, và khi đó drive 0001 báo nhầm.
4. Bản paper báo nhầm trên **18.8 % cửa sổ 24 frame liên tiếp** ở ngưỡng 0.90, toàn bộ ở drive 0005 (health thấp nhất 0.788). Mẫu gộp rải đều trên drive (0.942) che mất điều này. Với bản paper, không có ngưỡng nào trong 0.80–0.98 cho 0 % báo giả.
5. Cửa sổ 12 frame nhiễu hơn hẳn: bản improved đã có 5.3 % báo giả ở 0.90, bản paper 21.1 %.

## Đề xuất

**Giữ ngưỡng 0.90, chỉ dùng bản improved, gộp ≥ 24 frame.** Lý do:

- 0.90 là ngưỡng cao nhất mà mọi mẫu sạch 24 frame đều không báo nhầm, kể cả drive 0001.
- Mọi ngưỡng thấp hơn đều kém hơn: không bớt báo giả mà mất độ nhạy.

Giới hạn cần nói rõ: biên an toàn chỉ còn **một hàng xóm**. Drive 0001 ở 47/52 = 0.904; chỉ cần một hàng xóm nữa vượt lên là thành 46/52 = 0.885, bị báo nhầm. Vì vậy:

- **Xác nhận hai lần:** chỉ kích hoạt khi hai cửa sổ 24 frame liên tiếp đều < 0.90. Một cửa sổ 0.90–0.95 thì chỉ ghi log ở mức "theo dõi".
- **Không dùng bản paper** cho monitor trên cửa sổ liên tiếp: 18.8 % báo nhầm.
- **Không gộp 12 frame:** báo giả từ 5.3 % trở lên.
- **Nâng ngưỡng không chữa được lệch roll.** Lỗ hổng này phải bịt bằng tín hiệu khác, như kiểm tra độ nghiêng mặt đường (issue #4) hoặc kiểm tra bằng box phát hiện vật thể ([DETECTION_CHECK.md](DETECTION_CHECK.md)).

**Giới hạn của phép thử:**

- Mẫu sạch ít: 16 cửa sổ 24 frame chồng nhau, chỉ từ 2 drive, cộng 3 mẫu gộp.
- Không có frame ban đêm hay mưa.
- Lệch là mô phỏng trên extrinsic.

Tỉ lệ báo giả thật ngoài đường cần được đo trên log dài hơn, ví dụ nhiều drive KITTI khác hoặc dữ liệu của chính xe.
