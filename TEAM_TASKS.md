# Việc còn lại hôm nay — bốn thành viên

Repo: https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint · Thành viên: [TEAMMATES.md](TEAMMATES.md) · Mẫu báo cáo: [reports/HoangQuocViet.md](reports/HoangQuocViet.md), hướng dẫn: [reports/README.md](reports/README.md)

> **Tối thiểu bắt buộc** (mỗi người phải xong hôm nay)
>
> 1. **MSSV trong [TEAMMATES.md](TEAMMATES.md)**, chỉ sửa dòng của mình. Minh Dương, Khải, Sơn đã điền; **còn Dương Dương**.
> 2. **Báo cáo cá nhân `reports/<Ten>.md`** theo năm mục của đề, bằng lời của mình.
> 3. **Nộp VLearn:** URL repo + link tới tệp báo cáo của mình.
>
> Các issue GitHub bên dưới là **đóng góp thêm**: làm sau khi xong ba việc trên.

## Quy trình chung

- Cả bốn người đã là collaborator có quyền push (không cần fork): `minhduong814` (Nguyễn Minh Dương), `khaihoang004` (Hoàng Trung Khải), `sown101` (Nguyễn Hoàng Sơn), `duongduong1606` (Dương Dương).
- Mỗi phần việc một nhánh và một PR nhỏ vào `main`:

  ```bash
  git clone https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint   # lần đầu
  git switch main && git pull                      # luôn pull trước
  git switch -c <ten>/<viec>                       # ví dụ duong/report, khai/issue-5
  # ... sửa, rồi:
  git status                                       # không được có data/ hay *.zip
  git add <tệp của mình> && git commit -m "..." && git push -u origin <ten>/<viec>
  git fetch origin && git rebase origin/main       # trước khi merge, nếu main đã đổi
  ```

  Mở PR vào `main` trên GitHub. Không đẩy thay đổi lớn thẳng lên `main`.
- Cài đặt và dữ liệu (CPU là đủ): `pip install numpy matplotlib pillow opencv-python pandas`, rồi `python get_data.py` (drive 0048 + 0005; thêm `python get_data.py --drives 0052` nếu cần drive kiểm định).
- Không commit `data/`, `*.zip` hay trọng số mô hình. Không sửa tệp của người khác: báo cáo của người khác, `REPORT.md`, `REPORT_1PAGE.md`, `SUBMISSION.md`, `PHAN_BIEN.md`, các tệp đã có trong `results/`. Ưu tiên thêm tệp mới; cần sửa tệp chung thì ghi lý do trong mô tả PR.
- **Báo cáo cá nhân cần có:** năm mục Problem / Method / Benchmark / Failure case / Engineering decision; trích ít nhất một bảng hoặc hình chung bằng link tương đối kèm đúng lệnh đã tạo ra nó; câu "Nhóm quan sát được …" (số tự đo) tách khỏi câu "Paper/repo cho biết …" (số của nguồn); giả thuyết ghi rõ là giả thuyết; đúng một failure case được phân tích; một cải tiến gắn với failure case đó và metric/log sẽ kiểm chứng nó; nhắc phần issue mình đã làm (hoặc ghi thật là chưa xong).
- **Nộp VLearn:** sau khi PR báo cáo đã merge, nộp bài Day 4 với (1) URL repo ở trên, (2) link `https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/blob/main/reports/<Ten>.md`. Bấm nộp, mở lại bài nộp để xem đã lưu, rồi mở hai link trong cửa sổ ẩn danh: ra 404 nghĩa là repo đang private, báo nhóm trước khi hết hạn.

## 1. Nguyễn Minh Dương (`minhduong814`)

- (a) MSSV đã điền; chỉ kiểm tra lại.
- (b) Issue [#3](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/3) requirements.txt + script chạy một lệnh; [#4](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/4) kiểm tra độ nghiêng mặt đường để bắt lệch roll/pitch.
- (c) `reports/NguyenMinhDuong.md`; failure case gợi ý: monitor mù roll, gắn với #4.
- (d) Ba PR: `duong/report`, `duong/issue-3`, `duong/issue-4`. (e) Nộp VLearn như trên.

**Prompt cho agent:**

```text
Repo: https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint (tôi là minhduong814, có quyền push).
Đề T3: lệch hiệu chuẩn camera–LiDAR trên xe ADAS, KITTI raw. Đọc trước: README.md, REPORT_1PAGE.md, REPORT.md, SUBMISSION.md (mục Bước 1, Bước 2), TEAM_TASKS.md, reports/README.md, reports/HoangQuocViet.md (chỉ làm mẫu, không chép).
Cài: pip install numpy matplotlib pillow opencv-python pandas ; python get_data.py ; python get_data.py --drives 0052
Quy trình: git switch main && git pull; mỗi việc một nhánh (duong/report, duong/issue-3, duong/issue-4) và một PR nhỏ vào main; rebase lên origin/main trước khi merge.
Ràng buộc: không commit data/, *.zip, trọng số; không sửa tệp của người khác (REPORT*.md, SUBMISSION.md, PHAN_BIEN.md, báo cáo người khác, results/ đã có); ưu tiên tệp mới; viết tiếng Việt có dấu.
Việc 1 (bắt buộc): viết reports/NguyenMinhDuong.md, 5 mục Problem / Method / Benchmark / Failure case / Engineering decision bằng lời của tôi; trích bảng/hình chung bằng link tương đối kèm đúng lệnh (ví dụ ../results/holdout/holdout_summary.json từ `python validate_holdout.py --drives 0052 --label holdout`); có câu "Nhóm quan sát được …" và câu "Paper/repo cho biết …" tách riêng; một failure case (gợi ý: monitor không bắt được roll tới 2°, REPORT.md § 4); một cải tiến gắn với nó và metric/log kiểm chứng; nhắc kết quả issue #3, #4.
Việc 2 (#3): thêm requirements.txt ghim đúng phiên bản trong README.md (numpy 2.2.6, pandas 3.0.5, matplotlib 3.11.1, Pillow 12.3.0, opencv-python 5.0.0) và một script chạy một lệnh (get_data → run_benchmark → validate_holdout) cho Git Bash và PowerShell.
Việc 3 (#4): script mới ước lượng roll/pitch từ mặt đường LiDAR (RANSAC) chuyển sang hệ camera bằng extrinsic đang dùng; chạy trên 20 lệch của validate_holdout.py (seed 7) ở drive 0052; ghi CSV + log vào thư mục mới results/ground_check/.
Nghiệm thu: PR báo cáo có đủ 5 mục, ≥ 1 link tới bảng/hình chung + lệnh, hai câu "Nhóm quan sát được"/"Paper/repo cho biết", 1 failure case, 1 cải tiến có metric; `pip install -r requirements.txt` chạy sạch trong venv mới; ground_check báo được ca roll 14 (roll 1.15°) hay không, ghi rõ con số; git diff không có data/ hay .zip.
```

## 2. Hoàng Trung Khải (`khaihoang004`)

- (a) MSSV đã điền; chỉ kiểm tra lại.
- (b) Issue [#5](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/5) unit test cho t3calib bằng pytest; [#6](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/6) kiểm định thêm trên drive 0001.
- (c) `reports/HoangTrungKhai.md`; failure case gợi ý: kết quả trên 0001 so với 0052 (đỉnh giả roll–pitch có lặp lại không).
- (d) Ba PR: `khai/report`, `khai/issue-5`, `khai/issue-6`. (e) Nộp VLearn như trên.

**Prompt cho agent:**

```text
Repo: https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint (tôi là khaihoang004, có quyền push).
Đề T3: lệch hiệu chuẩn camera–LiDAR trên xe ADAS, KITTI raw. Đọc trước: README.md, REPORT_1PAGE.md, REPORT.md, SUBMISSION.md (mục Bước 1, Bước 2), TEAM_TASKS.md, reports/README.md, reports/HoangQuocViet.md (chỉ làm mẫu, không chép), t3calib/geometry.py, t3calib/metrics.py, validate_holdout.py.
Cài: pip install numpy matplotlib pillow opencv-python pandas pytest ; python get_data.py ; python get_data.py --drives 0001
Quy trình: git switch main && git pull; mỗi việc một nhánh (khai/report, khai/issue-5, khai/issue-6) và một PR nhỏ vào main; rebase lên origin/main trước khi merge.
Ràng buộc: không commit data/, *.zip; không sửa tệp của người khác (REPORT*.md, SUBMISSION.md, PHAN_BIEN.md, báo cáo người khác, results/ đã có); không đổi code t3calib/ trong PR test (chỉ thêm tests/); viết tiếng Việt có dấu.
Việc 1 (bắt buộc): viết reports/HoangTrungKhai.md, 5 mục Problem / Method / Benchmark / Failure case / Engineering decision bằng lời của tôi; trích bảng/hình chung bằng link tương đối kèm đúng lệnh; có câu "Nhóm quan sát được …" và câu "Paper/repo cho biết …" tách riêng; một failure case; một cải tiến gắn với nó và metric/log kiểm chứng; nhắc kết quả issue #5, #6.
Việc 2 (#5): thêm tests/ với pytest cho t3calib: Drift xoay/dịch rồi đảo ngược về đúng, project khớp với phép nhân ma trận tay, metrics trả 0 px / 100 % khi không lệch. Dùng dữ liệu tổng hợp, không cần data/.
Việc 3 (#6): `python validate_holdout.py --drives 0001 --label holdout0001` (nhãn mới, không ghi đè holdout_*); commit chỉ results/holdout/holdout0001_*; so sánh với results/holdout/holdout_summary.json.
Nghiệm thu: PR báo cáo đủ 5 mục, ≥ 1 link tới bảng/hình chung + lệnh, hai câu "Nhóm quan sát được"/"Paper/repo cho biết", 1 failure case, 1 cải tiến có metric; `python -m pytest -q` xanh dưới 10 s trên máy không có data/; PR #6 ghi tỉ lệ phát hiện và sai số xoay còn lại (trung vị, phân vị 90) của cả hai bản trên 0001; git diff không có data/ hay .zip.
```

## 3. Nguyễn Hoàng Sơn (`sown101`)

- (a) MSSV đã điền; chỉ kiểm tra lại.
- (b) Issue [#7](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/7) association theo loại vật thể; [#8](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/8) chọn ngưỡng kích hoạt hiệu chuẩn lại (độ nhạy và báo động giả).
- (c) `reports/NguyenHoangSon.md`; failure case gợi ý: vật xa và loại vật nhỏ mất điểm trước.
- (d) Ba PR: `son/report`, `son/issue-7`, `son/issue-8`. (e) Nộp VLearn như trên.

**Prompt cho agent:**

```text
Repo: https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint (tôi là sown101, có quyền push).
Đề T3: lệch hiệu chuẩn camera–LiDAR trên xe ADAS, KITTI raw. Đọc trước: README.md, REPORT_1PAGE.md, REPORT.md, SUBMISSION.md (mục Bước 1, Bước 2), TEAM_TASKS.md, reports/README.md, reports/HoangQuocViet.md (chỉ làm mẫu, không chép), run_benchmark.py, t3calib/metrics.py, results/holdout/*_random_drifts.csv.
Cài: pip install numpy matplotlib pillow opencv-python pandas ; python get_data.py ; python get_data.py --drives 0052
Quy trình: git switch main && git pull; mỗi việc một nhánh (son/report, son/issue-7, son/issue-8) và một PR nhỏ vào main; rebase lên origin/main trước khi merge.
Ràng buộc: không commit data/, *.zip; không sửa tệp của người khác (REPORT*.md, SUBMISSION.md, PHAN_BIEN.md, báo cáo người khác, results/ đã có); thêm script mới thay vì sửa run_benchmark.py; viết tiếng Việt có dấu.
Việc 1 (bắt buộc): viết reports/NguyenHoangSon.md, 5 mục Problem / Method / Benchmark / Failure case / Engineering decision bằng lời của tôi; trích bảng/hình chung bằng link tương đối kèm đúng lệnh (ví dụ ../results/rotation_sweep.csv từ `python run_benchmark.py`); có câu "Nhóm quan sát được …" và câu "Paper/repo cho biết …" tách riêng; một failure case; một cải tiến gắn với nó và metric/log kiểm chứng; nhắc kết quả issue #7, #8.
Việc 2 (#7): script mới tính association retention và range miss theo loại vật (Car, Van, Cyclist, Pedestrian) cho yaw/pitch 0–2°; ghi CSV + hình vào results/association_by_class/; ghi rõ Pedestrian chỉ có 3 lượt nên không kết luận được.
Việc 3 (#8): từ health trong results/holdout/dev_random_drifts.csv, holdout_random_drifts.csv và detection_sweep trong *_summary.json, quét ngưỡng 0.80–0.98; bảng tỉ lệ phát hiện và báo động giả cho cả hai bản (paper, improved); đề xuất một ngưỡng và lý do. Ghi vào results/threshold_sweep/.
Nghiệm thu: PR báo cáo đủ 5 mục, ≥ 1 link tới bảng/hình chung + lệnh, hai câu "Nhóm quan sát được"/"Paper/repo cho biết", 1 failure case, 1 cải tiến có metric; hai script chạy lại được bằng một lệnh ghi trong mô tả PR; ngưỡng 0.90 hiện tại cho lại đúng 95 % / 90 % trên 0052 (kiểm tra chéo); git diff không có data/ hay .zip.
```

## 4. Dương Dương (`duongduong1606`)

- (a) **MSSV chưa điền:** sửa đúng dòng 5 của [TEAMMATES.md](TEAMMATES.md), PR riêng `dd/mssv`, làm việc này trước tiên.
- (b) Issue [#9](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/9) bộ câu hỏi và trả lời; [#10](https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint/issues/10) kịch bản nói 3–5 phút chia đều cho 5 người. Nhóm không demo trực tiếp: kịch bản chỉ dùng hình và bảng đã có trong repo.
- (c) `reports/DuongDuong.md`; failure case gợi ý: đỉnh giả roll–pitch trên 0052, giải thích bằng lời của mình.
- (d) Bốn PR: `dd/mssv`, `dd/report`, `dd/issue-9`, `dd/issue-10`. (e) Nộp VLearn như trên.

**Prompt cho agent:**

```text
Repo: https://github.com/Catnip-harvest/K4-Track4-Day04-CeilingFan-Sensor-Reality-Sprint (tôi là duongduong1606, có quyền push).
Đề T3: lệch hiệu chuẩn camera–LiDAR trên xe ADAS, KITTI raw. Đọc trước: README.md, REPORT_1PAGE.md, REPORT.md, SUBMISSION.md (mục Bước 1, Bước 2), TEAM_TASKS.md, TEAMMATES.md, reports/README.md, reports/HoangQuocViet.md (chỉ làm mẫu, không chép), docs/DETECTION_CHECK.md.
Cài (để tự chạy lại số trích dẫn): pip install numpy matplotlib pillow opencv-python pandas ; python get_data.py
Quy trình: git switch main && git pull; mỗi việc một nhánh (dd/mssv, dd/report, dd/issue-9, dd/issue-10) và một PR nhỏ vào main; rebase lên origin/main trước khi merge.
Ràng buộc: không commit data/, *.zip; trong TEAMMATES.md chỉ sửa dòng "5. **Dương Dương** — MSSV: [tự điền]"; không sửa tệp của người khác (REPORT*.md, SUBMISSION.md, PHAN_BIEN.md, báo cáo người khác, results/); viết tiếng Việt có dấu.
Việc 0 (bắt buộc, làm trước): điền MSSV của tôi vào dòng 5 của TEAMMATES.md (tôi sẽ cung cấp số), PR riêng.
Việc 1 (bắt buộc): viết reports/DuongDuong.md, 5 mục Problem / Method / Benchmark / Failure case / Engineering decision bằng lời của tôi; trích bảng/hình chung bằng link tương đối kèm đúng lệnh (ví dụ ../results/figures/04_monitor.png từ `python run_benchmark.py`); có câu "Nhóm quan sát được …" và câu "Paper/repo cho biết …" tách riêng; một failure case; một cải tiến gắn với nó và metric/log kiểm chứng; nhắc kết quả issue #9, #10.
Việc 2 (#9): docs/QA.md, 10–15 câu hỏi người chấm có thể hỏi (proxy có hợp lý không, vì sao mù roll, vì sao improved kém hơn trên 0052, SoftCorr mới 5/20 lệch, tracklet là cận trên ...), mỗi câu trả lời 2–3 câu kèm link tới số liệu trong repo.
Việc 3 (#10): docs/SCRIPT.md, kịch bản 3–5 phút chia đều 5 người theo năm mục báo cáo, không có demo trực tiếp, chỉ dùng hình/bảng có sẵn trong results/.
Nghiệm thu: TEAMMATES.md chỉ đổi đúng một dòng; PR báo cáo đủ 5 mục, ≥ 1 link tới bảng/hình chung + lệnh, hai câu "Nhóm quan sát được"/"Paper/repo cho biết", 1 failure case, 1 cải tiến có metric; mọi con số trong QA.md và SCRIPT.md khớp với tệp trong results/; git diff không có data/ hay .zip.
```
