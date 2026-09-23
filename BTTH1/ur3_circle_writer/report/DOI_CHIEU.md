# Kết quả đối chiếu BaoCaoT1.pdf

Ngày đối chiếu: 23/09/2026.

Cả 5 listing trong PDF khớp mã nguồn về logic. Kiểm tra tự động xác nhận mỗi listing có trong mã nguồn sau khi bỏ chú thích và khoảng trắng.

| Listing trong PDF | Kết quả |
| --- | --- |
| 1: createLetterD | Khớp hàm trong ur3_letter_writer/src/draw_letter_d.cpp |
| 2: createCircle | Khớp hàm trong ur3_circle_writer/src/draw_circle.cpp |
| 3: Khởi tạo MoveGroupInterface | Khớp cả hai chương trình; chỉ là trích đoạn |
| 4: computeCartesianPath | Khớp chương trình hình tròn; chữ D gọi riêng hai ứng viên |
| 5: Chọn trajectory chữ D | Khớp; chưa trích phần chọn best_points và kiểm tra fraction |

Các tham số và công thức trong PDF khớp YAML/C++: chữ D cao 0.12 m, rộng 0.08 m, 50 điểm cung; hình tròn bán kính 0.07 m, 100 đoạn; eef_step=0.003, jump_threshold=2.0, min_fraction=0.999, hai hệ số 0.12, chờ guide 10 giây, nghỉ giữa lượt 2 giây, repeat_count=0 và ready_joints.

## Nội dung đã chỉnh/bổ sung

- Sửa mô tả lặp vô hạn: vòng lặp còn dừng nếu execute() không trả về SUCCESS.
- Làm rõ trajectory được tính trước vòng lặp rồi tái sử dụng.
- Trích nguyên văn 11 khối từ bài làm: hai hàm tạo hình, khởi tạo MoveIt và kiểm tra trạng thái, PRE-DRAW, tính Cartesian path hình tròn, tính hai ứng viên chữ D, chọn và kiểm tra ứng viên, hai vòng lặp và hai YAML.
- Làm rõ khi fraction bằng nhau thì chọn +X; chữ D có 52 pose; tâm hình tròn là (x0-R,y0,z0).
- Bổ sung khởi động RViz sau 8 giây và node vẽ sau 20 giây bằng TimerAction; đây là khoảng trễ cố định.
- Phân biệt tham số node 0.12 trong YAML với giá trị giao diện 0.1 trong RViz; frame của waypoint lấy từ pose hiện tại.
- Giữ ảnh và số liệu thực nghiệm dưới dạng chỗ điền. Hai URL trong PDF chứa THAY-LINK-GITHUB-TAI-DAY và THAY-LINK-VIDEO-TAI-DAY, nên bản mới để trống URL cho người dùng điền.

## Sử dụng

Tải BaoCaoT1_da_doi_chieu.tex lên Overleaf và chọn pdfLaTeX. Tệp này độc lập, không cần tải template, script hay mã nguồn bài làm lên cùng.

Các ảnh tùy chọn đặt cạnh tệp LaTeX: logo.png, overview_simulation.png, letter_d_rviz.png, terminal_fraction.png, circle_demo.png. Thiếu ảnh sẽ hiện khung giữ chỗ. Điền URL và fraction ở các lệnh githuburl, demourl, fractionD, fractionCircle đầu tệp.

Bố cục được dựng lại từ nội dung PDF vì không có tệp .tex gốc; số trang có thể khác bản gốc.

## Kiểm tra đã thực hiện

5 listing gốc khớp sau khi bỏ chú thích/khoảng trắng. 11 trích đoạn mới khớp nguyên văn tệp nguồn. Đã kiểm tra cấu trúc lồng môi trường LaTeX. source_manifest.json lưu đường dẫn, dòng bắt đầu và SHA-256 của mã nguồn.

Máy chưa có trình biên dịch LaTeX nên chưa biên dịch hoặc kiểm tra bố cục PDF. Không chạy lại ROS/Gazebo, build package hay xác nhận phiên bản hệ điều hành/ROS. Không sửa mã nguồn bài làm.

Có thể chạy python3 report/build_report.py từ package ur3_circle_writer để sinh lại báo cáo nếu mã nguồn thay đổi; lệnh này ghi đè bản .tex đã sinh.
