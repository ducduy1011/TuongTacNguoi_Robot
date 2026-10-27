# Bài 6 – Điều khiển TurtleBot 4 bằng cử chỉ tay

Bài làm dùng webcam, OpenCV, MediaPipe Gesture Recognizer và ROS 2 Humble.
Môi trường: **Ubuntu 22.04, Python 3.10, TurtleBot 4 Standard, Gazebo Fortress**.
Workspace độc lập, không cần các bài thực hành trước.

## 1. Cấu trúc

```text
bai6_gesture_ws/
|-- README.md
|-- requirements.txt
`-- src/turtlebot4_btth6/
    |-- package.xml, setup.py, setup.cfg, resource/
    |-- turtlebot4_btth6/
    |   |-- gesture_recognition_node.py
    |   |-- robot_behavior_node.py
    |   |-- gesture_filter.py, control.py
    |   `-- wait_for_controllers.py
    |-- launch/  (simulation.launch.py, gesture.launch.py)
    |-- config/controllers.yaml
    |-- worlds/gesture_arena.sdf
    `-- models/gesture_recognizer.task
```

`build/`, `install/`, `log/`, `.venv/` được tạo trên từng máy, không cần nộp.
Các node dùng gói ROS chuẩn, không ghi đè gói hệ thống. Đường dẫn tài nguyên được tìm bằng ament nên có thể build ở thư mục khác. `ROS_DOMAIN_ID=46` được đặt giống nhau ở các terminal để tách nhóm topic Bài 6.

## 2. Cài đặt và build trên máy thầy cô

Máy cần có Ubuntu 22.04 và ROS 2 Humble đã cài, có giao diện đồ họa và webcam.
Clone đúng nhánh Bài 6 vào Home:

```bash
cd ~
git clone --branch assignments_6 --single-branch https://github.com/ducduy1011/TuongTacNguoi_Robot.git bai6_gesture_ws
```
Nếu đã có TurtleBot 4 Simulator từ các bài trước thì không cần cài lại simulator.

Cài các gói còn thiếu (cần mạng và quyền sudo):

```bash
sudo apt update
sudo apt install python3-venv python3-pip python3-colcon-common-extensions ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes
```

Trong terminal mới:

```bash
cd ~/bai6_gesture_ws
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
PYTHONNOUSERSITE=1 python3 /usr/bin/colcon build --symlink-install --packages-select turtlebot4_btth6
```

Dùng Python của `.venv` để build giúp hai node sử dụng đúng MediaPipe/OpenCV.
`--system-site-packages` cho phép môi trường này dùng thư viện ROS hệ thống.
`PYTHONNOUSERSITE=1` tránh xung đột setuptools cá nhân khi build.
Không dùng Conda hoặc Python khác 3.10 để build bài này.

Sau khi build, mở các terminal mới theo mục 3. Không dùng thư mục build, install hoặc môi trường Python chép từ máy khác.

## 3. Các lệnh chạy

### Terminal 1 – Gazebo

```bash
cd ~/bai6_gesture_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=46
ros2 launch turtlebot4_btth6 simulation.launch.py
```

Nếu máy Intel/NVIDIA gặp lỗi đồ họa `X_GLXCreateContext` / `BadValue`, thay lệnh cuối bằng:

```bash
ros2 launch turtlebot4_btth6 simulation.launch.py use_mesa:=true
```

Tùy chọn này dùng Intel/Mesa cho riêng launch; không cần bật nếu đồ họa bình thường.
Đợi **ROBOT READY**, cửa sổ Gazebo xuất hiện và mô phỏng ở Play.
Tiện ích chờ controller giúp GUI không tranh tài nguyên khi robot đang nạp;
đây không phải node nhận diện hoặc điều khiển cử chỉ.

### Terminal 2 – mở webcam và hai node

```bash
cd ~/bai6_gesture_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=46
source .venv/bin/activate
ros2 launch turtlebot4_btth6 gesture.launch.py
```

Robot xuất hiện tự do giữa sân, không có dock, không cần lệnh undock hoặc thay đổi safety_override.
Mô phỏng dùng mô hình TurtleBot 4 Standard chính thức với `ros2_control`:
controller nhận trực tiếp `/cmd_vel` và phát `/odom`. Bài không chạy các hành vi tự động của Create 3.
Chỉ mở một phiên webcam và không mở teleop/Nav2 đồng thời.
Nếu webcam USB có index khác, thay lệnh cuối bằng:

```bash
ros2 launch turtlebot4_btth6 gesture.launch.py camera_index:=2
```

### Terminal 3 – xem topic, nếu cần

```bash
cd ~/bai6_gesture_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=46
ros2 topic echo /detected_gesture
```

Có thể xem `/cmd_vel` ở terminal khác. Khi mới chọn, vận tốc bằng 0;
sau xác nhận vận tốc khác 0; xòe tay làm vận tốc về 0. Khi nắm tay,
robot hủy quỹ đạo cũ rồi phát vận tốc để quay về điểm bắt đầu.

## 4. Cách dùng và quay video

Đặt Gazebo cạnh cửa sổ webcam. Chỉ đưa một tay vào ảnh, đủ sáng, thấy toàn bộ ngón tay.
Webcam vẽ đường nối và chấm tròn ở các khớp, hiển thị nhãn AI, confidence,
nhãn gửi topic và trạng thái robot. Xem **Topic** và **SELECTED/RUNNING** để biết đã nhận lệnh.

| Cử chỉ | Thực hiện |
|---|---|
| Một ngón – Pointing Up | Chọn tiến 0,4 m rồi lùi 0,4 m, lặp 4 lần (8 chặng) |
| Hai ngón – Victory | Chọn đi một vòng tròn, bán kính khoảng 0,4 m |
| Vẫy tay – Wave | Chọn chào: quay trái → phải → trở lại gần hướng đầu |
| Ngón cái – Thumbs Up | Xác nhận một lần; bỏ tay ra robot vẫn thực hiện nốt |
| Xòe tay giữ yên – Open Palm | Dừng ngay và xóa lựa chọn |
| Nắm tay – Closed Fist | Hủy hành vi, tự đi về vị trí và hướng bắt đầu rồi chờ |

**Trình tự:** chọn → đợi `SELECTED ...` → giơ ngón cái → đợi `RUNNING ...`.
Khi đã hiện `RUNNING ...`, có thể bỏ ngón cái và đưa tay khỏi ảnh;
robot tiếp tục đến `COMPLETED`, không cần giữ tay.
Đổi từ cử chỉ chọn sang xác nhận trong khoảng 3 giây; tránh xòe hoặc nắm tay giữa chừng.
Wave cũng cần xác nhận, đúng yêu cầu chỉ thực hiện sau Thumbs Up.

**Vị trí ban đầu** được ghi bằng odometry ngay lúc ngón cái xác nhận hành vi.
Nắm tay khi đang chạy sẽ hiện `RETURNING TO START`: robot ngừng hành vi cũ,
quay về hướng điểm đầu, di chuyển về đó và chỉnh lại hướng ban đầu.
Không cần xác nhận thêm, cũng không cần giữ nắm tay. Khi về xong hiện
`CANCELLED_RETURNED`; sai số mục tiêu khoảng 2 cm và 0,04 rad theo odometry.
Giữ nắm tay liên tục không khởi động lại việc quay về. Xòe tay vẫn dừng ngay
trong lúc robot đang quay về, xóa lệnh quay về và không tự tiếp tục.
Nếu chưa chạy hành vi nào, nắm tay chỉ hủy lựa chọn, robot đứng yên.

Để chọn Wave, xòe tay hướng lòng bàn tay về camera, vẫy ngang trái–phải–trái
trong 1–3 giây, mỗi lượt qua lại khoảng 1–2 bề ngang lòng bàn tay.
Theo dõi `turns 0/2 → 1/2 → 2/2`; khi hiện `Topic: Wave` và `SELECTED WAVE`,
đổi sang ngón cái. `AI: Open_Palm` đơn lẻ chưa có nghĩa là đã chọn Wave.
MediaPipe không có lớp Wave sẵn; bài dùng chuyển động của các điểm bàn tay AI để nhận Wave.
Confidence trên màn hình thuộc nhãn AI, không phải xác suất Wave.

Quay đủ: chọn nhưng chưa xác nhận, tiến–lùi 4 lần, đi vòng tròn, chào lại,
xòe tay dừng, nắm tay hủy và quay về điểm đầu. Đưa tay khỏi ảnh sau xác nhận
để chứng minh robot vẫn hoàn thành hành vi. Đóng webcam để chứng minh
mất kết nối điều khiển thì robot dừng.
Trong video, cho thấy đồng thời cử chỉ và phản ứng của robot, không gõ lệnh lái.

Chống nhiễu: chọn/xác nhận giữ ổn định 0,3 giây, có ngưỡng confidence,
không tự lặp khi giữ ngón cái. `No_Hand`/`Unknown` vẫn là message báo camera
đang hoạt động, nên không dừng hành vi đã xác nhận. Nếu camera ngừng gửi
message hoặc mất odometry quá khoảng 0,7 giây, robot dừng cả hành vi và lệnh
quay về. Sau mất kết nối phải chọn và xác nhận lại; không tự chạy tiếp.
Mọi chuyển động chỉ thử trong sân mô phỏng trống. Bài không có chức năng tự tránh vật cản hoặc bảo vệ Create 3; không dùng launch này để lái robot thật.

**Kết thúc:** xòe tay để dừng, rồi Ctrl+C ở Terminal 2 và Terminal 1.
Đóng cửa sổ webcam cũng dừng cả hai node cử chỉ.

## 5. Kiểm tra và xử lý lỗi

Kiểm tra controller bằng `ros2 control list_controllers`: cần có `joint_state_broadcaster` và `diffdrive_controller` đều active. Dùng `ros2 topic echo /cmd_vel` để xem lệnh vận tốc và `ros2 topic echo /robot_status` để xem trạng thái.

| Hiện tượng | Kiểm tra |
|---|---|
| Không mở được camera | Đóng ứng dụng camera khác; kiểm tra `camera_index` |
| AI thấy tay nhưng robot không chạy | Đợi Topic đúng và SELECTED, sau đó xác nhận; kiểm tra hai controller active |
| SIGNAL_LOST | Kiểm tra node camera còn chạy và cùng ROS_DOMAIN_ID; chọn và xác nhận lại sau khi dừng |
| ODOM_LOST | Gazebo phải Play; kiểm tra `ros2 control list_controllers` có hai controller active |
| Không tìm thấy package | Build và source đúng workspace; không source bài khác sau đó |
| X_GLXCreateContext / BadValue trên máy Intel/NVIDIA | Thử `use_mesa:=true`; tùy chọn này không sửa driver hệ thống |

Mô hình AI đã kèm trong bài, không cần tải khi chạy. Thư viện Python cần được cài trước.
Hướng dẫn chuẩn bị simulator theo [TurtleBot 4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html).
Mô hình theo [MediaPipe Gesture Recognizer](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer).

## 6. Video demo

[Video Bài 6 trên Google Drive](https://drive.google.com/drive/folders/1fgSjUEJVbLln_qFV6CG24UOq2GFeI8SX?usp=sharing).

Trong bản này, nắm tay hủy hành vi rồi tự quay về điểm bắt đầu trước khi chờ. Đây là phần mở rộng so với yêu cầu tối thiểu chỉ hủy và về trạng thái chờ. Bỏ tay khỏi ảnh không được coi là mất kết nối: camera vẫn gửi No_Hand; ngừng gửi dữ liệu hoặc mất odometry mới kích hoạt watchdog.
