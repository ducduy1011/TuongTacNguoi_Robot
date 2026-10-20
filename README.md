# Bài thực hành 05 — TurtleBot 4 Standard: Navigation bằng code và YOLO

Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, **TurtleBot 4 Standard**, world **maze**. Dùng lại nguyên bản `map.yaml` và `map.pgm` đã quét ở Bài 04. Workspace vẫn là `~/turtlebot4_btth4_ws`.

Tuần này **node Python gửi goal**, thay thao tác Nav2 Goal bằng chuột. Node YOLO nhận ảnh RGB thật của camera mô phỏng qua `cv_bridge`, vẽ bounding box/class/confidence và publish ảnh kết quả. Không có script `.sh` tự viết. `setup.bash` là file môi trường chuẩn do ROS/colcon sinh ra.

## Chạy demo để quay video — một launch

Trên máy đã chuẩn bị/build xong, bắt đầu quay màn hình rồi chạy trong Terminal:

```bash
source ~/turtlebot4_btth4_ws/install/setup.bash
ros2 run turtlebot4_btth4 stop_simulation
ros2 launch turtlebot4_btth5 demo.launch.py
```

Launch tự mở Gazebo Standard/maze, nạp map tuần trước, tạo **8 vật thể (4 người, 4 ghế)**, bật AMCL/Nav2 và cửa sổ YOLO. Node `prepare_robot` kiểm tra robot đang ở dock khởi đầu và wheel controllers active, đặt initial pose (0,0,0), chờ AMCL/TF/Nav2/ảnh YOLO rồi hiển thị map thêm 5 giây trước khi tự undock. Sau đó node `navigate_goals` gửi tuần tự cả 3 goal bằng code, dừng 5 giây tại mỗi goal. Không cần đặt goal bằng chuột, không mở teleop và không gửi lại initial pose khi robot đã đi.

Simulator được khởi động trước; sau 15 giây helper mới chờ controller và tạo vật thể, rồi mới mở localization/YOLO. Khởi động có thể mất khoảng một phút trên máy hiện tại; hãy để launch chờ, không chạy thêm một phiên khác. Bộ goal mới ngày 21/10 có đường đi dài hơn; nên dành khoảng 5–7 phút để quay, thời gian thực tế tùy quá trình điều hướng. Nếu đang dùng lệnh demo này thì không chạy thêm các launch riêng ở mục 3–6.

Phải thấy đủ `Goal 1 SUCCEEDED`, `Goal 2 SUCCEEDED`, `Goal 3 SUCCEEDED`, `COMPLETE: 3/3 goals SUCCEEDED.`. Giữ camera YOLO và RViz nhìn được khi quay. Nếu chuẩn bị robot hoặc tạo vật thể thất bại, launch dừng, không bỏ qua lỗi để chạy goal. Không chạy demo chồng lên phiên Gazebo cũ. Các lệnh riêng từng phần vẫn có ở dưới để học và kiểm tra.

Nếu góc nhìn Gazebo quá sát mặt đất, cuộn chuột lùi để thu nhỏ và kéo góc nhìn để thấy maze/robot; đây chỉ là camera quan sát, không điều khiển robot. Có thể phóng to cửa sổ RViz và sắp xếp cạnh cửa sổ YOLO trước khi ghi phần robot di chuyển.

Quay xong, mở Terminal khác, source môi trường và chạy `ros2 run turtlebot4_btth4 stop_simulation` để dừng sạch toàn phiên, giữ map/video.

## 1. Cấu trúc và các phần đáp ứng đề bài

```text
src/turtlebot4_btth4/            # Package tuần trước: Standard simulator, AMCL/Nav2, map
  maps/map.yaml
  maps/map.pgm
  launch/simulation.launch.py
  launch/navigation.launch.py
  README.md                    # Hướng dẫn mapping của Bài 04 được giữ tại đây
src/turtlebot4_btth5/
  scripts/navigate_goals.py     # Gửi lần lượt 3 NavigateToPose, chờ kết quả từng goal
  scripts/yolo_detector.py      # RGB -> cv_bridge -> YOLO -> ảnh/JSON kết quả
  scripts/record_pose.py        # In x, y, yaw từ AMCL để ghi nhận tọa độ
  scripts/spawn_objects.py      # Tạo 8 vật thể, kiểm tra phản hồi Gazebo
  scripts/prepare_robot.py      # Chuẩn bị simulator mới ở dock, initial pose và undock
  launch/demo.launch.py         # Toàn bộ demo tự động từ một launch
  launch/localization.launch.py
  launch/perception.launch.py
  launch/patrol.launch.py
  launch/objects.launch.py      # Thêm người/ghế 3-D vào maze khi cần
  config/goals.yaml
  config/localization.yaml      # AMCL dùng likelihood_field_prob và beam skipping
  config/btth5.rviz             # Robot, scan, map, costmap, camera RGB và ảnh YOLO
  config/scene_sources.md       # Nguồn, giấy phép và vị trí mô hình 3-D
tools/prepare_runtime.py       # Thư viện tương thích đã dùng tuần trước
tools/prepare_yolo.py          # Môi trường Python riêng và YOLOv8n
tools/yolo_requirements.txt
tools/prepare_scene.py         # Tải mô hình 3-D, không tự spawn/chạy robot
```

- Map nằm **một nơi duy nhất** trong package Bài 04; Bài 05 dùng trực tiếp, không quét lại.
- Map hiện tại: 402 × 401 ô, 0,05 m/ô, origin `[-8.04, -10, 0]`, ảnh tương đối `map.pgm`.
- Node navigation gọi action `/navigate_to_pose`, dùng quaternion từ yaw (radian), frame `map`. Chỉ sau `SUCCEEDED` mới gửi goal tiếp; lỗi/rejected/canceled/timeout thì dừng chuỗi.
- Node kiểm tra AMCL, TF, lifecycle Nav2 và trạng thái đã rời dock trước khi gửi goal. Độ lệch chuẩn AMCL phải không quá 0,7 m cho x/y và 0,5 rad cho yaw; kiểm tra lại trước goal tiếp theo. Covariance nhỏ vẫn cần đối chiếu scan/map. Không tự đặt initial pose, không tự undock, không tự phát `/cmd_vel`.
- YOLO và navigation là hai process riêng. YOLOv8n chạy CPU, tối đa 2 thread, mục tiêu 2 FPS, ảnh inference 320 px; giữ ảnh mới nhất để hạn chế ảnh tồn đọng. FPS thực tế tùy máy.

## 2. Cài đặt một lần

### Máy đang dùng của sinh viên

Workspace đã có simulator/runtime tuần trước. Cài bổ sung nếu thiếu:

```bash
sudo apt update
sudo apt install python3-pip python3-venv ros-humble-nav2-simple-commander \
  ros-humble-turtlebot4-navigation ros-humble-cv-bridge python3-opencv
cd ~/turtlebot4_btth4_ws
source /opt/ros/humble/setup.bash
python3 tools/prepare_yolo.py
python3 tools/prepare_scene.py
source .deps/install/setup.bash
colcon build --symlink-install --packages-select turtlebot4_btth4 turtlebot4_btth5
source install/setup.bash
```

Không cần chạy lại phần cài đặt mỗi lần quay. `prepare_yolo.py` tạo `.venv-yolo/` với system site packages để thấy ROS/cv_bridge, cài PyTorch CPU 2.5.1, torchvision 0.20.1, Ultralytics 8.3.203, NumPy 1.26.4 và OpenCV 4.11.0.86; tải `.models/yolov8n.pt` và thử inference ảnh rỗng. Giữ NumPy dưới 2 để tương thích bản `cv_bridge` của Humble. Không cài pip đè vào Python hệ thống. Lần đầu cần Internet để tải thư viện/trọng số/mô hình.

### Máy mới của người chấm

Cần cài Ubuntu 22.04, ROS 2 Humble Desktop và cấu hình apt ROS/Gazebo theo tài liệu chính thức trước. Source Bài 05 nằm trên nhánh Public [assignments_5](https://github.com/ducduy1011/TuongTacNguoi_Robot/tree/assignments_5). Clone vào `~/turtlebot4_btth4_ws` bằng lệnh dưới đây; nếu đường dẫn này đã tồn tại, dùng một thư mục mới hoặc sao lưu workspace đang có trước:

```bash
git clone --branch assignments_5 --single-branch \
  https://github.com/ducduy1011/TuongTacNguoi_Robot.git ~/turtlebot4_btth4_ws
cd ~/turtlebot4_btth4_ws
sudo apt update
sudo apt install git cmake build-essential python3-pip python3-venv python3-pil python3-yaml \
  python3-opencv python3-colcon-common-extensions libceres-dev libtbb-dev qtbase5-dev \
  libboost-thread-dev libboost-serialization-dev libboost-filesystem-dev \
  ignition-fortress ros-humble-ament-cmake-gen-version-h \
  ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes \
  ros-humble-turtlebot4-navigation ros-humble-slam-toolbox ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-nav2-simple-commander ros-humble-cv-bridge \
  ros-humble-rviz2 ros-humble-rclpy ros-humble-geometry-msgs ros-humble-rmw-fastrtps-cpp
source /opt/ros/humble/setup.bash
python3 tools/prepare_runtime.py
python3 tools/prepare_yolo.py
python3 tools/prepare_scene.py
source .deps/install/setup.bash
colcon build --symlink-install --packages-select turtlebot4_btth4 turtlebot4_btth5
source install/setup.bash
```

Không cần copy `.deps/`, `.venv-yolo/`, `.models/`, `build/`, `install/`, `log/` lên GitHub. Các công cụ Python chuẩn bị lại chúng trên máy chạy. Runtime giữ TF2 0.25.22 cục bộ, controller 100 Hz và DDS UDP loopback; `ROS_LOCALHOST_ONLY=0` để Humble không tự thêm SHM, còn XML vẫn chỉ cho phép interface 127.0.0.1; không sửa `/opt/ros`. Cần desktop/OpenGL hoạt động; WSL2 cần WSLg.

`prepare_runtime.py` cũng tạo wrapper Python cho controller spawner: chờ controller manager tối đa 120 giây và chờ mỗi service call tối đa 60 giây khi khởi động nguội. Cấu hình simulator tắt `power_saver` của node TurtleBot 4 để không gọi các dịch vụ tiết kiệm điện dành cho phần cứng camera/LiDAR. Controller, robot Standard và cảm biến mô phỏng vẫn dùng package chính thức.

Profile DDS tăng `max_blocking_time` của writer lên 10 giây để phản hồi service có thời gian chờ discovery khi Gazebo mới mở. Mặc định 100 ms có thể làm mất phản hồi `robot_description`, khiến việc tạo controller bị chờ mãi. Cơ chế chờ này có trong [mã nguồn RMW Fast DDS của Humble](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/rmw_response.cpp).

## 3. Bắt đầu một phiên demo mới — Phần 1 của đề

Mở 5 Terminal. **Mỗi Terminal mới** đều chạy:

```bash
source ~/turtlebot4_btth4_ws/install/setup.bash
```

Các Terminal dùng cùng máy, ROS domain 44 và partition do workspace thiết lập. Không mở đồng thời simulator cũ, SLAM hoặc teleop.

**Terminal 5 — dừng phiên cũ trước khi bắt đầu:**

```bash
ros2 daemon stop
ros2 run turtlebot4_btth4 stop_simulation
```

**Terminal 1 — simulator, đúng maze của tuần trước:**

```bash
ros2 launch turtlebot4_btth4 simulation.launch.py
```

Chờ robot Standard xuất hiện, mô phỏng Play và controller active. Ban đầu robot ở dock tại world x=-2, y=0. Đây là tọa độ Gazebo, không phải map.

**Terminal 2 — load map, AMCL, Nav2 và RViz:**

```bash
ros2 launch turtlebot4_btth5 localization.launch.py
```

Giữ robot ở dock. Dùng **2D Pose Estimate** trên RViz: với map/spawn đi kèm, pose gần **x=0, y=0, yaw=0**, mũi tên hướng +X (sang phải trên bản đồ). Chờ scan trùng tường. Terminal sẽ báo `AMCL/TF ready; starting Nav2.` Không dùng origin YAML làm initial pose và không gán pose (0,0) sau khi robot đã di chuyển.

**Terminal 5 — kiểm tra:**

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 topic echo /scan --once --qos-reliability best_effort --field header
ros2 topic echo /oakd/rgb/preview/image_raw --once --qos-reliability best_effort --field header
```

Lifecycle phải `active`. RViz hiển thị robot, map, LiDAR và **Camera RGB**; chưa chạy YOLO thì ô YOLO chưa có ảnh là bình thường. Nếu scan lệch tường, sửa localization trước khi đi tiếp. Pose/TF có dữ liệu chưa tự chứng minh localization đúng.

## 4. Tọa độ mục tiêu — Phần 2 của đề

Ba goal dưới đây là ba điểm Publish Point sinh viên cung cấp ngày 21/10/2026, giữ nguyên x/y và thứ tự trong frame `map`. Cả ba nằm trong vùng trống; khoảng cách từ tâm goal đến ô occupied/unknown lần lượt khoảng 0,95 / 1,10 / 1,50 m trên raster. Planner Nav2 đã tính được đường cho cả ba đoạn, nhưng bộ goal mới chưa được kiểm chứng bằng một lượt robot di chuyển hoàn chỉnh. Tuyến mới có lối hẹp hơn tuyến đã chạy ngày 19/10; không giảm bán kính robot hoặc inflation của Nav2 để ép đi qua.

Publish Point không có yaw. Hướng đến cuối goal được chọn để camera nhìn về ghế `chair_4`, người `person_2` và ghế `chair`, tương ứng Goal 1/2/3. Bảng làm tròn để đọc; YAML giữ nguyên độ chính xác của x/y đã cung cấp:

| Goal | x (m) | y (m) | yaw (rad) |
|---|---:|---:|---:|
| 1 | 11.082431 | -8.204782 | 1.918947 |
| 2 | -6.781111 | 1.195105 | 1.234788 |
| 3 | -6.455601 | -8.403458 | 0.696685 |

Muốn khảo sát/chọn lại: dùng **Publish Point** trên RViz và ở Terminal 5 chạy:

```bash
ros2 topic echo /clicked_point
```

Click ít nhất 3 vị trí ở khu vực khác nhau, trong vùng trắng có đường đi và đủ khoảng hở. Ghi x/y vào `src/turtlebot4_btth5/config/goals.yaml`. **Publish Point chỉ có vị trí, không có yaw**: tự chọn yaw theo hướng mong muốn, 0 hướng +X, pi/2 hướng +Y, pi hướng -X, -pi/2 hướng -Y. Đơn vị radian, không phải độ. Đừng dùng 2D Pose Estimate để lấy tọa độ goal vì công cụ đó thay đổi localization.

Nếu muốn ghi lại pose/hướng robot đang đứng thực tế trên map (khảo sát riêng trước lượt demo), dùng:

```bash
ros2 run turtlebot4_btth5 record_pose --ros-args -p use_sim_time:=true
```

Ctrl+C dừng lệnh echo/record trước lệnh tiếp theo. Build symlink đọc trực tiếp `goals.yaml` mới, không cần build lại khi chỉ sửa file này. Các goal mặc định đã kiểm tra nằm trong free space của map, nhưng sau khi thêm vật thể vẫn phải kiểm tra môi trường thực tế.

## 5. Thêm vật thể và chạy YOLO — Phần 4 của đề

Maze gốc chủ yếu là tường. Muốn thấy nhận diện **person/chair**, tạo 8 vật thể từ hai mô hình 3-D thật đã tải (không phải ảnh dán lên tường).

**Terminal 5, sau khi simulator đã sẵn sàng:**

```bash
ros2 launch turtlebot4_btth5 objects.launch.py
```

Lệnh chỉ spawn mô hình, không chạy robot; chạy **một lần mỗi phiên Gazebo**. Launch tự đọc tên world từ file maze của TurtleBot 4; trên máy hiện tại là `maze`. Chỉ báo tạo thành công khi dịch vụ Gazebo trả về `data: true`. Vị trí, nguồn và giấy phép nằm ở `config/scene_sources.md`. Trong Gazebo, mở **Entity Tree**, tìm `btth5_person` và `btth5_chair`. Tám vật thể được phân bố ở các vùng khác nhau; camera tại dock chưa nhìn thấy tất cả. Các vật thể gần Goal 1/2 nằm theo hướng quay camera đã cấu hình. Kiểm tra mô hình hiện đúng, không đè robot/tường/goal. Nếu chỉnh vị trí bằng Gazebo, giữ hành lang đủ rộng và sửa yaw/goal phù hợp để vật thể vào góc nhìn camera. Map cũ được giữ nguyên; vật cản mới được LiDAR cập nhật vào costmap khi robot quan sát được.

**Terminal 3 — chạy YOLO liên tục:**

```bash
ros2 launch turtlebot4_btth5 perception.launch.py show_window:=true
```

Mặc định `show_window:=false` nếu chỉ muốn ảnh YOLO trên RViz. Hai ô **Camera RGB** và **YOLO detections** trong RViz cho phép so sánh ảnh gốc/kết quả. Cửa sổ OpenCV vẽ box, class, confidence. Terminal in tên và độ tin cậy khoảng mỗi 2 giây, kể cả `none (0 objects)` khi không có vật thể phù hợp trong ảnh.

Có thể kiểm tra tại Terminal 5:

```bash
ros2 topic hz /yolo/image
ros2 topic echo /yolo/detections --once
```

Ctrl+C dừng riêng `topic hz` trước lệnh tiếp theo. `/yolo/detections` là `std_msgs/String` chứa JSON gồm frame/stamp, class, confidence và box xyxy theo pixel. `/yolo/image` là `sensor_msgs/Image` BGR8, giữ nguyên header của camera.

Demo lọc kết quả YOLO theo hai class `person` và `chair` tương ứng mô hình trong scene (tham số `target_classes` của node). Đây vẫn là suy luận thật từ RGB, không phải đọc tên model Gazebo. YOLOv8n pretrained COCO có class person/chair; không có class bình chữa cháy. Không gán tên tùy ý cho hình hộp hoặc tạo kết quả giả. Nhận diện trên mesh mô phỏng phụ thuộc góc nhìn, kích thước, ánh sáng; chỉ có ảnh camera chưa đủ chứng minh nhận diện. Cần thực sự thấy box/class/confidence. Nếu chưa thấy, kiểm tra ảnh RGB, vị trí/hướng mô hình và camera; khi cần chỉnh `confidence` trong node hoặc dùng mô hình phù hợp, không hạ ngưỡng đến mức chỉ còn false positive.

## 6. Chạy code tự động qua 3 goal — Phần 3 của đề

**Terminal 4 — chuẩn bị rời dock sau khi localization đúng và Nav2 active:**

```bash
ros2 topic echo /dock_status --once
```

Nếu `is_docked: true`, chạy và chờ `SUCCEEDED`:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'
```

Sau đó:

```bash
ros2 param set /motion_control safety_override backup_only --no-daemon --spin-time 3
```

Không mở WASD. Không sửa initial pose về (0,0) sau undock. Giữ Terminal 3 YOLO hoạt động.

**Bắt đầu chuỗi điều hướng, Terminal 4:**

```bash
ros2 run turtlebot4_btth5 navigate_goals --ros-args -p use_sim_time:=true -p pause_at_goal:=5.0
```

Lệnh trên chạy trực tiếp node Python `scripts/navigate_goals.py`; node đọc `config/goals.yaml` và gửi action `/navigate_to_pose`. Các lần bấm Publish Point trước đó chỉ dùng khảo sát tọa độ, không gửi goal điều hướng.

Từ lúc này **không điều khiển thủ công và không dùng chuột đặt Nav2 Goal**. Node tự gửi Goal 1, theo dõi action; khi `SUCCEEDED`, đứng chờ 5 giây theo tham số ở lệnh trên rồi gửi Goal 2, tương tự Goal 3. RViz vẫn hiển thị path/costmap và ảnh YOLO trong khi chạy. Log cần có:

```text
Goal 1 SUCCEEDED
Goal 2 SUCCEEDED
Goal 3 SUCCEEDED
COMPLETE: 3/3 goals SUCCEEDED.
```

Goal abort/reject/cancel/timeout thì node kết thúc chuỗi, không bỏ qua goal để báo hoàn thành. Mỗi goal mặc định tối đa 360 giây tính theo thời gian thực. Có thể điều chỉnh, ví dụ:

```bash
ros2 launch turtlebot4_btth5 patrol.launch.py goal_timeout:=600.0 pause_at_goal:=5.0
```

Ctrl+C ở Terminal 4 yêu cầu cancel goal hiện tại. Chờ thông báo/hết chuyển động; nếu Nav2 không phản hồi hãy dừng cả phiên ở Terminal 5. Không chạy hai node patrol cùng lúc. `SUCCEEDED` theo dung sai Nav2 (0,25 m / 0,25 rad), không có nghĩa sai số bằng không. Quan sát Gazebo để kiểm tra không va chạm; log thành công riêng lẻ chưa chứng minh điều đó.

## 7. Pipeline, node và topic cần hiểu

| Thành phần | Vai trò | Giao tiếp |
|---|---|---|
| Simulator/bridge | RGB, LiDAR, odom, nhận vận tốc | `/oakd/rgb/preview/image_raw`, `/scan`, `/odom`, `/clock`, `/cmd_vel` |
| Map Server | Nạp YAML/PGM tuần trước | `/map` |
| AMCL | Localization bằng map + scan + odom/TF | `/initialpose`, `/amcl_pose`, TF map -> odom |
| `navigate_goals` | Đọc x/y/yaw, gửi và chờ 3 action tuần tự | `/navigate_to_pose`, `/dock_status`, lifecycle/TF |
| Nav2 | BT điều phối; Navfn/Dijkstra lập đường; DWB điều khiển và tránh vật cản qua costmap | `/plan`, `/cmd_vel_nav`, `/cmd_vel` |
| `yolo_detector` | cv_bridge -> OpenCV BGR -> YOLOv8n -> box/class/confidence | RGB -> `/yolo/image`, `/yolo/detections` |
| RViz2 | Xem robot/map/LiDAR/path/RGB/YOLO; đặt initial pose trước khi chạy code | Các topic và TF ở trên |

YOLO hiện dùng để **quan sát/nhận diện**, không tự đổi route theo class. Tránh vật cản do Nav2 và LiDAR/costmap thực hiện. AMCL khác SLAM: bài này chỉ nạp map và định vị, không xây lại map. Odometry có trôi; AMCL hiệu chỉnh bằng đối chiếu scan/map.

## 8. Quay video và nội dung nộp

Để trình bày rõ từng phần của đề, dùng các lệnh riêng ở mục 3–6: simulator → tạo vật thể → localization và initial pose → khảo sát/đối chiếu tọa độ trong YAML → YOLO → undock → chạy node `navigate_goals`. Chỉ khởi chạy node điều hướng sau khi scan khớp map, Nav2 active và có ảnh camera. Khi dùng trình tự này, không chạy thêm `demo.launch.py` hoặc một node patrol thứ hai.

Quay từ lúc load bản đồ: cho thấy đúng map tuần trước và robot Standard ở maze, initial pose và scan khớp tường. Với cách một lệnh ở đầu README, `prepare_robot` tự đặt initial pose trước khi rời dock và tự gọi node điều hướng; không cần bấm lại các công cụ RViz. Cho thấy file 3 tọa độ, camera YOLO và Terminal log trong quá trình robot tự đi. Ghi đủ 3 lần `SUCCEEDED`, toàn chuỗi hoàn thành và có box/class/confidence trên vật thể thật trong Gazebo. Không dùng video tuần trước thay cho chuỗi goal bằng code tuần này.

Upload video mới lên Google Drive, chia sẻ công khai/người có link xem được. Source nộp GitHub Public ở nhánh [assignments_5](https://github.com/ducduy1011/TuongTacNguoi_Robot/tree/assignments_5) gồm cả hai package (Bài 05 phụ thuộc phần simulator/Nav2 Bài 04), launch/config, hai file map, tools chuẩn bị và README. Các thư mục sinh ra không nằm trong repository. Nhánh `assignments_4` giữ bài tuần trước.

## 9. Dừng và xử lý lỗi

Dừng patrol bằng Ctrl+C trước, sau đó Terminal 5:

```bash
ros2 run turtlebot4_btth4 stop_simulation
```

Lệnh giữ map/video, dừng các process trong domain/partition của workspace gồm cả node Bài 05. Không dùng `killall` rộng cho dự án khác.

- Thiếu model/Python YOLO: chạy lại `python3 tools/prepare_yolo.py` sau khi source ROS. Không dùng `pip install` vào Python hệ thống.
- Không có ảnh: kiểm tra Play, camera RGB gốc, `ros2 topic list -t`; có thể truyền `image_topic:=...` cho launch perception.
- RViz camera báo chưa có ảnh YOLO: chỉ bình thường trước khi chạy node; sau đó kiểm tra `/yolo/image` và log node.
- YOLO chậm: dùng CPU như mặc định, có thể giảm `max_fps:=1.0` hoặc `image_size:=256` trong launch; giảm FPS không làm Nav2 đổi tốc độ.
- Node patrol chưa gửi goal: kiểm tra lifecycle, initial pose/scan, độ bất định AMCL, TF, thời gian mô phỏng và dock status; nó không tự sửa localization.
- Robot đứng sau khi đã đi: xem log patrol và Nav2. `No valid trajectories`/`Trajectory Hits Obstacle` là Nav2 không tìm được chuyển động an toàn; không giảm bán kính robot hoặc tắt tránh vật cản. Nếu AMCL lệch map, dừng patrol và định vị lại. Tuyến mới tránh vùng biên phía trên chưa được thể hiện đầy đủ trong raster cũ. Sau khi đổi cấu hình goal/vật thể, khởi động lại toàn phiên để dùng đúng vị trí mới; không spawn lại trên các tên cũ trong cùng phiên.
- Nav2 không có đường/scan lệch: hủy/dừng patrol, sửa localization hoặc goal/đường đi rồi chạy lại từ vị trí được xác định đúng. Không gán pose khởi đầu khi robot đã đi chỗ khác.
- Object spawn trùng: không chạy lại `objects.launch.py` trong cùng phiên; reset simulator để làm lại.

## 10. Kết quả chạy thử và kiểm tra code

Hai lượt kiểm tra Gazebo thực tế liên tiếp ngày **19/10/2026** dùng **bộ goal cũ** `(-5,-2) → (-3,-8) → (6,-8)`, mỗi lượt chạy `demo.launch.py` từ simulator mới: đủ 8 vật thể, RViz nạp map 402 × 401, AMCL/Nav2 active, tự undock, **3/3 goal SUCCEEDED** theo thứ tự. Thời gian kể từ Goal 1 đến COMPLETE lần lượt khoảng 142 và 144 giây (gồm các lần dừng 5 giây). YOLO nhận diện person/chair từ camera, độ tin cậy cao nhất giữa hai lượt khoảng 0,89/0,92. Không có lỗi navigation trong khoảng thực hiện goal và không ghi nhận hazard bump/cliff/stall/wheel-drop. Map YAML/PGM giữ nguyên. Cả 20 kiểm tra code tại thời điểm đó đều đạt.

Ngày **21/10/2026** đổi sang ba Publish Point ở mục 4. Gọi riêng action tính đường `/compute_path_to_pose` từ `(0,0)` rồi qua từng goal cho kết quả SUCCEEDED cả ba đoạn, dài khoảng 17,72 / 22,58 / 11,74 m; phép kiểm tra này không điều khiển robot. Kiểm tra nối đường offline dùng khoảng hở tường 0,45 m và vùng chừa quanh tâm vật thể 0,75 m cho tuyến mới; bài kiểm tra tuyến cũ với khoảng hở 0,65/0,90 m vẫn được giữ. Đây không phải thay đổi cấu hình tránh vật cản. Kết quả chạy thành công ngày 19/10 không chứng minh bộ goal mới đã chạy thành công; cần thực hiện và quay lại đủ ba goal mới.

Đây là kết quả của lượt thử với cấu hình đi kèm, không bảo đảm mọi máy hoặc mọi bố trí vật thể đều giống nhau. Các package gốc vẫn có thể in cảnh báo lúc khởi động; đã quan sát thông báo chờ `robot_state_publisher` rồi tự kết nối thành công. Không xem thông báo thành công của action là bằng chứng duy nhất về không va chạm.

Bộ kiểm tra action dùng server giả trong domain 147, không có Gazebo hoặc lệnh vận tốc:

```bash
cd ~/turtlebot4_btth4_ws
source install/setup.bash
ROS_DOMAIN_ID=147 python3 tests/test_navigation.py
```

Gồm 8 trường hợp: đúng thứ tự 3 goal thành công, abort dừng chuỗi, rejected dừng chuỗi, timeout/cancel, Ctrl+C/cancel, còn dock không gửi goal, AMCL có độ bất định cao không gửi goal và YAML goal sai. Kiểm tra ảnh dùng domain 148, ảnh mẫu `bus.jpg` chính thức của Ultralytics, không phải ảnh kết quả Gazebo:

```bash
ROS_DOMAIN_ID=148 .venv-yolo/bin/python tests/test_perception.py
```

Kiểm tra ảnh yêu cầu thư viện/trọng số đã chuẩn bị; xác nhận có class/box/confidence, ảnh publish giữ header và cảnh rỗng không giữ box cũ. Các kiểm tra này xác nhận logic chương trình, **không thay thế** việc tự chạy 3 goal không va chạm và xác nhận YOLO nhận được mô hình qua camera Gazebo khi quay video.

## Tài liệu

- [TurtleBot 4 Navigator](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html)
- [Nav2 Python Commander API](https://docs.nav2.org/rolling/configuration_and_development/simple_commander_api/simple_commander_api/)
- [ROS 2 Humble NavigateToPose action](https://docs.ros.org/en/humble/p/nav2_msgs/action/NavigateToPose.html)
- [Ultralytics predict](https://docs.ultralytics.com/modes/predict/)
- [Ultralytics YOLOv8](https://docs.ultralytics.com/models/yolov8/)
