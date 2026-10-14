# Bài thực hành 04 — TurtleBot 4 Simulation

Sử dụng **TurtleBot 4 Standard**, Gazebo Fortress và world `maze` chính thức để xây dựng bản đồ bằng SLAM Toolbox, sau đó localization bằng AMCL và điều hướng bằng Nav2.

Repository chứa source Python, launch ROS 2, cấu hình, `map.yaml`, `map.pgm` và hướng dẫn chạy. Không có script Bash tự viết. Các lệnh dưới đây được nhập trực tiếp trong Terminal; `setup.bash` là môi trường chuẩn do ROS 2/colcon cung cấp.

## 1. Cấu trúc chương trình

```text
tools/
  prepare_runtime.py          # Chuẩn bị thư viện tương thích trên máy chạy
runtime/slam_solver/
  CMakeLists.txt              # Build solver chính thức với giới hạn số luồng
src/turtlebot4_btth4/
  package.xml
  CMakeLists.txt
  launch/
    simulation.launch.py     # TurtleBot 4 Standard và Gazebo
    mapping.launch.py        # SLAM Toolbox và RViz2
    navigation.launch.py     # Load map, AMCL, RViz2 và Nav2
  scripts/
    teleop_wasd.py            # Publish Twist từ bàn phím
    wait_localization.py     # Chờ pose/TF trước khi mở Nav2
    stop_simulation.py       # Dừng các tiến trình của bài thực hành
  config/                    # SLAM, RViz2, DDS và môi trường ROS
  maps/
    map.yaml
    map.pgm
```

Bản đồ đi kèm: **402 × 401 ô**, độ phân giải **0,05 m/ô**, origin **[-8.04, -10, 0]**. `map.yaml` dùng đường dẫn tương đối `image: map.pgm`; hai file phải ở cùng thư mục. Origin là tọa độ góc bản đồ, không phải vị trí ban đầu của robot.

## 2. Cài đặt và build

Yêu cầu Ubuntu 22.04, **ROS 2 Humble Desktop** và phiên desktop có OpenGL hoạt động. Nếu dùng WSL2 cần WSLg. Cài ROS trước theo [hướng dẫn Ubuntu chính thức](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), bao gồm cấu hình ROS apt repository.

Clone đúng nhánh Bài 04:

```bash
git clone --branch assignments_4 --single-branch \
  https://github.com/ducduy1011/TuongTacNguoi_Robot.git ~/turtlebot4_btth4_ws
cd ~/turtlebot4_btth4_ws
```

Cài các dependency:

```bash
sudo apt update
sudo apt install git cmake build-essential python3-pil python3-yaml \
  python3-colcon-common-extensions libceres-dev libtbb-dev qtbase5-dev \
  libboost-thread-dev libboost-serialization-dev libboost-filesystem-dev \
  ignition-fortress ros-humble-ament-cmake-gen-version-h \
  ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes \
  ros-humble-turtlebot4-navigation ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rviz2 \
  ros-humble-rclpy ros-humble-geometry-msgs ros-humble-rmw-fastrtps-cpp
source /opt/ros/humble/setup.bash
python3 tools/prepare_runtime.py
source .deps/install/setup.bash
colcon build --symlink-install --packages-select turtlebot4_btth4
source install/setup.bash
```

Chỉ chuẩn bị/build ở lần đầu hoặc sau khi sửa source. `prepare_runtime.py` tạo thư viện tại `.deps/`, không sửa `/opt/ros`: TF2 chính thức 0.25.22 tại commit `65c620c308920f558c7b5d3fb852941bd6d8fced`; solver SLAM Toolbox 2.6.10 tại commit `8293d21fe5c816d0405e0cc1f3eb46c70659c267` chỉ đổi số luồng Ceres từ 50 xuống tối đa 4; cấu hình controller Create 3 dùng 100 Hz phù hợp world maze. Các bước này giữ cách xử lý tương thích đã dùng khi chạy bài.

Khi source `install/setup.bash`, cấu hình môi trường `.dsv` của package tự đặt domain 44, Fast DDS UDP loopback, publication bất đồng bộ, partition Gazebo và đường dẫn thư viện. Các terminal cần chạy trên cùng một máy. `build/`, `install/`, `log/`, `.deps/` là thư mục được tạo lại trên máy chạy và không nộp lên GitHub.

## 3. Chuẩn bị Terminal

Mở 4 Terminal. Trong **mỗi Terminal mới** chạy:

```bash
source ~/turtlebot4_btth4_ws/install/setup.bash
```

Trước khi chạy lại từ đầu, dùng Terminal 4:

```bash
ros2 daemon stop
ros2 run turtlebot4_btth4 stop_simulation
```

Lệnh dừng chỉ xử lý các tiến trình của BTTH04 theo domain/partition, giữ nguyên bản đồ. Không mở hai simulator hoặc chạy SLAM và AMCL cùng lúc.

## 4. Simulation và xây dựng bản đồ

**Terminal 1 — Gazebo:**

```bash
ros2 launch turtlebot4_btth4 simulation.launch.py
```

Chờ robot Standard xuất hiện, mô phỏng ở trạng thái Play và controller hoạt động. Robot spawn tại x=-2, y=0 trong hệ tọa độ Gazebo, ban đầu ở dock. Launch chỉ chấp nhận `model:=standard`.

**Terminal 2 — SLAM Toolbox và RViz2:**

```bash
ros2 launch turtlebot4_btth4 mapping.launch.py
```

RViz có Fixed Frame `map`, hiển thị RobotModel, LaserScan và Map.

**Terminal 4 — kiểm tra trước khi di chuyển:**

```bash
ros2 control list_controllers
ros2 topic echo /scan --once --qos-reliability best_effort --field header
ros2 topic echo /odom --once --qos-reliability best_effort --field pose.pose
ros2 topic info /cmd_vel
ros2 param get /slam_toolbox use_sim_time
ros2 run tf2_ros tf2_echo map base_link --ros-args -p use_sim_time:=true
```

Controller phải `active`, scan/odom có dữ liệu, `/cmd_vel` có subscriber và `use_sim_time=true`. **Ghi lại x, y, yaw của robot trong hệ tọa độ map trước khi di chuyển** để dùng khi localization. Ctrl+C dừng riêng lệnh tf2_echo. Không dùng tọa độ Gazebo thay cho tọa độ map.

**Terminal 3 — rời dock và teleoperation:**

```bash
ros2 topic echo /dock_status --once
```

Nếu `is_docked: true`, chạy undock và chờ `SUCCEEDED`:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'
```

Sau đó chạy:

```bash
ros2 param set /motion_control safety_override backup_only --no-daemon --spin-time 3
ros2 run turtlebot4_btth4 teleop_wasd --ros-args \
  -p speed:=0.5 -p turn:=0.5 -p use_sim_time:=true
```

W/S tiến/lùi, A/D quay trái/phải, Space/X dừng, Ctrl+C thoát. Giữ focus tại Terminal và giữ/nhấn liên tiếp phím. Node phát Twist 10 Hz, tự dừng sau 0,8 giây không có phím mới, chuyển IBus sang English khi cần và khôi phục khi thoát. 0,5 m/s là vận tốc lệnh; Create 3 vẫn áp dụng giới hạn an toàn, `max_speed=0.306` trên cấu hình hiện tại. Quay ra vùng trống trước khi tiến để tránh dock.

Khảo sát phần lớn các nhánh/hành lang có thể đi, quay chậm ở góc và đi lại vùng đã khảo sát để hỗ trợ loop closure. Theo dõi map trên RViz: tường và vật cản chính rõ, không nhân đôi hoặc lệch lớn. Vùng trắng là free, đen là occupied, xám là unknown. Có thể đọc `/cmd_vel` ở Terminal 4 khi tự nhấn phím để kiểm tra robot nhận lệnh.

**Lưu map — Terminal 4:**

Nhấn Space rồi Ctrl+C ở Terminal teleop. Giữ Gazebo và SLAM chạy, chờ map cập nhật, rồi:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/turtlebot4_btth4_ws/src/turtlebot4_btth4/maps/map \
  --free 0.196 --occ 0.65 --fmt pgm --mode trinary \
  --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=20.0
```

Kiểm tra `map.yaml` và `map.pgm` khác 0 byte, YAML trỏ đúng PGM. Lệnh này ghi đè bản đồ đi kèm bằng map vừa khảo sát. Quan sát và quay bản đồ hoàn chỉnh trên RViz trước khi dừng SLAM.

## 5. Load map và localization

Dừng toàn bộ phiên cũ ở Terminal 4:

```bash
ros2 run turtlebot4_btth4 stop_simulation
```

**Terminal 1:** khởi động lại cùng simulator:

```bash
ros2 launch turtlebot4_btth4 simulation.launch.py
```

**Terminal 2:** load bản đồ, AMCL và RViz:

```bash
ros2 launch turtlebot4_btth4 navigation.launch.py nav2:=false
```

Giữ robot ở dock. Dùng **2D Pose Estimate** trên RViz đặt vị trí và hướng theo pose map đã ghi lúc bắt đầu mapping. Với map đi kèm và spawn mặc định, pose gần **(0,0), yaw≈0**, hướng +X. Chờ scan khớp các tường và vị trí/hướng robot tương ứng với Gazebo.

**Terminal 4:**

```bash
ros2 node list
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo /map --once --qos-durability transient_local --field info
ros2 topic echo /amcl_pose --once --qos-durability transient_local --field pose.pose
ros2 run tf2_ros tf2_echo map base_link --ros-args -p use_sim_time:=true
```

Không còn `/slam_toolbox`; Map Server/AMCL phải `active`, có pose và TF. Ctrl+C dừng tf2_echo. `nav2:=false` chỉ mở localization để kiểm tra và quay video.

## 6. Navigation bằng Nav2

Giữ Gazebo chạy. Ctrl+C dừng launch localization ở Terminal 2, rồi chạy:

```bash
ros2 launch turtlebot4_btth4 navigation.launch.py
```

Đặt lại **2D Pose Estimate** vì AMCL vừa khởi động lại. Giữ robot ở dock đến khi scan khớp tường và Terminal báo `AMCL/TF ready; starting Nav2.`

Kiểm tra ở Terminal 4:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
```

Các node phải `active`. Sau đó kiểm tra dock, undock nếu còn docked, chờ `SUCCEEDED`, đặt `safety_override=backup_only` bằng các lệnh ở Phần 4. Không mở teleop.

Thứ tự khi chạy từ đầu: **simulator → AMCL → 2D Pose Estimate/scan khớp map → Nav2 active → undock → Nav2 Goal**. Không undock trước rồi gán pose của điểm khởi đầu, cũng không đặt lại pose (0,0) khi robot đã di chuyển.

Dùng công cụ **Nav2 Goal** trên RViz. Chọn lần lượt ít nhất **3 goal ở các khu vực khác nhau**, trong vùng free đủ rộng, có đường nối với robot; tránh unknown, tường và dock. Chờ mỗi goal thành công trước khi đặt goal tiếp. Sau khi đặt goal không điều khiển thủ công. Quan sát Path, costmap và Gazebo để xác nhận robot tới đích, không va chạm.

Theo dõi trạng thái của đúng goal mới:

```bash
ros2 topic echo /navigate_to_pose/_action/status action_msgs/msg/GoalStatusArray \
  --qos-durability transient_local --qos-reliability reliable
```

`2`: executing, `4`: succeeded, `5`: canceled, `6`: aborted. Goal thất bại không tính vào ba goal thành công. Quay xong, dừng phiên bằng `ros2 run turtlebot4_btth4 stop_simulation`.

## 7. Các node, topic và pipeline chính

| Chức năng | Node/package chính | Topic chính |
|---|---|---|
| Mô phỏng, cảm biến và chuyển động | TurtleBot 4 simulator, Create 3, ros2_control | `/scan`, `/odom`, `/cmd_vel`, `/clock`, `/tf` |
| Xây dựng bản đồ | `slam_toolbox` | `/scan`, `/map`, `/tf` |
| Localization | `map_server`, `amcl` | `/map`, `/initialpose`, `/amcl_pose`, `/tf` |
| Điều hướng | `bt_navigator`, `planner_server`, `controller_server`, `velocity_smoother` | `/plan`, `/cmd_vel_nav`, `/cmd_vel`, action `/navigate_to_pose` |

Mapping: scan + odom/TF → SLAM Toolbox → map và TF map→odom.

Localization: map.yaml/PGM + scan/odom → Map Server + AMCL → pose và TF map→odom.

Navigation: RViz Goal → BT Navigator → Navfn (Dijkstra) + costmap → DWB → velocity smoother → `/cmd_vel` → motion control và diffdrive controller.

Nav2 dùng radius 0,25 m, inflation 0,60 m, tốc độ tối đa 0,26 m/s và dung sai goal 0,25 m/0,25 rad. `SUCCEEDED` nghĩa đạt dung sai, không phải sai số bằng không.

## 8. Xử lý lỗi thường gặp

- **AMCL yêu cầu initial pose:** dùng 2D Pose Estimate, giữ robot đứng yên và đợi scan khớp tường.
- **LiDAR lệch map, robot nhảy vị trí:** sửa localization trước khi gửi goal. Nếu đã undock rồi đặt pose ban đầu, dừng và khởi động lại đúng thứ tự. Không sửa origin YAML để dịch robot.
- **Không có planner/controller:** kiểm tra không dùng `nav2:=false` và đã có pose/TF.
- **Goal không có đường:** kiểm tra pose, vùng free và khoảng hở. Hủy goal trước khi sửa pose; không dùng WASD khi goal đang chạy.
- **WASD tiến rồi lùi:** nhấn Space, kiểm tra vật cản/dock và `/hazard_detection`; đợi reflex kết thúc, quay hướng ra vùng trống. Giữ các reflex an toàn.
- **Ctrl+C không phản hồi hoặc còn phiên cũ:** dùng `ros2 run turtlebot4_btth4 stop_simulation` từ Terminal đã source môi trường.

Đã kiểm tra build source sạch và các chức năng nền trên Ubuntu 22.04: simulator 1.0.3, SLAM Toolbox 2.6.10, Nav2 1.1.20, RViz 11.2.29 và Fortress 1.0.3. Kết quả mapping/goal cần đối chiếu với thao tác thực tế và video của sinh viên.

Tài liệu: [TurtleBot 4 Simulator](https://github.com/turtlebot/turtlebot4_simulator), [TurtleBot 4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/), [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/2.6.10).
