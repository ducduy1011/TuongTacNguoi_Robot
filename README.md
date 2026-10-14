# BTTH04 — TurtleBot 4 Standard: mapping, localization và Nav2

Ubuntu 22.04, ROS 2 Humble Desktop, Gazebo Fortress, TurtleBot 4 **Standard** và world `maze` chính thức. Launch chỉ chấp nhận `model:=standard`. Robot có shell, tháp cảm biến, RPLIDAR và OAK-D-Pro.

Workspace đã được làm sạch để bạn tự thực hiện lượt quay mới. Thư mục `maps/` chưa có bản đồ. Sau khi khảo sát, bạn phải lưu và upload `map.yaml` + `map.pgm`; không có kết quả navigation được điền sẵn.

## Cài và build

Máy khác cần Ubuntu 22.04, ROS 2 Humble Desktop, ROS apt repository, Internet và phiên desktop có GPU hoạt động.

```bash
git clone --branch assignments_4 --single-branch \
  https://github.com/ducduy1011/TuongTacNguoi_Robot.git ~/turtlebot4_btth4_ws
cd ~/turtlebot4_btth4_ws
bash install_dependencies.sh
bash setup_runtime.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select turtlebot4_btth4
```

Trên máy hiện tại đã build xong. Không cần cài/build lại trước mỗi lượt chạy.

Phiên bản đã kiểm tra: TurtleBot 4 Simulator 1.0.3, SLAM Toolbox 2.6.10, Nav2 1.1.20, RViz 11.2.29 và Fortress 1.0.3. Cài/build từ thư mục source sạch đã thành công trên máy Ubuntu 22.04 này.

`setup_runtime.sh` chuẩn bị các thư viện tại `.deps/`: TF2 chính thức 0.25.22 tại commit `65c620c308920f558c7b5d3fb852941bd6d8fced`, và solver Ceres của SLAM Toolbox 2.6.10 tại commit `8293d21fe5c816d0405e0cc1f3eb46c70659c267`. Solver giữ thuật toán chính thức, chỉ giới hạn số luồng từ 50 xuống tối đa 4, theo số luồng CPU có sẵn. Điều này xử lý cảnh báo quá số luồng trên máy triển khai. Cấu hình Create 3 dùng controller 100 Hz phù hợp bước vật lý của world maze. DDS sử dụng Fast DDS của ROS Humble với UDP loopback và publish bất đồng bộ, cấu hình trong `config/fastdds.xml` cho việc chạy trên một máy. RViz navigation giữ thư viện plugin Nav2 được nạp đến khi các callback ROS hủy xong để tránh lỗi đóng cửa sổ. Thư viện hệ thống không bị sửa.

Các thư mục `.deps/`, `build/`, `install/`, `log/` được tạo trên máy chạy; không nộp lên GitHub. Giữ `.deps/` để chạy. RViz dùng 20 FPS, chỉ hiện TF cần thiết; bản cấu hình mapping không mở costmap/Nav2.

## Chuẩn bị quay

Mở **4 Terminal**. Trong mỗi Terminal mới chạy:

```bash
source ~/turtlebot4_btth4_ws/env.sh
```

Sau khi đổi middleware lần đầu, trong Terminal 4 chạy `ros2 daemon stop` để bỏ cache ROS CLI của phiên cũ.

Mọi Terminal phải cùng domain 44 và Fast DDS. Trước lượt chạy mới, trong Terminal 4 chạy:

```bash
bash ~/turtlebot4_btth4_ws/stop_btth4.sh
```

Chờ thông báo đã dừng. Lệnh chỉ dừng tiến trình BTTH04 thuộc domain/partition của workspace; giữ nguyên file map đã lưu. Không mở hai simulator hoặc chạy SLAM và AMCL cùng lúc.

Trong Ubuntu có thể nhấn **Ctrl+Alt+Shift+R** để bắt đầu/dừng quay màn hình; nếu GNOME hiện bảng chọn, chọn vùng có cả Gazebo và RViz, rồi nhấn Record. Bố trí hai cửa sổ cạnh nhau, phóng đủ lớn để thấy robot. Robot Standard phải có tháp cảm biến. Không dùng nút teleop trong Gazebo khi Nav2 chạy goal.

## Phần 1 và 2 — Simulation, kiểm tra và xây dựng map

### Terminal 1 — Gazebo

```bash
ros2 launch turtlebot4_btth4 simulation.launch.py
```

Chờ khởi động xong (có thể mất 30–60 giây ở lượt đầu), Standard xuất hiện trong world `maze`, mô phỏng ở trạng thái Play. Robot spawn tại x=-2, y=0 trong hệ tọa độ Gazebo, ban đầu ở dock.

### Terminal 2 — SLAM Toolbox và RViz

```bash
ros2 launch turtlebot4_btth4 mapping.launch.py
```

RViz tự mở với Fixed Frame `map`. Quan sát RobotModel, LaserScan, Map và các frame `map/odom/base_link`. Giữ robot tại vị trí ban đầu cho tới khi có map và TF.

### Terminal 4 — kiểm tra trước khi di chuyển

Chạy lần lượt:

```bash
ros2 control list_controllers
ros2 topic echo /clock --once
ros2 topic echo /scan --once --qos-reliability best_effort --field header
ros2 topic echo /odom --once --qos-reliability best_effort --field pose.pose
ros2 topic info /cmd_vel
ros2 param get /slam_toolbox use_sim_time
ros2 topic echo /map --once --qos-durability transient_local --qos-reliability reliable --field info
ros2 run tf2_ros tf2_echo map base_link --ros-args -p use_sim_time:=true
```

Hai controller phải `active`; scan có dữ liệu, odom có pose, `/cmd_vel` có subscriber, `use_sim_time=true`, kích thước map khác 0 và TF hợp lệ. **Ghi lại x, y, yaw của robot trên map trước khi di chuyển**, dùng khi localization. Ctrl+C chỉ dừng lệnh tf2_echo.

Không lấy x=-2, y=0 của Gazebo làm initial pose trên map: đó là hai hệ tọa độ khác nhau. Lượt mapping bắt đầu tại dock thường cho pose map gần (0,0), yaw≈0; dùng số bạn đã ghi để xác nhận.

### Terminal 3 — rời dock và teleop WASD

```bash
ros2 topic echo /dock_status --once
```

Chỉ khi `is_docked: true` và simulator đã sẵn sàng, chạy:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'
ros2 param set /motion_control safety_override backup_only --no-daemon --spin-time 3
```

Chờ undock `SUCCEEDED`, rồi:

```bash
bash ~/turtlebot4_btth4_ws/teleop_wasd.sh
```

| Phím | Chức năng |
|---|---|
| W / S | Tiến / lùi |
| A / D | Quay trái / phải |
| Space / X | Dừng ngay |
| Ctrl+C | Dừng và thoát |

Giữ focus ở Terminal 3. Giữ hoặc nhấn liên tiếp phím. Teleop phát lệnh 10 Hz; tốc độ tiến/lùi 0,5 m/s, quay 0,5 rad/s. Không có phím mới trong 0,8 s thì tự phát lệnh dừng. Nhấn Space trước khi chuyển cửa sổ.

Script tự đặt `safety_override=backup_only` mỗi lần mở và chuyển bộ gõ IBus sang English nếu đang dùng Unikey/bộ gõ khác; khôi phục bộ gõ khi thoát. Terminal hiện `Key W/A/S/D` để xác nhận nhận đúng phím. 0,5 m/s là lệnh WASD; Create 3 vẫn áp dụng giới hạn tốc độ của chế độ an toàn (trên cấu hình hiện tại `max_speed=0.306`) và controller bánh xe.

Nếu W làm robot tiến rồi lùi, xem `/hazard_detection` và log motion control: reflex va chạm có thể đang ưu tiên điều khiển và bỏ qua WASD. Nhấn Space, đợi reflex kết thúc; quay A/D hướng ra vùng trống trước khi W, tránh chạy thẳng trở lại dock sau undock. Không vô hiệu hóa reflex va chạm để vượt qua vật cản.

Để kiểm tra robot nhận lệnh, trong Terminal 4 có thể chạy `ros2 topic echo /cmd_vel`, quan sát khi bạn nhấn phím rồi Ctrl+C dừng echo. Đọc lại odom và đối chiếu robot thật sự di chuyển trong Gazebo.

Khảo sát phần lớn các hành lang và nhánh có thể đi. Quay chậm ở góc, tránh chạm tường, trở lại khu vực đã đi để hỗ trợ loop closure. Theo dõi scan và map trên RViz. Map đạt yêu cầu khi tường, hành lang và vật cản chính rõ, không có tường nhân đôi hoặc vùng lệch lớn. Vùng trắng là free, đen là occupied, xám là unknown.

### Terminal 4 — lưu bản đồ hoàn chỉnh

Dừng robot bằng Space, Ctrl+C ở Terminal 3. Giữ Gazebo và SLAM chạy, chờ map cập nhật vài giây rồi:

```bash
bash ~/turtlebot4_btth4_ws/save_map.sh
ls -lh ~/turtlebot4_btth4_ws/src/turtlebot4_btth4/maps/map.yaml \
       ~/turtlebot4_btth4_ws/src/turtlebot4_btth4/maps/map.pgm
cat ~/turtlebot4_btth4_ws/src/turtlebot4_btth4/maps/map.yaml
```

Hai file phải khác 0 byte, YAML trỏ tới `map.pgm`. Chụp **map hoàn chỉnh trên RViz** và quay vài giây robot + map trước khi dừng SLAM.

## Phần 3 — restart và localization

Terminal 3 teleop phải đã thoát và map đã lưu. Trong Terminal 4 chạy:

```bash
bash ~/turtlebot4_btth4_ws/stop_btth4.sh
```

Chờ các cửa sổ đóng rồi khởi động lại dưới đây. Dùng lệnh này cũng khi cần reset toàn bộ hoặc Ctrl+C không phản hồi.

### Terminal 1 — cùng simulator Standard

```bash
ros2 launch turtlebot4_btth4 simulation.launch.py
```

### Terminal 2 — load map, AMCL và RViz

```bash
ros2 launch turtlebot4_btth4 navigation.launch.py nav2:=false
```

Launch tự đọc map mới trong thư mục source khi build bằng symlink. Nếu chưa có map hoặc YAML/ảnh không hợp lệ, launch hướng dẫn lưu map và kết thúc trước khi mở node.

Giữ robot ở dock. Dùng **2D Pose Estimate** trong RViz: click vị trí ban đầu trên map, kéo mũi tên theo yaw đã ghi. Đợi scan khớp với các tường và robot ở đúng khu vực/hướng như Gazebo.

### Terminal 4 — kiểm tra localization

```bash
ros2 node list
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo /map --once --qos-durability transient_local --qos-reliability reliable --field info
ros2 topic echo /amcl_pose --once --qos-durability transient_local --qos-reliability reliable --field pose.pose
ros2 run tf2_ros tf2_echo map base_link --ros-args -p use_sim_time:=true
```

Không có `/slam_toolbox`. Map Server và AMCL `active`; map đúng bản vừa lưu, pose/TF hợp lệ; scan khớp tường. Quay lại thao tác load map, đặt initial pose và so sánh Gazebo/RViz. Ctrl+C dừng tf2_echo.

## Phần 4 — Nav2 và ít nhất 3 goal

Giữ Gazebo ở Terminal 1. Ctrl+C dừng riêng launch localization ở Terminal 2, rồi:

```bash
ros2 launch turtlebot4_btth4 navigation.launch.py
```

Launch mở Map Server, AMCL và RViz trước. Đặt lại **2D Pose Estimate** vì AMCL vừa khởi động lại; đợi scan khớp map. Khi đã có `/amcl_pose` và TF `map → base_link`, launch tự mở Nav2 và báo `AMCL/TF ready; starting Nav2`. Trước đó các node planner/controller chưa xuất hiện là bình thường. Giữ robot đứng yên trong bước này.

Terminal 4:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
```

Các node phải `active`. Kiểm tra dock status; nếu còn docked, rời dock bằng các lệnh undock/backup_only như phần 1. Chờ thành công **trước khi đặt goal**. Không mở teleop.

Dùng **Nav2 Goal** trên RViz, chọn một vùng free đủ rộng và kéo mũi tên theo hướng đích. Thực hiện lần lượt:
1. G1 ở một nhánh/khu vực trên map.
2. G2 ở một khu vực khác, có đường đi đã được khảo sát.
3. G3 ở khu vực thứ ba.

Chừa khoảng hở với tường, tránh vùng unknown, vật cản và dock. Chờ mỗi goal hoàn tất rồi mới gửi goal tiếp. Quan sát Path/Costmap, robot trong Gazebo và kết quả action. Sau khi đặt goal không điều khiển robot thủ công. Nếu goal lỗi, hủy goal, kiểm tra map/pose/costmap rồi chọn lại; không tính lần đó là thành công.

Theo dõi đúng goal mới, không lấy trạng thái của goal cũ:

```bash
ros2 topic echo /navigate_to_pose/_action/status action_msgs/msg/GoalStatusArray \
  --qos-durability transient_local --qos-reliability reliable
```

`status: 2` đang chạy, `4` SUCCEEDED, `5` canceled, `6` aborted. Quay toàn bộ robot tự di chuyển tới ba đích. Kết quả thành công phải đi kèm robot tới đích thực tế và không va chạm.

Có thể theo dõi bumper/stall ở Terminal khác:

```bash
ros2 topic echo /hazard_detection --qos-reliability best_effort \
  --filter "any(h.type in (1, 3) for h in m.detections)"
```

Lệnh yên khi không có sự kiện; vẫn quan sát Gazebo.

Khi quay xong, nhấn dừng quay rồi chạy `bash ~/turtlebot4_btth4_ws/stop_btth4.sh` ở Terminal 4 để dừng phiên.

## Những thứ cần nộp sau lượt chạy mới

- Video Standard: mapping, map hoàn chỉnh trên RViz, restart/load map, localization và ít nhất 3 goal ở các khu vực khác nhau, không va chạm/teleop khi goal chạy.
- Ảnh: robot Standard; mapping; map hoàn chỉnh; localization; các goal đã hoàn tất.
- GitHub Public nhánh `assignments_4`: source/launch/config/README và **map.yaml + map.pgm mới**.
- PDF dùng đúng ảnh và số liệu của lượt Standard. Điền tọa độ/độ phân giải/kích thước map thực tế, kết quả goal, link video và GitHub.

Video mới có thể đặt trong thư mục Drive bạn đã tạo. Xóa video minh chứng không dùng nữa ở Drive và bật chia sẻ **Anyone with the link / Viewer**. Hiện máy chỉ có kết nối GitHub; việc dọn video trên Drive do bạn thực hiện.

Đưa map mới lên GitHub bằng:

```bash
cd ~/turtlebot4_btth4_ws
git add src/turtlebot4_btth4/maps/map.yaml src/turtlebot4_btth4/maps/map.pgm
git commit -m "Add map built with TurtleBot 4 Standard"
git -c credential.helper= -c 'credential.helper=!gh auth git-credential' push origin assignments_4
```

Hoặc báo đã lưu map để agent cập nhật. Tải video lên Drive, kiểm tra link khi chưa đăng nhập; chỉ nộp PDF sau khi thay đủ ảnh/kết quả và link.

## Kết quả kiểm tra và phạm vi

Build, các phím WASD/dừng tự động, dữ liệu scan/odom/clock, SLAM, lưu/load map thử và sáu node localization/Nav2 đã được kiểm tra ở domain riêng. Robot giữ tại dock, không gửi lệnh vận tốc/undock/navigation goal tới phiên demo. Map thử không đưa vào thư mục source. Chất lượng map toàn môi trường và ba goal thành công phải được xác nhận bằng lượt chạy của bạn.

Package chính thức có thể báo chờ robot_state_publisher hoặc thiếu TF ngay lúc khởi động; chỉ tiếp tục khi controller/topic/TF đã sẵn sàng. Nếu các kiểm tra ở từng phần không đạt hoặc cảnh báo mất dữ liệu tiếp tục khi robot đã đứng yên và khởi động xong, dừng để kiểm tra, không ghi nhận kết quả là thành công.

## Cấu trúc và pipeline

`simulation.launch.py`: Standard/Gazebo; `mapping.launch.py`: SLAM + RViz; `navigation.launch.py`: kiểm tra map rồi mở AMCL; `wait_localization.py` chờ pose/TF trước khi mở Nav2. `teleop_wasd.py` là publisher bàn phím nhỏ, không phải controller bánh xe.

Mapping: scan + odom/TF → SLAM Toolbox → map và TF map→odom.
Localization: map.yaml/PGM + scan/odom → Map Server + AMCL → pose và TF map→odom.
Navigation: RViz Goal → BT Navigator → Navfn (Dijkstra) + costmap → DWB → velocity smoother → /cmd_vel → motion control và diffdrive controller.

Nav2 dùng radius 0,25 m, inflation 0,60 m, vận tốc tối đa 0,26 m/s và goal tolerance 0,25 m/0,25 rad. `SUCCEEDED` nghĩa đạt dung sai, không phải sai số bằng không.

Tài liệu chính thức: [Simulator](https://github.com/turtlebot/turtlebot4_simulator),
[User Manual](https://turtlebot.github.io/turtlebot4-user-manual/),
[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/2.6.10).
