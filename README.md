# BTTH2 - Điều khiển UR3e bằng ngôn ngữ tự nhiên và LLM

## Sinh viên

- Họ và tên: Nguyễn Đức Duy
- MSSV: 23020731
- GitHub: ducduy1011

## Mô tả

Bài thực hành xây dựng hệ thống điều khiển robot UR3e bằng câu lệnh ngôn ngữ tự nhiên.

Kiến trúc:

```text
Natural Language Command
        ↓
     LLM Planner
        ↓
  Structured Plan
        ↓
   Robot Skills
        ↓
     MoveIt 2
        ↓
      UR3e
```

LLM chỉ thực hiện phân tích yêu cầu và sinh kế hoạch mức tác vụ.
LLM không sinh joint trajectory và không điều khiển trực tiếp các khớp robot.

## Robot Skills

```text
home()
pick(object)
place(object, zone)
```

Objects:

```text
red_cube
yellow_cube
blue_cube
```

Zones:

```text
zone_a
zone_b
zone_c
```

## Cấu trúc thư mục

```text
BTTH2/
├── prompt/
│   └── system_prompt.txt
└── ur3_llm_control/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   ├── prompt.txt
    │   └── student_config.yaml
    ├── launch/
    │   └── sim_moveit.launch.py
    ├── scripts/
    │   ├── llm_task.py
    │   └── user_command_cli.py
    ├── src/
    │   └── skill_server.cpp
    ├── srv/
    │   └── ExecuteSkill.srv
    └── worlds/
        └── ur3_task.sdf
```

## Môi trường

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- Gazebo
- RViz2
- Python 3
- 9Router

## Build

```bash
cd ~/ur3_llm_ws
source /opt/ros/humble/setup.bash

colcon build \
--symlink-install \
--packages-select ur3_llm_control

source install/setup.bash
```

## Chạy mô phỏng

### Terminal 1 - Gazebo + MoveIt + RViz

```bash
source /opt/ros/humble/setup.bash
source ~/ur3_llm_ws/install/setup.bash

ros2 launch ur3_llm_control sim_moveit.launch.py
```

### Terminal 2 - Gazebo Bridge

```bash
source /opt/ros/humble/setup.bash
source ~/ur3_llm_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
/world/ur3_task/set_pose@ros_gz_interfaces/srv/SetEntityPose
```

### Terminal 3 - Skill Server

```bash
source /opt/ros/humble/setup.bash
source ~/ur3_llm_ws/install/setup.bash

ros2 run ur3_llm_control skill_server \
--ros-args -p use_sim_time:=true
```

### Terminal 4 - LLM Planner

```bash
source /opt/ros/humble/setup.bash
source ~/ur3_llm_ws/install/setup.bash
source ~/.ur3_llm_env

ros2 run ur3_llm_control llm_task.py
```

### Terminal 5 - Nhập lệnh tự nhiên

```bash
source /opt/ros/humble/setup.bash
source ~/ur3_llm_ws/install/setup.bash

python3 ~/ur3_llm_ws/src/ur3_llm_control/scripts/user_command_cli.py
```

Ví dụ:

```text
Đưa khối đỏ vào vùng B
```

Kế hoạch mong đợi:

```text
pick(red_cube)
place(red_cube, zone_b)
home()
```

## MSSV

MSSV:

```text
23020731
```

Hai chữ số cuối:

```text
XX = 31
```

Tính:

```text
P = 31 mod 6 = 1
```

Ánh xạ:

```text
Zone A -> red_cube
Zone B -> blue_cube
Zone C -> yellow_cube
```

Lệnh nâng cao:

```text
Hãy sắp xếp tất cả các vật theo mã sinh viên của tôi
```

Kế hoạch mong đợi:

```text
pick(red_cube)
place(red_cube, zone_a)

pick(blue_cube)
place(blue_cube, zone_b)

pick(yellow_cube)
place(yellow_cube, zone_c)

home()
```

## Xử lý vùng đã có vật

Nếu vùng đích đã có vật:

- vật đầu tiên nằm tại tâm vùng;
- vật thứ hai được đặt bên trái;
- nếu bên trái đã có vật, vật thứ ba được đặt bên phải.

Các vật được đặt song song với nhau.

## Prompt

Prompt đang được chương trình sử dụng:

```text
BTTH2/ur3_llm_control/config/prompt.txt
```

Bản prompt dành cho phần nộp bài:

```text
BTTH2/prompt/system_prompt.txt
```

## Lưu ý

Không đưa API key lên GitHub.

Không upload:

```text
build/
install/
log/
```
