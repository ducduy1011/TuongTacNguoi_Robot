# BTTH2 - Hướng dẫn chạy

## Build

```bash
cd ~/ur3_llm_ws
source /opt/ros/humble/setup.bash

colcon build \
--symlink-install \
--packages-select ur3_llm_control

source install/setup.bash
```

## Terminal 1 - Gazebo + MoveIt + RViz

```bash
source /opt/ros/humble/setup.bash
source ~/ur3_llm_ws/install/setup.bash

ros2 launch ur3_llm_control sim_moveit.launch.py
```

## Terminal 2 - Gazebo Bridge

```bash
source /opt/ros/humble/setup.bash
source ~/ur3_llm_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
/world/ur3_task/set_pose@ros_gz_interfaces/srv/SetEntityPose
```

## Terminal 3 - Skill Server

```bash
source /opt/ros/humble/setup.bash
source ~/ur3_llm_ws/install/setup.bash

ros2 run ur3_llm_control skill_server \
--ros-args -p use_sim_time:=true
```

## Terminal 4 - LLM Planner

Trước khi chạy, cấu hình 9Router:

```bash
source ~/.ur3_llm_env
```

Sau đó:

```bash
source /opt/ros/humble/setup.bash
source ~/ur3_llm_ws/install/setup.bash
source ~/.ur3_llm_env

ros2 run ur3_llm_control llm_task.py
```

## Terminal 5 - Nhập lệnh

```bash
source /opt/ros/humble/setup.bash
source ~/ur3_llm_ws/install/setup.bash

python3 ~/ur3_llm_ws/src/ur3_llm_control/scripts/user_command_cli.py
```

Ví dụ:

```text
COMMAND > Đưa khối đỏ vào vùng B
```

```text
COMMAND > Trở về vị trí home
```

```text
COMMAND > Hãy sắp xếp tất cả các vật theo mã sinh viên của tôi
```

Nhập:

```text
exit
```

để thoát chương trình nhập lệnh.
