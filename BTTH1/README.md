# BTTH1 - UR3e Cartesian Trajectory

## 1. Nội dung

Bài thực hành sử dụng ROS 2 Humble, MoveIt 2, Gazebo và RViz2
để điều khiển robot UR3e thực hiện quỹ đạo trong không gian Cartesian.

Project gồm hai package:

- `ur3_letter_writer`: vẽ chữ D
- `ur3_circle_writer`: vẽ hình tròn

## 2. Cấu trúc

```text
BTTH1/
├── README.md
├── ur3_letter_writer/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── src/
│   │   └── draw_letter_d.cpp
│   ├── launch/
│   │   └── draw_letter_d.launch.py
│   ├── config/
│   │   └── letter_d.yaml
│   └── rviz/
│       └── letter_writer.rviz
│
└── ur3_circle_writer/
    ├── CMakeLists.txt
    ├── package.xml
    ├── src/
    │   └── draw_circle.cpp
    ├── launch/
    │   └── draw_circle.launch.py
    ├── config/
    │   └── circle.yaml
    └── rviz/
        └── circle_writer.rviz
```

## 3. Build

Tạo workspace:

```bash
mkdir -p ~/ur3_letter_ws/src
```

Copy hai package vào:

```text
~/ur3_letter_ws/src/
├── ur3_letter_writer
└── ur3_circle_writer
```

Build:

```bash
cd ~/ur3_letter_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
  ur3_letter_writer \
  ur3_circle_writer

source install/setup.bash
```

## 4. Chạy chữ D

```bash
cd ~/ur3_letter_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur3_letter_writer draw_letter_d.launch.py
```

## 5. Chạy hình tròn

Dừng chương trình chữ D trước nếu đang chạy.

```bash
cd ~/ur3_letter_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur3_circle_writer draw_circle.launch.py
```

## 6. Thông số chính

### Chữ D

```text
letter_height: 0.12
letter_width: 0.08
arc_points: 50
eef_step: 0.003
jump_threshold: 2.0
min_fraction: 0.999
velocity_scale: 0.12
acceleration_scale: 0.12
```

### Hình tròn

```text
circle_radius: 0.07
circle_points: 100
eef_step: 0.003
jump_threshold: 2.0
min_fraction: 0.999
velocity_scale: 0.12
acceleration_scale: 0.12
```

## 7. MoveIt 2

Planning group:

```text
ur_manipulator
```

Cartesian path được tính bằng:

```cpp
move_group.computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    trajectory,
    true);
```

Trajectory được yêu cầu đạt:

```text
fraction >= 0.999
```

## 8. Tác giả

Nguyễn Đức Duy  
MSSV: 23020731
