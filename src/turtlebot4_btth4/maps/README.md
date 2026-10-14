# Bản đồ SLAM — TurtleBot 4 Standard

`map.yaml` và `map.pgm` được lưu khi khảo sát world `maze` bằng SLAM Toolbox. Giữ hai file cùng thư mục.

- Kích thước: 402 × 401 ô; độ phân giải: 0,05 m/ô.
- Origin: [-8.04, -10, 0], là góc bản đồ, không phải pose robot.
- Mode: trinary; occupied threshold: 0,65; free threshold: 0,196.

Sau khi cài/build và source `install/setup.bash`, khởi động simulator ở vị trí mặc định, rồi chạy `ros2 launch turtlebot4_btth4 navigation.launch.py`. Đặt 2D Pose Estimate ở điểm khởi đầu gần (0,0), hướng +X khi robot còn ở dock. Kiểm tra scan khớp tường trước khi undock và đặt goal. Xem README ở gốc để chạy từng bước.

Nếu mapping lại, dùng trực tiếp `ros2 run nav2_map_server map_saver_cli` theo hướng dẫn README. Lệnh lưu ghi đè cả YAML/PGM bằng map đang publish trên `/map`.
