# Bản đồ SLAM — TurtleBot 4 Standard

`map.yaml` và `map.pgm` được lưu từ lượt khảo sát world `maze` bằng SLAM Toolbox. YAML trỏ tương đối tới `map.pgm`; cần giữ hai file cùng thư mục.

- Kích thước ảnh: 402 × 401 ô.
- Độ phân giải: 0,05 m/ô.
- Origin: [-8.04, -10, 0]. Đây là góc bản đồ, không phải pose robot.
- Mode: trinary; occupied threshold: 0,65; free threshold: 0,196.

Sau khi build và source `env.sh`, chạy simulator ở vị trí mặc định, rồi `ros2 launch turtlebot4_btth4 navigation.launch.py`. Giữ robot tại dock, đặt `2D Pose Estimate` ở điểm khởi đầu gần (0, 0), hướng +X; kiểm tra scan khớp tường trước khi undock và đặt goal. Xem README ở gốc repository để chạy từng bước.

Nếu mapping lại, `save_map.sh` ghi đè hai file này bằng bản đồ đang publish trên `/map`. Không chạy script đó trong phiên localization/Nav2 để lưu lại bản đồ cũ.
