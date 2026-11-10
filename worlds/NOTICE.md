`warehouse.sdf` được tạo từ world của `turtlebot4_ignition_bringup`
(Clearpath Robotics, Apache-2.0):
https://github.com/turtlebot/turtlebot4_simulator/blob/humble/turtlebot4_ignition_bringup/worlds/warehouse.sdf

Thay URI Fuel bằng đường dẫn cache hiện có trên máy; bỏ include
`Person 2 - Walking` (Casual female) vì tải qua mạng bị chậm.
Thêm hệ thống Sensors với `render_engine=ogre2` ở cấp world để sửa
laser bị kẹt toàn bộ ở giá trị tối thiểu 0,164 m với bộ dựng Ogre cũ.
Tham khảo: https://github.com/turtlebot/turtlebot4/issues/676
Các model tham chiếu giữ giấy phép riêng của tác giả, không được sao chép vào bài.
Giấy phép mã world: xem `../maps/LICENSE`.
