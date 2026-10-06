# Bài 03 — UR3e, gripper, camera và LLM

Nguyen Duc Duy — 23020731. Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress.
Bài 03 kế thừa kiến trúc Bài 02; hai workspace độc lập.

## Cấu trúc

```text
ur3_llm_week3_ws/
├── README.md
├── setup_week3.bash             # Nạp môi trường
├── run_week3.py                 # Nạp môi trường và chạy thành phần
├── run_9router.bash             # Chạy 9Router
└── src/ur3_llm_control_week3/
    ├── config/                 # Camera, vùng, controller, prompt, RViz, sinh viên
    ├── launch/                 # Mô phỏng; camera và video
    ├── scripts/                # Nhận diện, hiển thị RViz, LLM, nhập yêu cầu
    ├── src/                    # Skill server và liên kết gắp vật lý
    ├── srv/                    # ExecuteSkill, GetSceneState
    ├── urdf/                   # UR3e và gripper
    ├── worlds/                 # Bàn, 5 khối, 3 vùng, ô tạm, camera
    ├── CMakeLists.txt
    └── package.xml
```

`build/`, `install/` được colcon tạo; `log/`, `tmp/` chứa dữ liệu chạy.
`private/ninerouter.env` chứa cấu hình API cho planner, không chia sẻ hoặc đưa vào Git.

## Cài đặt và build

Yêu cầu Ubuntu 22.04 đã cài ROS 2 Humble; 9Router dùng Node.js >=18.
Trên máy chưa có workspace, tải đúng nhánh Bài 03:

```bash
git clone --branch assignments_3 --single-branch \
  https://github.com/ducduy1011/TuongTacNguoi_Robot.git ~/ur3_llm_week3_ws
cd ~/ur3_llm_week3_ws
sudo apt update
sudo apt install build-essential python3-colcon-common-extensions python3-rosdep \
  ignition-fortress gir1.2-gtk-3.0
```

Nếu chưa khởi tạo rosdep, chạy `sudo rosdep init` một lần. Sau đó:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
npm install -g 9router@0.5.91
```

Tạo cấu hình riêng cho planner, thay `YOUR_9ROUTER_API_KEY` bằng API key
của 9Router trong dashboard. File này được `.gitignore` loại khỏi Git:

```bash
mkdir -p private
cat > private/ninerouter.env <<'EOF'
export NINEROUTER_URL='http://localhost:20128/v1'
export NINEROUTER_MODEL='cx/gpt-5.6-luna'
export NINEROUTER_KEY='YOUR_9ROUTER_API_KEY'
EOF
```

Build workspace:

```bash
cd ~/ur3_llm_week3_ws
source setup_week3.bash
colcon build --base-paths src
```

`run_week3.py` tự nạp môi trường. Mỗi lệnh chạy trực tiếp trong terminal của nó.

## Chạy trong 6 terminal

**1. Gazebo, RViz, MoveIt và controller** — chờ ba controller `activated`.

```bash
~/ur3_llm_week3_ws/run_week3.py sim
```

**2. Camera và video** — nhận diện và cửa sổ video chạy chung một node.

```bash
~/ur3_llm_week3_ws/run_week3.py camera
```

Q/Esc, đóng cửa sổ hoặc Ctrl+C đều dừng camera.
Thêm `viewer:=false` nếu cần chạy nhận diện không mở cửa sổ.

**3. Skill server** — chờ `Skill server ready`.

```bash
~/ur3_llm_week3_ws/run_week3.py skills
```

**4. 9Router**

```bash
~/ur3_llm_week3_ws/run_week3.py router
```

Dùng lệnh `9router` cài sẵn và dữ liệu `~/.9router` như Bài 02.
Mở `http://localhost:20128/dashboard` để kết nối tài khoản Codex
và tạo API key khi chạy lần đầu.
Planner gọi `http://localhost:20128/v1/chat/completions`, Bearer key,
model mặc định `cx/gpt-5.6-luna`, `temperature=0`, timeout 60 giây.
Nếu báo `token_revoked`, mở `/dashboard/providers/codex`, chọn **Add Connection**
và hoàn tất xác thực tới khi router báo **Connected Successfully!**.

**5. LLM planner** — xem kế hoạch và kết quả tại đây.

```bash
~/ur3_llm_week3_ws/run_week3.py planner
```

**6. Nhập yêu cầu**

```bash
~/ur3_llm_week3_ws/run_week3.py cli
```

Ban đầu 5 khối nằm trên bàn, 3 vùng trống. Tại `COMMAND >`, nhập
`Put the blue cube in zone B.` để tạo tình huống B bị chiếm và chờ kết quả.
Sau đó nhập `Put the red cube in zone B.`: robot chuyển xanh sang ô tạm trước,
đặt đỏ vào B rồi về home.
Chờ `TASK SUCCESS` / `TASK FAILED` rồi mới gửi yêu cầu tiếp theo.
Dừng từng terminal bằng Ctrl+C; với 9Router, chọn **Exit** trong menu
để dừng cả máy chủ. Đóng Gazebo/RViz dừng mô phỏng;
các terminal khác được dừng riêng. Trước khi mở lại theo thứ tự 1–6,
nếu tiến trình cũ còn chạy, xem PID rồi kill đúng tiến trình của Bài 3:

```bash
ps -u "$USER" -o pid,comm,args
kill -9 PID
fuser -k 20128/tcp
```

Thay `PID` bằng số của tiến trình cần dừng; lệnh cuối giải phóng cổng 9Router.
Nếu dừng khi đang giữ vật, khởi động lại mô phỏng và các node để làm lại từ đầu.

## Thiết kế và luồng xử lý

Ngôn ngữ tự nhiên → LLM → kế hoạch JSON → validator → skills → MoveIt 2 → UR3e + gripper.

- Bàn 1.10 × 0.90 m, mặt bàn z=0; 5 khối cạnh 50 mm rải ngoài các vùng, cách gốc robot 28–45 cm.
- A/B/C: x=0.43, y=0.20/0/-0.20; ô tạm: (0.25, 0), cách tâm vùng gần nhất 18 cm. Camera nhìn xuống từ (0.33, 0, 1.25), ảnh 800×800.
- Gripper thân 100 mm, khe mở 80 mm. Plugin kiểm tra vật giữa hai ngón đóng, tạo liên kết vật lý rồi nhả khi mở; không sửa pose vật để gắp.
- Camera lấy tâm mặt trên của khối và xác định vùng trống/bị chiếm. Vật mất dấu hoặc ảnh cũ không được coi là trống; nếu bị tay che, robot chuyển sang tư thế quan sát. Vật chặn được chuyển sang ô tạm; nếu ô tạm bị chiếm, dùng một zone khác đang trống.
- Skill kiểm tra điểm dừng TCP; place chỉ thành công khi camera xác nhận vật lệch tâm vùng không quá 2.5 mm.
- LLM chỉ chọn `pick`, `place`, `home` và tham số. JSON, thứ tự hành động và trạng thái vùng được kiểm tra trước khi chạy; MoveIt tạo quỹ đạo và kiểm tra va chạm.
- RViz dùng khớp và pose thực của Gazebo để hiển thị; dữ liệu pose này không được dùng cho camera hoặc lập kế hoạch.
- `moveit_shutdown.cpp` sửa thứ tự giải phóng plugin của MoveIt Humble khi tắt;
  chỉ nạp cho `move_group`, không thay đổi ROS hoặc workspace Bài 2.

## Kiểm tra

Sau khi source `setup_week3.bash`:

```bash
ros2 control list_controllers
ros2 service call /get_scene_state ur3_llm_control_week3/srv/GetSceneState '{}'
ros2 service call /physical_grasp/status std_srvs/srv/Trigger '{}'
```

Ảnh: `/camera/color/image_raw`; ảnh chú thích: `/camera/color/image_annotated`.
Không gọi skill thủ công khi LLM đang chạy.
