cmake_minimum_required(VERSION 3.8)
project(my_mobilerobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


install(DIRECTORY
  urdf
  launch
  config
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)


# ====================== [THÊM] Cài 2 node Python điều khiển robot ======================
# Trước đây package KHÔNG có node nào => không ai phát lệnh => robot đứng yên dù
# ros2_control đã hoạt động đúng.
#
# LƯU Ý QUAN TRỌNG: 2 file .py PHẢI có quyền thực thi (chmod +x) vì
# `colcon build --symlink-install` chỉ tạo SYMLINK trỏ về file nguồn trong src/.
# Nếu thiếu quyền x, `ros2 run` sẽ báo "No executable found".
#
# Dùng RENAME để lệnh chạy gọn: `ros2 run my_mobilerobot teleop_keyboard`
# (thay vì phải gõ ... teleop_keyboard.py). Nếu muốn biết script nằm ở đâu:
#     ros2 pkg prefix my_mobilerobot    => .../install/my_mobilerobot
#     ls install/my_mobilerobot/lib/my_mobilerobot/
# install(PROGRAMS
#   my_mobilerobot/teleop_keyboard.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME teleop_keyboard
# )
# install(PROGRAMS
#   my_mobilerobot/demo_square.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME demo_square
# )

# # [THÊM 2026-10-13] Node test vòng kín trên ROBOT THẬT: kiểm tra 2 bánh cùng chiều khi
# # ra lệnh đi thẳng, chiều quay đúng convention URDF, và ước lượng MAX_WHEEL_RAD_S.
# # Dùng: ros2 run my_mobilerobot wheel_test   (CẢNH BÁO: xe chạy thật ~0.3 m!)
# install(PROGRAMS
#   my_mobilerobot/wheel_test.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME wheel_test
# )


ament_package()

