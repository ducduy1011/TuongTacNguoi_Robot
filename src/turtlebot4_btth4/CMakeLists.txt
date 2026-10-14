cmake_minimum_required(VERSION 3.8)
project(turtlebot4_btth4)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/teleop_wasd.py
  DESTINATION lib/${PROJECT_NAME} RENAME teleop_wasd)
install(PROGRAMS scripts/wait_localization.py
  DESTINATION lib/${PROJECT_NAME} RENAME wait_localization)
install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
ament_package()
