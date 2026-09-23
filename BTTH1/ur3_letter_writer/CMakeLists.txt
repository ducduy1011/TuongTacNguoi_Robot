cmake_minimum_required(VERSION 3.8)
project(ur3_letter_writer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(
  draw_letter_d
  src/draw_letter_d.cpp
)

ament_target_dependencies(
  draw_letter_d
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

install(
  TARGETS draw_letter_d
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()