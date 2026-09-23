cmake_minimum_required(VERSION 3.8)
project(ur3_circle_writer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(
  draw_circle
  src/draw_circle.cpp
)

ament_target_dependencies(
  draw_circle
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

install(
  TARGETS draw_circle
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()