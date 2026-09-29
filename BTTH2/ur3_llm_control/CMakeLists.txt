cmake_minimum_required(VERSION 3.8)
project(ur3_llm_control)

# AMENT
find_package(ament_cmake REQUIRED)

# ROS 2
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# MOVEIT
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# GAZEBO
find_package(ros_gz_interfaces REQUIRED)

# TF
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# CUSTOM INTERFACE
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteSkill.srv"
)

# SKILL SERVER
add_executable(
  skill_server
  src/skill_server.cpp
)

ament_target_dependencies(
  skill_server
  rclcpp
  geometry_msgs
  moveit_msgs
  shape_msgs
  moveit_ros_planning_interface
  ros_gz_interfaces
  tf2
  tf2_ros
)

# TYPESUPPORT
rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  skill_server
  "${cpp_typesupport_target}"
)

# INSTALL C++
install(
  TARGETS
  skill_server
  DESTINATION
  lib/${PROJECT_NAME}
)

# INSTALL PYTHON
install(
  PROGRAMS
  scripts/llm_task.py
  DESTINATION
  lib/${PROJECT_NAME}
)

# INSTALL DATA
install(
  DIRECTORY
  launch
  config
  worlds
  DESTINATION
  share/${PROJECT_NAME}
)

ament_package()