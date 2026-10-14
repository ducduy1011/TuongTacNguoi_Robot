cmake_minimum_required(VERSION 3.8)
project(turtlebot4_btth4)
find_package(ament_cmake REQUIRED)
get_filename_component(BTTH4_WORKSPACE "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(BTTH4_GPU_ENVIRONMENT "")
if(EXISTS "/proc/driver/nvidia/version")
  set(BTTH4_GPU_ENVIRONMENT "set;__NV_PRIME_RENDER_OFFLOAD;1\nset;__GLX_VENDOR_LIBRARY_NAME;nvidia")
endif()
ament_environment_hooks(config/environment.dsv.in)
install(PROGRAMS scripts/teleop_wasd.py
  DESTINATION lib/${PROJECT_NAME} RENAME teleop_wasd)
install(PROGRAMS scripts/wait_localization.py
  DESTINATION lib/${PROJECT_NAME} RENAME wait_localization)
install(PROGRAMS scripts/stop_simulation.py
  DESTINATION lib/${PROJECT_NAME} RENAME stop_simulation)
install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
ament_package()
