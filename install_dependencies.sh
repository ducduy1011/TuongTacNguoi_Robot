#!/usr/bin/env bash
set -euo pipefail
packages=(
  git
  curl
  cmake
  build-essential
  libceres-dev
  libboost-thread-dev
  libboost-serialization-dev
  libboost-filesystem-dev
  libtbb-dev
  qtbase5-dev  # Qt5 CMake dependency exported by slam_toolbox
  python3-pil
  python3-yaml
  python3-colcon-common-extensions
  ros-humble-ament-cmake-gen-version-h
  ignition-fortress
  ros-humble-turtlebot4-simulator
  ros-humble-irobot-create-nodes
  ros-humble-turtlebot4-navigation
  ros-humble-slam-toolbox
  ros-humble-navigation2
  ros-humble-nav2-bringup
  ros-humble-rviz2
  ros-humble-rclpy
  ros-humble-geometry-msgs
  ros-humble-rmw-fastrtps-cpp
)
missing=()
for package in "${packages[@]}"; do
  if [[ $(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true) != 'install ok installed' ]]; then
    missing+=("$package")
  fi
done
if ((${#missing[@]})); then
  sudo apt-get update
  sudo apt-get install --no-upgrade --no-remove -y "${missing[@]}"
else
  echo 'Dependencies already installed.'
fi
