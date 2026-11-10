"""Khoi dong server va GUI rieng de tranh ket o starting_world."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    def share(name):
        return Path(get_package_share_directory(name))

    bringup = share('turtlebot4_ignition_bringup')
    create = share('irobot_create_ignition_bringup')
    resources = [
        bringup / 'worlds', create / 'worlds',
        share('turtlebot4_description').parent,
        share('irobot_create_description').parent,
    ]
    plugins = [
        share('turtlebot4_ignition_gui_plugins') / 'lib',
        share('irobot_create_ignition_plugins') / 'lib',
    ]
    project = Path(__file__).resolve().parents[1]
    world = project / 'worlds' / 'warehouse.sdf'
    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ':'.join(map(str, resources))),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', ':'.join(map(str, plugins))),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                               str(share('gz_ros2_control').parent.parent / 'lib')
                               + ':' + os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')),
        SetEnvironmentVariable('IGN_PARTITION', f'bai7_{os.getuid()}'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', '-v', '3', str(world)],
                       output='screen'),
        TimerAction(period=2.0, actions=[
            ExecuteProcess(cmd=['ign', 'gazebo', '-g', '-v', '3', '--gui-config',
                                str(bringup / 'gui/standard/gui.config')], output='screen'),
        ]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='clock_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/turtlebot4_spawn.launch.py')),
            launch_arguments={'world': 'warehouse', 'rviz': 'false',
                              'use_sim_time': 'true'}.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', str(project / 'config/navigation.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
