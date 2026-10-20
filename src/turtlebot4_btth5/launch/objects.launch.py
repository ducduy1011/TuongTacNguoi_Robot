"""Create eight static objects in the same maze world as simulation.launch.py."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_world():
    # The TB4 world is searched before the Create 3 world; their names differ.
    sdf = Path(get_package_share_directory('turtlebot4_ignition_bringup')) / 'worlds/maze.sdf'
    return ET.parse(sdf).getroot().find('world').attrib['name']


def generate_launch_description():
    workspace = Path(get_package_prefix('turtlebot4_btth5')).parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('models', default_value=str(workspace / '.models/scene')),
        DeclareLaunchArgument('world', default_value=default_world()),
        Node(package='turtlebot4_btth5', executable='spawn_objects', output='screen',
             arguments=['--world', LaunchConfiguration('world'),
                        '--models', LaunchConfiguration('models'), '--wait-controllers']),
    ])
