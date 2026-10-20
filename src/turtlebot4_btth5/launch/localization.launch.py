"""Reuse week 04's map, AMCL and Nav2, with camera displays for week 05."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    previous = get_package_share_directory('turtlebot4_btth4')
    current = get_package_share_directory('turtlebot4_btth5')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([previous, 'maps', 'map.yaml'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([previous, 'launch', 'navigation.launch.py'])),
            launch_arguments={'map': LaunchConfiguration('map'), 'nav2': 'true', 'rviz': 'true',
                              'localization_params': PathJoinSubstitution([current, 'config', 'localization.yaml']),
                              'rviz_config': PathJoinSubstitution([current, 'config', 'btth5.rviz'])}.items()),
    ])
