"""Online asynchronous SLAM and RViz for the separately launched TurtleBot 4 simulator."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    navigation = get_package_share_directory('turtlebot4_navigation')
    exercise = get_package_share_directory('turtlebot4_btth4')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([navigation, 'launch', 'slam.launch.py'])),
            launch_arguments={'use_sim_time': 'true', 'sync': 'false',
                              'params': PathJoinSubstitution([exercise, 'config', 'slam.yaml'])}.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([exercise, 'config', 'mapping.rviz'])],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
