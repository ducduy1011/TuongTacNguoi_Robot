"""Explicitly start the automatic three-goal program after localization/undock."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('turtlebot4_btth5')
    return LaunchDescription([
        DeclareLaunchArgument('goals_file', default_value=PathJoinSubstitution([share, 'config', 'goals.yaml'])),
        DeclareLaunchArgument('goal_timeout', default_value='360.0'),
        DeclareLaunchArgument('pause_at_goal', default_value='3.0'),
        Node(package='turtlebot4_btth5', executable='navigate_goals', output='screen',
             parameters=[{'use_sim_time': True, 'goals_file': LaunchConfiguration('goals_file'),
                          'goal_timeout': ParameterValue(LaunchConfiguration('goal_timeout'), value_type=float),
                          'pause_at_goal': ParameterValue(LaunchConfiguration('pause_at_goal'), value_type=float)}]),
    ])
