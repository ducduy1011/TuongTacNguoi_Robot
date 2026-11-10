from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('regions_file'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('speech', default_value='true'),
        DeclareLaunchArgument('device', default_value='-1'),
        DeclareLaunchArgument('min_confidence', default_value='0.6'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='voice_navigation', executable='map_labels', output='screen',
             parameters=[{'regions_file': LaunchConfiguration('regions_file'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='voice_navigation', executable='voice_nav', output='screen',
             parameters=[{'regions_file': LaunchConfiguration('regions_file'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='voice_navigation', executable='speech', output='screen',
             condition=IfCondition(LaunchConfiguration('speech')),
             parameters=[{'model_path': LaunchConfiguration('model_path'),
                          'regions_file': LaunchConfiguration('regions_file'),
                          'min_confidence': ParameterValue(
                              LaunchConfiguration('min_confidence'), value_type=float),
                          'device': ParameterValue(LaunchConfiguration('device'), value_type=int)}]),
    ])
