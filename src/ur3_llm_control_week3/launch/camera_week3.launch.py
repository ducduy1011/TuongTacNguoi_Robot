from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true', description='Show live camera detection'),
        Node(package='ur3_llm_control_week3', executable='scene_perception.py',
             parameters=[{'use_sim_time': True, 'viewer': LaunchConfiguration('viewer')}], output='screen',
             on_exit=[Shutdown(reason='Camera stopped')]),
    ])
