from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    behavior = Node(package='turtlebot4_btth6', executable='robot_behavior_node', output='screen')
    recognition = Node(package='turtlebot4_btth6', executable='gesture_recognition_node', output='screen',
                       parameters=[{
                           'camera_index': ParameterValue(LaunchConfiguration('camera_index'), value_type=int),
                           'show_window': ParameterValue(LaunchConfiguration('show_window'), value_type=bool)}])

    def stop_pair(event, context):
        if context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(reason='Camera/control node closed; stop gesture session.'))]

    return LaunchDescription([
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('show_window', default_value='true'),
        RegisterEventHandler(OnProcessExit(target_action=recognition, on_exit=stop_pair)),
        RegisterEventHandler(OnProcessExit(target_action=behavior, on_exit=stop_pair)),
        behavior,
        recognition,
    ])
