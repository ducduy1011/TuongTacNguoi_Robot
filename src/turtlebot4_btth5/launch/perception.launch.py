"""Use the isolated YOLO Python interpreter without changing ROS's interpreter."""
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    python = LaunchConfiguration('python').perform(context)
    model = LaunchConfiguration('model').perform(context)
    if not Path(python).is_file() or not Path(model).is_file():
        raise RuntimeError('Run python3 tools/prepare_yolo.py first (YOLO Python/weights missing).')
    return [LogInfo(msg='YOLO uses CPU and a separate process; Nav2 keeps its ROS interpreter.'),
            Node(package='turtlebot4_btth5', executable='yolo_detector', output='screen',
                 prefix=[python + ' '],
                 parameters=[{'use_sim_time': True, 'model': model,
                              'image_topic': LaunchConfiguration('image_topic'),
                              'show_window': ParameterValue(LaunchConfiguration('show_window'), value_type=bool),
                              'image_size': ParameterValue(LaunchConfiguration('image_size'), value_type=int),
                              'max_fps': ParameterValue(LaunchConfiguration('max_fps'), value_type=float)}])]


def generate_launch_description():
    # Prefix is <workspace>/install/turtlebot4_btth5 for the standard isolated build.
    workspace = Path(get_package_prefix('turtlebot4_btth5')).parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('python', default_value=str(workspace / '.venv-yolo/bin/python')),
        DeclareLaunchArgument('model', default_value=str(workspace / '.models/yolov8n.pt')),
        DeclareLaunchArgument('image_topic', default_value='/oakd/rgb/preview/image_raw'),
        DeclareLaunchArgument('show_window', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('max_fps', default_value='2.0'),
        DeclareLaunchArgument('image_size', default_value='320'),
        OpaqueFunction(function=start),
    ])
