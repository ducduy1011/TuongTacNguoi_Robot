"""Validate a newly saved map before starting official AMCL and Nav2."""
import math
import os
from pathlib import Path
import yaml
from PIL import Image

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def validate_map(path):
    map_file = Path(path).expanduser().resolve()
    if not map_file.is_file():
        raise ValueError(f'Chua co map: {map_file}. Hay mapping va luu map bang nav2_map_server/map_saver_cli truoc.')
    data = yaml.safe_load(map_file.read_text())
    if not isinstance(data, dict) or not isinstance(data.get('image'), str):
        raise ValueError('map.yaml phai co truong image hop le.')
    resolution = data.get('resolution')
    if isinstance(resolution, bool) or not isinstance(resolution, (int, float)) or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('resolution trong map.yaml phai la so duong.')
    origin = data.get('origin')
    if not isinstance(origin, list) or len(origin) != 3 or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in origin):
        raise ValueError('origin trong map.yaml phai gom ba so: x, y, yaw.')
    image_file = map_file.parent / data['image']
    if not image_file.is_file() or image_file.stat().st_size < 16:
        raise ValueError(f'Anh map khong ton tai hoac rong: {image_file}')
    try:
        with Image.open(image_file) as image:
            image.load()
    except (OSError, ValueError) as error:
        raise ValueError(f'Anh map khong doc duoc: {image_file}') from error
    if data.get('mode', 'trinary') not in ('trinary', 'scale', 'raw'):
        raise ValueError('mode trong map.yaml phai la trinary, scale hoac raw.')
    if data.get('negate') not in (0, 1):
        raise ValueError('negate trong map.yaml phai la 0 hoac 1.')
    for key in ('free_thresh', 'occupied_thresh'):
        value = data.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 <= value <= 1:
            raise ValueError(f'{key} trong map.yaml phai trong [0, 1].')
    if data['free_thresh'] >= data['occupied_thresh']:
        raise ValueError('free_thresh phai nho hon occupied_thresh.')
    return str(map_file)


def start_navigation(context, exercise, navigation):
    try:
        map_file = validate_map(LaunchConfiguration('map').perform(context))
    except (OSError, ValueError, yaml.YAMLError) as error:
        return [LogInfo(msg=f'[BTTH04] {error}'),
                EmitEvent(event=Shutdown(reason='Save a valid map before localization.'))]
    nav2_params = RewrittenYaml(
        source_file=PathJoinSubstitution([navigation, 'config', 'nav2.yaml']),
        root_key='', convert_types=True,
        param_rewrites={'robot_radius': '0.25', 'inflation_radius': '0.60',
                        'cost_scaling_factor': '3.0'})
    rviz_config = PythonExpression([
        "'btth4.rviz' if '", LaunchConfiguration('nav2'),
        "' == 'true' else 'mapping.rviz'"])
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [navigation, 'launch', 'localization.launch.py'])),
        launch_arguments={'use_sim_time': 'true', 'map': map_file}.items())
    plugin = Path(get_package_prefix('nav2_rviz_plugins')) / 'lib/libnav2_rviz_plugins.so'
    preload = str(plugin)
    if os.environ.get('LD_PRELOAD'):
        preload += ' ' + os.environ['LD_PRELOAD']
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', PathJoinSubstitution([exercise, 'config', rviz_config])],
                parameters=[{'use_sim_time': True}],
                # Keep the plugin resident until ROS callback groups are destroyed.
                additional_env={'LD_PRELOAD': preload},
                condition=IfCondition(LaunchConfiguration('rviz')))
    if LaunchConfiguration('nav2').perform(context) == 'false':
        return [localization, rviz]
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [navigation, 'launch', 'nav2.launch.py'])),
        launch_arguments={'use_sim_time': 'true', 'params_file': nav2_params}.items())
    waiter = Node(package='turtlebot4_btth4', executable='wait_localization',
                  parameters=[{'use_sim_time': True}], output='screen')
    def after_localization(event, launch_context):
        if launch_context.is_shutdown:
            return []
        if event.returncode != 0:
            return [LogInfo(msg='[BTTH04] Localization wait stopped; Nav2 was not started.')]
        return [LogInfo(msg='[BTTH04] AMCL/TF ready; starting Nav2.'), nav2]
    return [localization, rviz,
            RegisterEventHandler(OnProcessExit(target_action=waiter,
                                              on_exit=after_localization)), waiter]



def generate_launch_description():
    exercise = get_package_share_directory('turtlebot4_btth4')
    navigation = get_package_share_directory('turtlebot4_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(
            Path(__file__).resolve().parents[1] / 'maps' / 'map.yaml')),
        DeclareLaunchArgument('nav2', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=start_navigation,
                       kwargs={'exercise': exercise, 'navigation': navigation}),
    ])
