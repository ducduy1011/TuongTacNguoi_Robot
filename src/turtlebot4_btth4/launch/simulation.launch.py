"""Official TurtleBot 4 Standard simulator with Fortress resource paths."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    simulator = get_package_share_directory('turtlebot4_ignition_bringup')
    create = get_package_share_directory('irobot_create_ignition_bringup')
    models = str(Path(get_package_share_directory('turtlebot4_description')).parent)
    gpu = []
    if Path('/proc/driver/nvidia/version').exists():
        gpu = [SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
               SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')]
    return LaunchDescription(gpu + [
        DeclareLaunchArgument('world', default_value='maze'),
        DeclareLaunchArgument('model', default_value='standard',
                              choices=['standard'],
                              description='Use the official TurtleBot 4 Standard.'),
        # Keep the low charging dock outside the tested navigation route.
        DeclareLaunchArgument('x', default_value='-2.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               ':'.join([simulator + '/worlds', create + '/worlds', models])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [simulator, 'launch', 'turtlebot4_ignition.launch.py'])),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'model': LaunchConfiguration('model'),
                              'x': LaunchConfiguration('x'),
                              'y': LaunchConfiguration('y'),
                              'use_sim_time': 'true', 'rviz': 'false',
                              'slam': 'false', 'localization': 'false',
                              'nav2': 'false'}.items()),
    ])
