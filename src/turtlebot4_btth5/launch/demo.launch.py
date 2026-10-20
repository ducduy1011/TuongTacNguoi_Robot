"""One fresh simulation: eight objects, AMCL/Nav2, YOLO and three coded goals."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent, LogInfo, GroupAction, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    old = Path(get_package_share_directory('turtlebot4_btth4'))
    current = Path(get_package_share_directory('turtlebot4_btth5'))
    workspace = Path(get_package_prefix('turtlebot4_btth5')).parents[1]
    def include(path, **arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                       launch_arguments=arguments.items())
    objects = Node(package='turtlebot4_btth5', executable='spawn_objects', output='screen',
                   arguments=['--world', 'maze', '--models', str(workspace / '.models/scene'),
                              '--wait', '120', '--wait-controllers'])
    prepare = Node(package='turtlebot4_btth5', executable='prepare_robot', output='screen',
                   parameters=[{'use_sim_time': True, 'startup_pause': 5.0}], sigterm_timeout='15')
    patrol = Node(package='turtlebot4_btth5', executable='navigate_goals', output='screen',
                  parameters=[{'use_sim_time': True, 'pause_at_goal': 5.0}], sigterm_timeout='15')
    def after_objects(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Could not create all eight objects.'))]
        # Start perception/RViz only after robot controllers and all scene objects.
        # A simultaneous cold start can overload DDS controller-service discovery.
        return [include(current / 'launch/localization.launch.py'),
                GroupAction(actions=[include(current / 'launch/perception.launch.py',
                                             show_window='true',
                                             model=str(workspace / '.models/yolov8n.pt'))]),
                prepare]
    def after_prepare(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Robot preparation failed; no navigation goals sent.'))]
        return [patrol]
    return LaunchDescription([
        LogInfo(msg='BTTH05 demo: start from a fresh simulator; initial pose/undock/goals are automatic.'),
        RegisterEventHandler(OnProcessExit(target_action=objects, on_exit=after_objects)),
        RegisterEventHandler(OnProcessExit(target_action=prepare, on_exit=after_prepare)),
        include(old / 'launch/simulation.launch.py'),
        TimerAction(period=15.0, actions=[objects]),
    ])
