"""TurtleBot 4 Standard o giua san, dieu khien truc tiep bang /cmd_vel."""
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            GroupAction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('turtlebot4_btth6'))
    description = Path(get_package_share_directory('turtlebot4_description'))

    # Dung mo hinh chinh thuc, chi doi file tham so controller va topic van toc.
    robot = xacro.process_file(
        str(description / 'urdf/standard/turtlebot4.urdf.xacro'),
        mappings={'gazebo': 'ignition', 'namespace': ''}).toxml()
    xml = ET.fromstring(robot)
    for plugin in xml.findall('.//gazebo/plugin'):
        parameters = plugin.find('parameters')
        if parameters is not None and parameters.text.endswith('/control.yaml'):
            parameters.text = str(package / 'config/controllers.yaml')
            ET.SubElement(plugin.find('ros'), 'remapping').text = (
                '/diffdrive_controller/cmd_vel_unstamped:=/cmd_vel')
    robot = ET.tostring(xml, encoding='unicode')

    server = ExecuteProcess(
        cmd=['ign', 'gazebo', str(package / 'worlds/gesture_arena.sdf'), '-s', '-r', '-v', '2'],
        output='screen')
    ready = Node(package='turtlebot4_btth6', executable='wait_for_controllers', output='screen')
    gui = ExecuteProcess(cmd=['ign', 'gazebo', '-g'], output='screen')

    def open_gui(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Controller chua san sang.'))]
        return [gui]

    return LaunchDescription([
        DeclareLaunchArgument('use_mesa', default_value='false'),
        GroupAction(
            condition=IfCondition(LaunchConfiguration('use_mesa')), scoped=False,
            actions=[SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '0'),
                     SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'mesa'),
                     SetEnvironmentVariable('DRI_PRIME', '0')]),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', str(description.parent)),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                               EnvironmentVariable('LD_LIBRARY_PATH', default_value='')),
        RegisterEventHandler(OnProcessExit(
            target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server closed.'))])),
        RegisterEventHandler(OnProcessExit(
            target_action=gui,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo window closed.'))])),
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=open_gui)),
        server,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'gesture_arena', '-name', 'turtlebot4',
                        '-topic', 'robot_description', '-z', '0.01']),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['joint_state_broadcaster', 'diffdrive_controller',
                        '--controller-manager-timeout', '60', '--service-call-timeout', '60']),
        ready,
    ])
