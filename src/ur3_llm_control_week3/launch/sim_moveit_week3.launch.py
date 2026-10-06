import os
import xacro
import yaml
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    p = get_package_share_directory('ur3_llm_control_week3')
    robot = xacro.process_file(os.path.join(p,'urdf','ur3e_with_gripper.urdf.xacro')).toxml()
    description = ET.fromstring(robot)
    # Use one elbow-up branch for this tabletop exercise; limits are narrower than UR hardware limits.
    bounds = {'shoulder_pan_joint':(-3.14159,3.14159),'shoulder_lift_joint':(-3.14159,0.0),
              'elbow_joint':(0.0,3.14159),'wrist_1_joint':(-3.14159,0.0),
              'wrist_2_joint':(-3.14159,0.0),'wrist_3_joint':(-3.14159,3.14159)}
    for joint in description.findall('joint'):
        if joint.get('name') in bounds:
            lo,hi = bounds[joint.get('name')]
            joint.find('limit').set('lower',str(lo)); joint.find('limit').set('upper',str(hi))
    robot = ET.tostring(description,encoding='unicode')
    ur_moveit = get_package_share_directory('ur_moveit_config')
    semantic = ET.fromstring(xacro.process_file(os.path.join(ur_moveit, 'srdf', 'ur.srdf.xacro'), mappings={'name':'ur','prefix':''}).toxml())
    semantic.set('name', 'ur3e_week3')
    for chain in semantic.iter('chain'):
        chain.set('tip_link', 'grasp_tcp')
    for a,b in [('gripper_base_link','wrist_3_link'),('gripper_base_link','left_finger_link'),('gripper_base_link','right_finger_link')]:
        ET.SubElement(semantic,'disable_collisions',link1=a,link2=b,reason='Adjacent')
    joints = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
    planning = {'planning_plugin':'ompl_interface/OMPLPlanner', 'request_adapters':'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints', 'start_state_max_bounds_error':0.05}
    with open(os.path.join(ur_moveit,'config','ompl_planning.yaml')) as f:
        planning.update(yaml.safe_load(f))
    moveit = {'robot_description':robot, 'robot_description_semantic':ET.tostring(semantic,encoding='unicode'),
        'robot_description_kinematics':{'ur_manipulator':{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin','kinematics_solver_timeout':0.1}},
        'robot_description_planning':{'joint_limits':{j:{'has_acceleration_limits':True,'max_acceleration':1.0} for j in joints}},
        'move_group':planning, 'use_sim_time':True,
        'publish_robot_description':True,'publish_robot_description_semantic':True,
        'moveit_controller_manager':'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager':{'controller_names':['arm_controller'],'arm_controller':{'type':'FollowJointTrajectory','action_ns':'follow_joint_trajectory','default':True,'joints':joints}},
        'trajectory_execution.allowed_execution_duration_scaling':2.0,
        'trajectory_execution.allowed_goal_duration_margin':5.0,
        'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True}
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    # Keep Gazebo's standard layout, but close immediately instead of leaving an exit dialog.
    gui_config = os.path.join(os.environ.get('TMPDIR', '/tmp'), 'ur3_week3_gazebo.config')
    with open('/usr/share/ignition/ignition-gazebo6/gui/gui.config') as stream:
        layout = stream.read().replace('<dialog_on_exit>true</dialog_on_exit>', '<dialog_on_exit>false</dialog_on_exit>')
    with open(gui_config, 'w') as stream:
        stream.write(layout)
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', description='Open the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value=gui, description='Open RViz with the live Gazebo scene'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(p, 'config', 'week3.rviz')], parameters=[{'use_sim_time': True}], condition=IfCondition(rviz), output='screen', on_exit=[Shutdown(reason='RViz closed')]),
        Node(package='ur3_llm_control_week3', executable='scene_visualizer.py', parameters=[{'use_sim_time': True}], condition=IfCondition(rviz), output='screen'),
        Node(package='moveit_ros_move_group',executable='move_group',parameters=[moveit],output='screen',
             additional_env={'LD_PRELOAD': os.path.join(get_package_prefix('ur3_llm_control_week3'), 'lib', 'libmoveit_shutdown.so')}),
        ExecuteProcess(cmd=['ign','gazebo','-s','-r',os.path.join(p,'worlds','ur3_task_week3.sdf')], output='screen', on_exit=[Shutdown(reason='Gazebo server stopped')]),
        ExecuteProcess(cmd=['ign','gazebo','-g','--gui-config',gui_config], condition=IfCondition(gui), output='screen', on_exit=[Shutdown(reason='Gazebo GUI closed')]),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':robot,'use_sim_time':True}]),
        Node(package='ros_gz_sim',executable='create',arguments=['-name','ur3e_week3','-topic','robot_description'],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock','/camera/color/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image','/world/ur3_task_week3/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],output='screen'),
        *[Node(package='controller_manager',executable='spawner',arguments=[name,'--controller-manager-timeout','60'],output='screen') for name in ['joint_state_broadcaster','arm_controller','gripper_controller']]
    ])
