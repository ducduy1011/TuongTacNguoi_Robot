import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


# ============================================================
# DOC YAML
# ============================================================

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)

    with open(absolute_path, "r") as file:
        return yaml.safe_load(file)


# ============================================================
# LAUNCH SETUP
# ============================================================

def launch_setup(context, *args, **kwargs):

    # --------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------

    ur_type = LaunchConfiguration("ur_type")

    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    prefix = LaunchConfiguration("prefix")


    # ========================================================
    # PACKAGE CUA CHUNG TA
    # ========================================================

    writer_share = get_package_share_directory(
        "ur3_circle_writer"
    )


    # YAML chu D
    circle_config = os.path.join(
        writer_share,
        "config",
        "circle.yaml",
    )


    # RViz config co san Letter D Guide
    rviz_config = os.path.join(
        writer_share,
        "rviz",
        "circle_writer.rviz",
    )


    # ========================================================
    # 1. GAZEBO + ros2_control
    #
    # KHONG MO RVIZ MAC DINH
    # ========================================================

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_simulation_gz"),
                    "launch",
                    "ur_sim_control.launch.py",
                ]
            )
        ),

        launch_arguments={
            "ur_type": ur_type,

            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,

            "prefix": prefix,

            # Rat quan trong
            "launch_rviz": "false",

            # Mo Gazebo GUI
            "gazebo_gui": "true",

            # Bat controller
            "start_joint_controller": "true",

            "initial_joint_controller":
                "joint_trajectory_controller",

        }.items(),
    )


    # ========================================================
    # 2. MOVEIT
    #
    # CUNG KHONG MO RVIZ MAC DINH
    # ========================================================

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_moveit_config"),
                    "launch",
                    "ur_moveit.launch.py",
                ]
            )
        ),

        launch_arguments={
            "ur_type": ur_type,

            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,

            "description_package":
                "ur_description",

            "description_file":
                "ur.urdf.xacro",

            "moveit_config_package":
                "ur_moveit_config",

            "moveit_config_file":
                "ur.srdf.xacro",

            "prefix": prefix,

            "use_sim_time": "true",

            # Rat quan trong
            "launch_rviz": "false",

            # Khong can Servo
            "launch_servo": "false",

        }.items(),
    )


    # ========================================================
    # 3. ROBOT DESCRIPTION
    #
    # Dung cho:
    #   - draw_circle
    #   - RViz cua chung ta
    # ========================================================

    joint_limit_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            ur_type,
            "joint_limits.yaml",
        ]
    )


    kinematics_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            ur_type,
            "default_kinematics.yaml",
        ]
    )


    physical_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            ur_type,
            "physical_parameters.yaml",
        ]
    )


    visual_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            ur_type,
            "visual_parameters.yaml",
        ]
    )


    robot_description_content = Command(
        [
            PathJoinSubstitution(
                [
                    FindExecutable(name="xacro")
                ]
            ),

            " ",

            PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "urdf",
                    "ur.urdf.xacro",
                ]
            ),

            " ",

            "robot_ip:=xxx.yyy.zzz.www",

            " ",

            "joint_limit_params:=",
            joint_limit_params,

            " ",

            "kinematics_params:=",
            kinematics_params,

            " ",

            "physical_params:=",
            physical_params,

            " ",

            "visual_params:=",
            visual_params,

            " ",

            "safety_limits:=",
            safety_limits,

            " ",

            "safety_pos_margin:=",
            safety_pos_margin,

            " ",

            "safety_k_position:=",
            safety_k_position,

            " ",

            "name:=ur",

            " ",

            "ur_type:=",
            ur_type,

            " ",

            "script_filename:=ros_control.urscript",

            " ",

            "input_recipe_filename:=rtde_input_recipe.txt",

            " ",

            "output_recipe_filename:=rtde_output_recipe.txt",

            " ",

            "prefix:=",
            prefix,

            " ",
        ]
    )


    robot_description = {
        "robot_description":
            ParameterValue(
                robot_description_content,
                value_type=str,
            )
    }


    # ========================================================
    # 4. SRDF
    # ========================================================

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution(
                [
                    FindExecutable(name="xacro")
                ]
            ),

            " ",

            PathJoinSubstitution(
                [
                    FindPackageShare("ur_moveit_config"),
                    "srdf",
                    "ur.srdf.xacro",
                ]
            ),

            " ",

            "name:=ur",

            " ",

            "prefix:=",
            prefix,

            " ",
        ]
    )


    robot_description_semantic = {
        "robot_description_semantic":
            ParameterValue(
                robot_description_semantic_content,
                value_type=str,
            )
    }


    # ========================================================
    # 5. IK / KINEMATICS
    # ========================================================

    robot_description_kinematics = PathJoinSubstitution(
        [
            FindPackageShare("ur_moveit_config"),
            "config",
            "kinematics.yaml",
        ]
    )


    # ========================================================
    # 6. JOINT LIMITS MOVEIT
    # ========================================================

    robot_description_planning = {
        "robot_description_planning":
            load_yaml(
                "ur_moveit_config",
                "config/joint_limits.yaml",
            )
    }


    # ========================================================
    # 7. OMPL CONFIG
    #
    # Can cho MotionPlanning trong RViz.
    # ========================================================

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin":
                "ompl_interface/OMPLPlanner",

            "request_adapters":
                "default_planner_request_adapters/"
                "AddTimeOptimalParameterization "
                "default_planner_request_adapters/"
                "FixWorkspaceBounds "
                "default_planner_request_adapters/"
                "FixStartStateBounds "
                "default_planner_request_adapters/"
                "FixStartStateCollision "
                "default_planner_request_adapters/"
                "FixStartStatePathConstraints",

            "start_state_max_bounds_error":
                0.1,
        }
    }


    ompl_yaml = load_yaml(
        "ur_moveit_config",
        "config/ompl_planning.yaml",
    )


    if ompl_yaml is not None:
        ompl_planning_pipeline_config[
            "move_group"
        ].update(ompl_yaml)


    # ========================================================
    # 8. WAREHOUSE
    # ========================================================

    warehouse_ros_config = {
        "warehouse_plugin":
            "warehouse_ros_sqlite::DatabaseConnection",

        "warehouse_host":
            os.path.expanduser(
                "~/.ros/warehouse_ros.sqlite"
            ),
    }


    # ========================================================
    # 9. RVIZ CUA CHUNG TA
    #
    # Day la RViz DUY NHAT.
    #
    # No mo bang:
    #   circle_writer.rviz
    #
    # Trong file nay da co:
    #   /letter_d_guide
    # ========================================================

    rviz_node = TimerAction(
        period=8.0,

        actions=[
            Node(
                package="rviz2",
                executable="rviz2",

                name="rviz2_circle_writer",

                output="screen",

                arguments=[
                    "-d",
                    rviz_config,
                ],

                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                    warehouse_ros_config,

                    {
                        "use_sim_time": True
                    },
                ],
            )
        ],
    )


    # ========================================================
    # 10. NODE VE CHU D
    #
    # Cho Gazebo + MoveIt khoi dong truoc.
    # ========================================================

    draw_circle_node = TimerAction(
        period=20.0,

        actions=[
            Node(
                package="ur3_circle_writer",
                executable="draw_circle",

                name="draw_circle",

                output="screen",

                parameters=[
                    circle_config,

                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,

                    {
                        "use_sim_time": True
                    },
                ],
            )
        ],
    )


    # ========================================================
    # RETURN
    # ========================================================

    return [
        ur_control_launch,
        ur_moveit_launch,
        rviz_node,
        draw_circle_node,
    ]


# ============================================================
# GENERATE LAUNCH DESCRIPTION
# ============================================================

def generate_launch_description():

    return LaunchDescription(
        [

            # ------------------------------------------------
            # UR3e
            # ------------------------------------------------

            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                description="UR robot type",
            ),


            # ------------------------------------------------
            # Safety
            # ------------------------------------------------

            DeclareLaunchArgument(
                "safety_limits",
                default_value="true",
            ),


            DeclareLaunchArgument(
                "safety_pos_margin",
                default_value="0.15",
            ),


            DeclareLaunchArgument(
                "safety_k_position",
                default_value="20",
            ),


            # ------------------------------------------------
            # Prefix
            # ------------------------------------------------

            DeclareLaunchArgument(
                "prefix",
                default_value='""',
            ),


            # ------------------------------------------------
            # Setup
            # ------------------------------------------------

            OpaqueFunction(
                function=launch_setup
            ),
        ]
    )