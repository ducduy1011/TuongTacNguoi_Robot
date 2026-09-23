import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def xacro_parameter(name, file, args):
    command = [FindExecutable(name="xacro"), " ", file]

    for key, value in args:
        command += [" ", key, ":=", value]

    return {
        name: ParameterValue(
            Command(command),
            value_type=str,
        )
    }


def generate_launch_description():

    writer_dir = get_package_share_directory(
        "ur3_letter_writer"
    )

    sim_dir = get_package_share_directory(
        "ur_simulation_gz"
    )

    moveit_dir = get_package_share_directory(
        "ur_moveit_config"
    )

    description_dir = get_package_share_directory(
        "ur_description"
    )


    config = os.path.join(
        writer_dir,
        "config",
        "letter_d.yaml",
    )

    rviz_config = os.path.join(
        writer_dir,
        "rviz",
        "letter_writer.rviz",
    )


    ur_type = "ur3e"
    prefix = '""'

    safety_limits = "true"
    safety_margin = "0.15"
    safety_k = "20"


    common = {
        "ur_type": ur_type,
        "safety_limits": safety_limits,
        "safety_pos_margin": safety_margin,
        "safety_k_position": safety_k,
        "prefix": prefix,
    }


    # ========================================================
    # GAZEBO + ros2_control
    # ========================================================

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sim_dir,
                "launch",
                "ur_sim_control.launch.py",
            )
        ),
        launch_arguments={
            **common,

            "launch_rviz": "false",
            "gazebo_gui": "true",

            "start_joint_controller": "true",

            "initial_joint_controller":
                "joint_trajectory_controller",

        }.items(),
    )


    # ========================================================
    # MOVEIT 2
    # ========================================================

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_dir,
                "launch",
                "ur_moveit.launch.py",
            )
        ),
        launch_arguments={
            **common,

            "description_package":
                "ur_description",

            "description_file":
                "ur.urdf.xacro",

            "moveit_config_package":
                "ur_moveit_config",

            "moveit_config_file":
                "ur.srdf.xacro",

            "use_sim_time":
                "true",

            "launch_rviz":
                "false",

            "launch_servo":
                "false",

        }.items(),
    )


    # ========================================================
    # ROBOT DESCRIPTION
    # ========================================================

    joint_limits = os.path.join(
        description_dir,
        "config",
        ur_type,
        "joint_limits.yaml",
    )

    kinematics_params = os.path.join(
        description_dir,
        "config",
        ur_type,
        "default_kinematics.yaml",
    )

    physical_params = os.path.join(
        description_dir,
        "config",
        ur_type,
        "physical_parameters.yaml",
    )

    visual_params = os.path.join(
        description_dir,
        "config",
        ur_type,
        "visual_parameters.yaml",
    )


    robot_description = xacro_parameter(
        "robot_description",

        os.path.join(
            description_dir,
            "urdf",
            "ur.urdf.xacro",
        ),

        [
            ("robot_ip", "xxx.yyy.zzz.www"),

            ("joint_limit_params", joint_limits),

            ("kinematics_params", kinematics_params),

            ("physical_params", physical_params),

            ("visual_params", visual_params),

            ("safety_limits", safety_limits),

            ("safety_pos_margin", safety_margin),

            ("safety_k_position", safety_k),

            ("name", "ur"),

            ("ur_type", ur_type),

            ("script_filename",
             "ros_control.urscript"),

            ("input_recipe_filename",
             "rtde_input_recipe.txt"),

            ("output_recipe_filename",
             "rtde_output_recipe.txt"),

            ("prefix", prefix),
        ],
    )


    robot_description_semantic = xacro_parameter(
        "robot_description_semantic",

        os.path.join(
            moveit_dir,
            "srdf",
            "ur.srdf.xacro",
        ),

        [
            ("name", "ur"),
            ("prefix", prefix),
        ],
    )


    kinematics_yaml = os.path.join(
        moveit_dir,
        "config",
        "kinematics.yaml",
    )


    # ========================================================
    # RVIZ
    # ========================================================

    rviz = TimerAction(
        period=8.0,

        actions=[
            Node(
                package="rviz2",
                executable="rviz2",

                name="rviz2_letter_writer",

                arguments=[
                    "-d",
                    rviz_config,
                ],

                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,

                    {
                        "use_sim_time": True
                    },
                ],

                output="screen",
            )
        ],
    )


    # ========================================================
    # LETTER D NODE
    # ========================================================

    draw_letter_d = TimerAction(
        period=20.0,

        actions=[
            Node(
                package="ur3_letter_writer",
                executable="draw_letter_d",

                name="draw_letter_d",

                parameters=[
                    config,
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,

                    {
                        "use_sim_time": True
                    },
                ],

                output="screen",
            )
        ],
    )


    return LaunchDescription(
        [
            simulation,
            moveit,
            rviz,
            draw_letter_d,
        ]
    )