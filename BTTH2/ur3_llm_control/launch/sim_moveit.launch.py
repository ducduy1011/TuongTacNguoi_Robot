from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # PATHS
    sim_pkg = get_package_share_directory(
        "ur_simulation_gz"
    )
    moveit_pkg = get_package_share_directory(
        "ur_moveit_config"
    )
    task_pkg = get_package_share_directory(
        "ur3_llm_control"
    )

    world_file = os.path.join(
        task_pkg,
        "worlds",
        "ur3_task.sdf"
    )
    # GAZEBO + UR3e
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sim_pkg,
                "launch",
                "ur_sim_control.launch.py"
            )
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "world_file": world_file,
            "launch_rviz": "false",
        }.items()
    )
    # MOVEIT + RVIZ
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_pkg,
                "launch",
                "ur_moveit.launch.py"
            )
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items()
    )
    # LAUNCH
    return LaunchDescription([
        gazebo,
        moveit,
    ])