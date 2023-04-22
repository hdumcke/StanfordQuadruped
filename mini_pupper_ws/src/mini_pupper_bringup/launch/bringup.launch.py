import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    dt = LaunchConfiguration("dt")

    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true")

    config_filepath = os.path.join(
        get_package_share_directory('mini_pupper_bringup'),
        'config'
    )

    dt_arg = DeclareLaunchArgument(
        name="dt", default_value="0.015",
        description="Gait parameter dt")

    return launch.LaunchDescription(
        [
            declare_use_sim_time,
            dt_arg,

            Node(
                package='mini_pupper_controller',
                executable='mini_pupper_controller',
                name='mini_pupper_controller_node',
                parameters=[{'dt': dt}]
            ),

        ]
    )  # return LD
