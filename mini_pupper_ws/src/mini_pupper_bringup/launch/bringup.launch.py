import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

import xacro


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

    joy_config_filepath = os.path.join(
        config_filepath,
        "joy.config.yaml"
    )

    teleop_config_filepath = os.path.join(
        config_filepath,
        "teleop.config.yaml"
    )

    pkg_share = FindPackageShare('mini_pupper_description').find('mini_pupper_description')
    xacro_dir = os.path.join(pkg_share, 'xacro')
    xacro_file = os.path.join(xacro_dir, 'mini_pupper_description.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}

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

            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[joy_config_filepath]
            ),

            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[teleop_config_filepath]
            ),

            Node(
                 package='robot_state_publisher',
                 executable='robot_state_publisher',
                 output='both',
                 parameters=[params]
            ),

            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher'
            ),

        ]
    )  # return LD
