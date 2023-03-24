import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument

import yaml

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true")

    config_filepath = os.path.join(
        get_package_share_directory('mini_pupper_bringup'), 
        'config'
    )

    joy_config_filepath = os.path.join(
        config_filepath,
        "joy.config.yaml"
    )

    teleop_config_filepath = os.path.join(
        config_filepath,
        "teleop.config.yaml"
    )


    return launch.LaunchDescription(
        [
            declare_use_sim_time,

			Node(
				package='mini_pupper_controller',
				executable='mini_pupper_controller',
				name='mini_pupper_controller_node',
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
        		package='joint_state_publisher',
        		executable='joint_state_publisher'
        	),

		]
    ) # return LD
