import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import yaml

def generate_launch_description():

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

			launch_ros.actions.Node(
				package='mini_pupper_controller',
				executable='mini_pupper_controller',
				name='_mini_pupper_controller_node',
			),  
					
			launch_ros.actions.Node(
				package='joy', 
				executable='joy_node', 
				name='joy_node',
				parameters=[joy_config_filepath]
			),

			launch_ros.actions.Node(
				package='teleop_twist_joy', 
				executable='teleop_node',
				name='teleop_twist_joy_node',
				parameters=[teleop_config_filepath]
			),

			launch_ros.actions.Node(
				package='robot_localization',
				executable='ekf_node',
				name='ekf_filter_node',
				output='screen',
				parameters=[robot_localization_config_filepath],
                remappings=[("imu", "imu/raw_data")]
			),
            
    		launch_ros.actions.Node(
        		package='joint_state_publisher',
        		executable='joint_state_publisher'
        	),

			launch_ros.actions.Node(
        		package='robot_state_publisher',
        		executable='robot_state_publisher',
        		parameters=[{'robot_description': robot_description}]
    		),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch_path),
            )

		]
    ) # return LD
