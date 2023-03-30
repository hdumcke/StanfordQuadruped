import os

import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    pkg_share = FindPackageShare('mini_pupper_description').find('mini_pupper_description')
    xacro_dir = os.path.join(pkg_share, 'xacro')
    xacro_file = os.path.join(xacro_dir, 'mini_pupper_description.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[params])

    return launch.LaunchDescription([rsp])
