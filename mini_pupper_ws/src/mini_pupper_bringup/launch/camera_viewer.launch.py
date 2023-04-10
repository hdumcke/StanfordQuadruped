import os

import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    pkg_share = FindPackageShare('mini_pupper_description').find('mini_pupper_description')
    xacro_dir = os.path.join(pkg_share, 'xacro')
    xacro_file = os.path.join(xacro_dir, 'mini_pupper_description.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    pkg_share = FindPackageShare('mini_pupper_bringup').find('mini_pupper_bringup')
    rviz_dir = os.path.join(pkg_share, 'rviz')
    rviz_file = os.path.join(rviz_dir, 'camera_viewer.rviz')
    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[params])
    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=["-d%s" % rviz_file])

    return launch.LaunchDescription([rsp, rviz])
