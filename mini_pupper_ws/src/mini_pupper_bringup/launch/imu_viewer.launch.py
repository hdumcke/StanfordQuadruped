import os

import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    pkg_share = FindPackageShare('mini_pupper_bringup').find('mini_pupper_bringup')
    rviz_dir = os.path.join(pkg_share, 'rviz')
    rviz_file = os.path.join(rviz_dir, 'imu_viewer.rviz')
    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=["-d%s" % rviz_file])

    return launch.LaunchDescription([rviz])
