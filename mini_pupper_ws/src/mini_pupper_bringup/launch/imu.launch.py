import launch
from launch_ros.actions import Node


def generate_launch_description():
    imu = Node(package='mini_pupper_imu',
                        executable='mini_pupper_imu',
                        name='mini_pupper_imu_node',
            )
    return launch.LaunchDescription([imu])
