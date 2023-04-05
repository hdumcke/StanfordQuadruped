import os
import launch
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_madgwick = LaunchConfiguration("madgwick",
                                       default="False")

    config_dir = os.path.join(get_package_share_directory('mini_pupper_bringup'), 'config')

    imu = Node(package='mini_pupper_imu',
               executable='mini_pupper_imu',
               name='mini_pupper_imu_node')
    # base_link to imu tf node
    imu_tf = Node(package='tf2_ros',
                  executable='static_transform_publisher',
                  name='base_link_to_imu',
                  arguments=['0', '0', '0',
                             '0', '0', '0',
                             'base_link', 'imu'],
                  condition=UnlessCondition(use_madgwick))
    imu_filter_complementary = Node(package='imu_complementary_filter',
                                    executable='complementary_filter_node',
                                    name='imu_complementary_filter',
                                    output='screen',
                                    parameters=[
                                        {'do_bias_estimation': True},
                                        {'do_adaptive_gain': True},
                                        {'use_mag': False},
                                        {'gain_acc': 0.01},
                                        {'gain_mag': 0.01},
                                    ],
                                    condition=UnlessCondition(use_madgwick))
    imu_filter_madgwick = Node(package='imu_filter_madgwick',
                               executable='imu_filter_madgwick_node',
                               name='imu_filter_madgwick',
                               output='screen',
                               parameters=[
                                   os.path.join(config_dir, 'imu_filter.yaml')
                               ],
                               condition=IfCondition(use_madgwick))
    return launch.LaunchDescription([imu, imu_tf, imu_filter_complementary, imu_filter_madgwick])
