#!/bin/bash

source ~/StanfordQuadruped/mini_pupper_ws/install/setup.bash
source ~/lidar_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
ros2 launch mini_pupper_bringup ld06.launch.py
