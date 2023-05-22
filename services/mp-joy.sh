#!/bin/bash

source ~/StanfordQuadruped/mini_pupper_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=$(cat /home/ubuntu/mini_pupper_ros_bsp/services/ROS_DOMAIN_ID)
ros2 run mini_pupper_joy mini_pupper_joy
