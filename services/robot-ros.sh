#!/bin/bash

source ~/StanfordQuadruped/mini_pupper_ws/install/setup.bash
rm -rf ~/.ros/log/*
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=$(cat /home/ubuntu/mini_pupper_ros_bsp/services/ROS_DOMAIN_ID)
ros2 launch mini_pupper_bringup bringup.launch.py dt:=0.01
