#!/bin/bash

source ~/StanfordQuadruped/mini_pupper_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch mini_pupper_bringup imu.launch.py
