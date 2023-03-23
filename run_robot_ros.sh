#!/bin/bash

source ~/StanfordQuadruped/mini_pupper_ws/install/setup.bash
rm -rf ~/.ros/log/*
ros2 launch mini_pupper_bringup bringup.launch.py
