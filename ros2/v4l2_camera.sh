#!/bin/bash

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
ros2 run v4l2_camera v4l2_camera_node --ros-args -p output_encoding:="yuv422_yuy2"
