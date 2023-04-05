#!/bin/bash

ROS_DOMAIN_ID=0 ros2 topic pub --rate 50 -t 100 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
