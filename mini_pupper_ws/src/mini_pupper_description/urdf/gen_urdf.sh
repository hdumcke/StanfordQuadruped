#!/usr/bin/env bash

set -e
ros2 run xacro xacro ../xacro/mini_pupper_description.xacro > mini_pupper.urdf
