#!/bin/bash

cd ~/StanfordQuadruped/services
sed -i "/CYCLONEDDS_URI/d" *.sh
cd ~/mini_pupper_ros_bsp/services
sed -i "/CYCLONEDDS_URI/d" *.sh
sed -i "/CYCLONEDDS_URI/d" ~/.bashrc
