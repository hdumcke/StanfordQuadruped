#!/bin/bash

cd ~/StanfordQuadruped/services
sed -i "/RMW_IMPLEMENTATION/a export CYCLONEDDS_URI=file:///home/ubuntu/StanfordQuadruped/tests/vpn/cyclone_dds_config.xml" *.sh
cd ~/mini_pupper_ros_bsp/services
sed -i "/RMW_IMPLEMENTATION/a export CYCLONEDDS_URI=file:///home/ubuntu/StanfordQuadruped/tests/vpn/cyclone_dds_config.xml" *.sh
echo "export CYCLONEDDS_URI=file:///home/ubuntu/StanfordQuadruped/tests/vpn/cyclone_dds_config.xml" >> ~/.bashrc
