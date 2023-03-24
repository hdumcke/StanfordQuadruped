#!/bin/bash

# check Ubuntu version
source /etc/os-release

if [[ $UBUNTU_CODENAME != 'jammy' ]]
then
    echo "Ubuntu 22.04.1 LTS (Jammy Jellyfish) is required"
    echo "You are using $VERSION"
    exit 1
fi

### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

yes | sudo apt-get install libatlas-base-dev
yes | pip3 install numpy transforms3d pyserial
yes | pip install numpy transforms3d pyserial
yes | sudo pip install numpy transforms3d pyserial
sudo apt-get install -y unzip

### Install ROS2
cd ~
git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
./ros2_setup_scripts_ubuntu/run.sh

source /opt/ros/humble/setup.bash
cd ~/StanfordQuadruped/mini_pupper_ws
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent
sudo pip install setuptools==58.2.0 # suppress colcon build warning
colcon build --executor sequential --symlink-install

cd ~/StanfordQuadruped/ros2
sudo adduser ubuntu input
sudo pip install ds4drv
sudo ln -s $(realpath .)/robot-ros.service /etc/systemd/system/
sudo ln -s $(realpath .)/joystick.service /etc/systemd/system/
sudo ln -s $(realpath .)/restart_joy.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot-ros
sudo systemctl start robot-ros
sudo systemctl enable joystick
sudo systemctl start joystick
sudo systemctl enable restart_joy
sudo systemctl start restart_joy

# Cyclon DDS
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
