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

cd ..
git clone https://github.com/stanfordroboticsclub/PupperCommand.git
cd PupperCommand
sed -i "s/pi/ubuntu/" joystick.service
sudo bash install.sh
cd ..

### Install ROS2
cd ~
git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
./ros2_setup_scripts_ubuntu/run.sh

source /opt/ros/humble/setup.bash
mkdir -p ~/mini_pupper_ws/src
cd ~/mini_pupper_ws
cp -r $BASEDIR/ros2/mini_pupper src/
mkdir -p src/mini_pupper/mini_pupper/src
mkdir -p src/mini_pupper/mini_pupper/pupper
cp -r $BASEDIR/src/IMU.py src/mini_pupper/mini_pupper/src
cp -r $BASEDIR/src/Controller.py src/mini_pupper/mini_pupper/src
cp -r $BASEDIR/src/State.py src/mini_pupper/mini_pupper/src
cp -r $BASEDIR/src/Gaits.py src/mini_pupper/mini_pupper/src
cp -r $BASEDIR/src/StanceController.py src/mini_pupper/mini_pupper/src
cp -r $BASEDIR/src/SwingLegController.py src/mini_pupper/mini_pupper/src
cp -r $BASEDIR/src/Utilities.py src/mini_pupper/mini_pupper/src
mkdir -p src/mini_pupper/mini_pupper/pupper
cp -r $BASEDIR/pupper/Kinematics.py src/mini_pupper/mini_pupper/pupper
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent
sudo pip install setuptools==58.2.0 # suppress colcon build warning
colcon build --executor sequential --symlink-install
