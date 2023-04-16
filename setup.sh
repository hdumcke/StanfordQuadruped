#!/bin/bash
######################################################################################
# Stanford
#
# This stack will consist of board support package (mini_pupper_bsp),
#    the StanfordQuadruped controller and the mini_pupper_web_controller
#
# After installation you can either pair a supported PS4 joystick or
# point your web browser to 
#   http://x.x.x.x:8080
# where x.x.x.x is the IP address of your Mini Pupper as displayed on the LCD screen
#
# To install
#    ./setup.sh <SSID> "<your Wifi password>"
######################################################################################

set -e
echo "setup.sh started at $(date)"

### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# check Ubuntu version
source /etc/os-release

if [[ $UBUNTU_CODENAME != 'jammy' ]]
then
    echo "Ubuntu 22.04 LTS (Jammy Jellyfish) is required"
    echo "You are using $VERSION"
    exit 1
fi

cd ~
[[ "$1" == "v1" ]] && git clone https://github.com/mangdangroboticsclub/mini_pupper_bsp.git mini_pupper_bsp
#[[ "$1" == "v2" ]] && git clone https://github.com/mangdangroboticsclub/mini_pupper_2_bsp.git mini_pupper_bsp
#TODO remove after testing
[[ "$1" == "v2" ]] && git clone -b add_calibration_tools https://github.com/hdumcke/mini_pupper_2_bsp.git mini_pupper_bsp
sed -i "s/PBR_VERSION/DUMMY/" ~/mini_pupper_bsp/install.sh
##########################
[[ -d ~/StanfordQuadruped ]] || git clone https://github.com/mangdangroboticsclub/StanfordQuadruped.git
./mini_pupper_bsp/install.sh

#TODO move this to mini_pupper_2_bsp
### Enable UART5
### RXD5 = Pin 13
grep -q "uart5" /boot/firmware/config.txt || echo "dtoverlay=uart5" | sudo tee -a /boot/firmware/config.txt
sudo sed -i "s/^dtoverlay=audremap/#dtoverlay=audremap/" /boot/firmware/config.txt

cd ~/StanfordQuadruped
./install_ros.sh

# Install Lidar
source /opt/ros/humble/setup.bash
mkdir -p ~/lidar_ws/src
cd ~/lidar_ws
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git src/ldlidar
colcon build
cd ~/StanfordQuadruped/ros2
sudo ln -s $(realpath .)/lidar.service /etc/systemd/system/
sudo systemctl enable lidar

# Install IMU
if [[ "$1" == "v2" ]]
then
cd ~/StanfordQuadruped/ros2
sudo ln -s $(realpath .)/imu.service /etc/systemd/system/
sudo systemctl enable imu
sudo apt install -y ros-humble-imu-tools
fi

# Install Camera
~/mini_pupper_bsp/RPiCamera/install.sh
cd ~/StanfordQuadruped/ros2
sudo ln -s $(realpath .)/v4l2_camera.service /etc/systemd/system/
sudo systemctl enable v4l2_camera
sudo apt install -y ros-humble-v4l2-camera ros-humble-image-transport-plugins
mkdir -p ~/.ros/camera_info/
# copy defaultcamera calibration file. This should be replaced by a camera spefici calibration file
cp $BASEDIR/ros2/mmal_service_16.1.yaml ~/.ros/camera_info/

echo "setup.sh finished at $(date)"
sudo reboot
