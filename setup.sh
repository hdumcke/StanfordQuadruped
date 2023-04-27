#!/bin/bash

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
[[ "$1" == "v2" ]] && git clone https://github.com/mangdangroboticsclub/mini_pupper_2_bsp.git mini_pupper_bsp
[[ -d ~/mini_pupper_ros_bsp ]] || git clone https://github.com//mangdangroboticsclub/mini_pupper_ros_bsp.git
[[ -d ~/StanfordQuadruped ]] || git clone https://github.com/mangdangroboticsclub/StanfordQuadruped.git /home/ubuntu/StanfordQuadruped

cd ~/mini_pupper_ros_bsp
sed -i "/reboot/d" setup.sh
./setup.sh $1

cd ~/StanfordQuadruped
./install_ros.sh

# disable unused services
sudo systemctl disable servo_interface
sudo systemctl disable display_interface

#TODO remove after PR that enables uart2is merged
### Enable UART5
### RXD5 = Pin 13
grep -q "uart5" /boot/firmware/config.txt || echo "dtoverlay=uart5" | sudo tee -a /boot/firmware/config.txt
sudo sed -i "s/^dtoverlay=audremap/#dtoverlay=audremap/" /boot/firmware/config.txt

echo "setup.sh finished at $(date)"
sudo reboot
