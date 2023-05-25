#!/bin/bash

#### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

source  ~/mini-pupper-release
if [ "$MACHINE" == "x86_64" ]
then
ACTIVE="rc-local lidar v4l2_camera robot-ros mp-joy"
ACTIVATING="battery_monitor"
if [ "$HARDWARE" == "mini_pupper_2" ]
then
ACTIVE=$ACTIVE" imu"
ACTIVATING=$ACTIVATING" esp32-proxy"
fi
else
ACTIVE="rc-local lidar v4l2_camera robot-ros mp-joy"
ACTIVATING="battery_monitor"
if [ "$HARDWARE" == "mini_pupper_2" ]
then
ACTIVE=$ACTIVE" imu"
ACTIVATING=$ACTIVATING" esp32-proxy"
fi
fi

# Give services time to start
sleep 10

echo "Checking systemd services"
for SERVICE in $ACTIVE; do
    STATE=$(sudo systemctl show $SERVICE | grep ActiveState | awk -F'=' '{print $2}')
    if [ "$STATE" == "active" ]
    then
	echo $SERVICE is OK
    else
	echo $SERVICE is NOT OK
    fi
done

for SERVICE in $ACTIVATING; do
    STATE=$(sudo systemctl show $SERVICE | grep ActiveState | awk -F'=' '{print $2}')
    if [ "$STATE" == "activating" ]
    then
	echo $SERVICE is OK
    else
	echo $SERVICE is NOT OK
    fi
done

# check ros topics
echo "Checking ROS topics"
ros2 topic list > /tmp/topics
if cmp --silent -- /tmp/topics $BASEDIR/expected/topics; then
  echo "ROS2 topics are OK"
else
  echo "ROS2 topics are NOT OK"
  diff /tmp/topics $BASEDIR/expected/topics
fi
