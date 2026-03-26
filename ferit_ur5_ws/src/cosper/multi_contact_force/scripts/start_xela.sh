#!/bin/bash
# filepath: /home/RVLuser/ferit_ur5_ws/src/cosper/multi_contact_force/scripts/start_xela.sh

set -e

TTYUSB=${1:-/dev/ttyUSB0}

echo "[XELA] Setting up XELA sensor on $TTYUSB ..."

export PATH="/xela_suite_linux:/etc/xela:${PATH}"

# Setup CAN-USB bridge
slcand -o -s8 -t hw -S 3000000 "$TTYUSB"
sleep 1

# Bring up the network interface
ifconfig slcan0 up
sleep 1

echo "[XELA] Hardware ready. Launching ROS node..."
source /home/RVLuser/ferit_ur5_ws/devel/setup.bash
roslaunch xela_server_ros service.launch