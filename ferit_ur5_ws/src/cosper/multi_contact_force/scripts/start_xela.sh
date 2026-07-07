#!/usr/bin/env bash
# start_xela.sh - Setup XELA USP44 tactile sensor and launch ROS node
# Usage: ./start_xela.sh [/dev/ttyUSBx]

set -euo pipefail

DEVICE="${1:-/dev/xela_usb}"
XSERV_INI="/etc/xela/xServ.ini"
ROS_WS="/home/RVLuser/ferit_ur5_ws"

if [[ ! -e "$DEVICE" ]]; then
    echo "ERROR: Device $DEVICE not found. Plug in the sensor or pass the correct port." >&2
    exit 1
fi

export PATH="/xela_suite_linux:/etc/xela:${PATH}"

# Tear down existing slcan0 if already up
if ip link show slcan0 &>/dev/null; then
    sudo ip link set slcan0 down 2>/dev/null || true
    sudo pkill -f "slcand.*$(basename "$DEVICE")" 2>/dev/null || true
    sleep 0.5
fi

sudo slcand -o -s8 -t hw -S 3000000 "$DEVICE" slcan0
sleep 0.5
sudo ip link set slcan0 up

source "${ROS_WS}/devel/setup.bash"
exec roslaunch xela_server_ros service.launch file:="$XSERV_INI"