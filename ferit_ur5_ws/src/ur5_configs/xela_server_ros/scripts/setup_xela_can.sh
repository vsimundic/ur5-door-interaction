#!/usr/bin/env bash
# setup_xela_can.sh — Host-side CAN setup for XELA USP44 tactile sensor
#
# Run this on the HOST (not inside the dev container) before launching ROS.
# The /dev/ttyXELA symlink is created by the udev rule 99-xela-can.rules.
#
# Usage:
#   ./setup_xela_can.sh [device]
#
# Examples:
#   ./setup_xela_can.sh               # uses /dev/ttyXELA (default)
#   ./setup_xela_can.sh /dev/ttyUSB2  # explicit device

set -euo pipefail

DEVICE="${1:-/dev/ttyXELA}"

# ── Validate device ──────────────────────────────────────────────────────────
if [ ! -e "$DEVICE" ]; then
  echo "[ERROR] Device '$DEVICE' not found." >&2
  echo "        Is the XELA USB-CAN adapter plugged in?" >&2
  echo "        If the udev rule is not installed, run:" >&2
  echo "          sudo cp 99-xela-can.rules /etc/udev/rules.d/" >&2
  echo "          sudo udevadm control --reload-rules && sudo udevadm trigger" >&2
  exit 1
fi

# ── Tear down existing slcan0 interface (if any) ─────────────────────────────
if ip link show slcan0 &>/dev/null; then
  echo "[INFO] Bringing down existing slcan0..."
  sudo ip link set slcan0 down
  sudo slcan_attach -d /dev/null 2>/dev/null || true
  sleep 0.5
fi

# Kill any existing slcand process holding the device
if pgrep -x slcand &>/dev/null; then
  echo "[INFO] Stopping existing slcand process..."
  sudo pkill -x slcand || true
  sleep 0.5
fi

# ── Start slcand ─────────────────────────────────────────────────────────────
# -o  : open mode (no ACK required — device is in listen-capable mode)
# -s8 : CAN bitrate 1 Mbit/s  (s8 = 1000 kbps in SLCAN notation)
# -t hw : hardware flow control
# -S 3000000 : serial baud rate 3 Mbit/s
echo "[INFO] Starting slcand on $DEVICE..."
sudo slcand -o -s8 -t hw -S 3000000 "$DEVICE" slcan0

# ── Bring up the network interface ───────────────────────────────────────────
echo "[INFO] Bringing up slcan0..."
sudo ip link set slcan0 up

echo "[OK] slcan0 is up and ready."
echo "     You can now launch ROS inside the container."
