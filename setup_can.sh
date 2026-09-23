#!/bin/bash
# setup_can.sh — one-time PCAN interface initialisation on the Jetson AGX Orin
# Run once after each boot (or add to /etc/rc.local for auto-start).
#
# Usage:
#   sudo bash setup_can.sh            # bring up can0 at 250 kbps
#   sudo bash setup_can.sh 500000     # custom bitrate

set -e

BITRATE=${1:-250000}

echo "[setup_can] loading peak_usb driver..."
modprobe peak_usb

# Wait for the interface to appear (up to 5 s)
for i in $(seq 1 10); do
    ip link show can0 &>/dev/null && break
    sleep 0.5
done

if ! ip link show can0 &>/dev/null; then
    echo "[setup_can] ERROR: can0 not found after 5 s. Check PCAN cable."
    exit 1
fi

echo "[setup_can] bringing up can0 at ${BITRATE} bps..."
ip link set can0 down 2>/dev/null || true
ip link set can0 up type can bitrate ${BITRATE}
ip link set can0 txqueuelen 1000

echo "[setup_can] done."
ip link show can0
