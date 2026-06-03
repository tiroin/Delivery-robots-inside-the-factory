#!/usr/bin/env bash
set -e
BITRATE=${1:-500000}
sudo modprobe can || true
sudo modprobe can_raw || true
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate "$BITRATE" restart-ms 100
ip -details link show can0
