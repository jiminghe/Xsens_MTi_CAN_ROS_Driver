#!/bin/bash

# Load the necessary modules for CAN and CAN RAW communication
sudo modprobe can
sudo modprobe can_raw

# Put the CAN interface down (necessary before making changes)
sudo ip link set can0 down

# Add a small delay (replace with 'sleep 1'(second) if busybox is not available)
sleep 1

# Set the CAN interface bitrate and type 250000(default), 500000, 1000000
sudo ip link set can0 up type can bitrate 250000

# Put the CAN interface up
sudo ip link set can0 up

# Print status message
echo "CAN0 interface set up with bitrate 250000"

