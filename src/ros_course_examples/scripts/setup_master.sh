#!/bin/bash

# Usage: source setup_master.sh

echo "Setting up this machine as ROS MASTER..."

# Get current IP
MY_IP=$(hostname -I | cut -d' ' -f1)

export ROS_MASTER_URI=http://$MY_IP:11311
export ROS_IP=$MY_IP

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_IP set to: $ROS_IP"
echo "Done. Please run this on the robot/server computer."
