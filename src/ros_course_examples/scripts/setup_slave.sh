#!/bin/bash

# Usage: source setup_slave.sh <MASTER_IP>

if [ -z "$1" ]; then
    echo "Please provide the Master IP address."
    echo "Usage: source setup_slave.sh <MASTER_IP>"
    return
fi

MASTER_IP=$1

echo "Setting up this machine as ROS SLAVE..."

# Get current IP
MY_IP=$(hostname -I | cut -d' ' -f1)

export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_IP=$MY_IP

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_IP set to: $ROS_IP"
echo "Done. Please run this on the student/client computer."
