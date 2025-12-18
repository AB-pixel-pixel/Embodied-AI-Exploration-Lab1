#!/bin/bash
# Script to run Game Version 1 (Single PC, Dual Teleop)
source ../../devel/setup.bash

# Trap SIGINT to kill background processes
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

echo "Starting Gazebo Simulation with 2 Robots..."
roslaunch ros_course_examples multi_turtlebot_game.launch &
SIM_PID=$!

echo "Waiting for Gazebo to start..."
sleep 10

echo "Starting Referee Node..."
rosrun ros_course_examples game_referee.py &

echo "Starting Teleop Controller..."
rosrun ros_course_examples dual_teleop.py

wait $SIM_PID
