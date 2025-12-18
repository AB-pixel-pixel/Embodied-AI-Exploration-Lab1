#!/bin/bash
# Script to run Game Version 3 (Coding Challenge)
source ../../devel/setup.bash

# Trap SIGINT to kill background processes
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

echo "Starting Gazebo Simulation with 2 Robots..."
roslaunch ros_course_examples multi_turtlebot_game.launch &
SIM_PID=$!

echo "Waiting for Gazebo to start..."
sleep 10

echo "Starting Random Walker (Prey)..."
rosrun ros_course_examples random_walker.py &

echo "Starting Referee Node..."
rosrun ros_course_examples game_referee.py &

echo "Starting Student Chaser Node (Template)..."
# In a real lab, students would edit this file and run it themselves.
# Here we run the template to show it's working (it does nothing but connect).
rosrun ros_course_examples student_chaser_template.py

wait $SIM_PID
