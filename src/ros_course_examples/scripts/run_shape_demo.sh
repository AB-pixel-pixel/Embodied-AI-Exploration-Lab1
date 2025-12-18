#!/bin/bash
# Script to run Turtlebot Shape Demo
source ../../devel/setup.bash

echo "Select Shape:"
echo "1. Circle"
echo "2. Square"
echo "3. Rectangle"
read -p "Enter choice [1-3]: " choice

case $choice in
    1) SHAPE="circle" ;;
    2) SHAPE="square" ;;
    3) SHAPE="rectangle" ;;
    *) SHAPE="square" ;;
esac

roslaunch ros_course_examples move_turtlebot_demo.launch shape:=$SHAPE
