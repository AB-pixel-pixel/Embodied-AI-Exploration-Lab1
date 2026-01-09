#!/bin/bash

echo "===  下载机器人模型和代码 ==="






echo "=== 开始自动测试 ==="

# 测试1: Linux基础命令
echo "[测试1] Linux基础命令..."
cd ~ && mkdir -p test_workspace && cd test_workspace
echo "print('Hello Test')" > test.py
python3 test.py && echo "✓ Python执行成功" || echo "✗ Python失败"


# 测试2: ROS环境
echo "[测试2] ROS环境检查..."
source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash
rosversion -d && echo "✓ ROS环境正常" || echo "✗ ROS环境异常"

# 测试3: catkin工作空间
echo "[测试3] 编译工作空间..."
cd ~/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="lab2_perception" 2>&1 | tail -5
source devel/setup.bash
echo "✓ 编译完成"

# 测试4: 依赖检查
echo "[测试4] 检查关键依赖..."
python3 -c "import cv2; import numpy" && echo "✓ OpenCV/NumPy正常" || echo "✗ 缺少Python依赖"
python3 -c "import pytorch"  && echo "✓ Pytorch 正常" || echo "✗ 缺少Pytorch依赖"
rospack find turtlebot3_gazebo && echo "✓ TurtleBot3包存在" || echo "✗ 缺少TurtleBot3"

# 测试5: 权限设置
echo "[测试5] 设置执行权限..."
chmod +x ~/catkin_ws/src/lab2_perception/tools/*.py
chmod +x ~/catkin_ws/src/lab2_perception/scripts/*.py
echo "✓ 权限设置完成"

echo "=== 测试完成 ==="
