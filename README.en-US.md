

# Embodied AI Exploration Lab 1

This repository contains the experimental code and related simulation environments for the Embodied AI course. The main contents include:

- **ROS Basic Examples** (`ros_course_examples`)
- **Turtlebot3 Robot Simulation** (`turtlebot3`, `turtlebot3_simulations`)
- **Related Message Definitions** (`turtlebot3_msgs`)

---

## 🚀 Quick Start

**For TA Configuration Only**

To spare you from tedious environment configuration, this project provides a one-click setup script. Please follow the steps below to deploy it.

### 1. Clone the Code

Open a terminal and run the following commands to clone the code locally:

```bash
git clone https://github.com/AB-pixel-pixel/Embodied-AI-Exploration-Lab1.git
mv Embodied-AI-Exploration-Lab1 catkin_ws
cd catkin_ws
```

### 2. Build the Code

We provide a `setup_course.sh` script that can initialize the workspace and complete the compilation in one click (assuming your ROS environment is already configured).

```bash
# Grant execute permissions to the script
chmod +x setup_course.sh

# Run the build script
./setup_course.sh
```

> **Note**: This script is only responsible for compilation. Please ensure that ROS Noetic and related dependencies are already installed in your environment.

### 3. Get Started

After successful compilation, you need to source the workspace environment variables:

```bash
source devel/setup.bash
```

You can now run the relevant nodes or simulation environments.

### 💡 Tips

If you want the environment to be automatically loaded every time you open a terminal, run:

```bash
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
