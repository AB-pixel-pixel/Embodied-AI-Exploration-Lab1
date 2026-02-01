# Embodied AI Exploration Lab 1 (具身智能探索实验1)

本仓库包含具身智能课程的实验代码和相关仿真环境。主要内容包括：

- **ROS 基础示例** (`ros_course_examples`)
- **Turtlebot3 机器人仿真** (`turtlebot3`, `turtlebot3_simulations`)
- **相关消息定义** (`turtlebot3_msgs`)

---

## 🚀 快速开始 (Quick Start)

**仅提供给TA配置**

为了让大家能够免去繁琐的环境配置，本项目提供了一键配置脚本。请按照以下步骤进行部署。

### 1. 下载代码

打开终端，运行以下命令将代码克隆到本地：

```bash
git clone https://github.com/AB-pixel-pixel/Embodied-AI-Exploration-Lab1.git
mv Embodied-AI-Exploration-Lab1 catkin_ws
cd catkin_ws
```

### 2. 编译代码

我们提供了一个 `setup_course.sh` 脚本，可以一键初始化工作空间并完成编译（假设你已经配置好了ROS环境）。

```bash
# 赋予脚本执行权限
chmod +x setup_course.sh

# 运行编译脚本
./setup_course.sh
```

> **注意**：本脚本仅负责编译。请确保你的环境中已经安装了 ROS Noetic 及相关依赖。

### 3. 开始使用

编译成功后，你需要激活工作空间的环境变量：

```bash
source devel/setup.bash
```

现在你可以运行相关节点或仿真环境了。test

### 💡 小贴士

如果你希望每次打开终端自动加载该环境，可以运行：

```bash
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
