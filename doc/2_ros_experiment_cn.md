## 实验二：ROS 基础与通信机制

### 1. 实验目的

* 理解 ROS 节点（Node）概念
* 掌握话题（Topic）发布-订阅机制
* 理解 ROS Master 的作用

### 2. ROS 核心概念

#### 2.1 基本组成

* **Node（节点）** ：ROS 系统的基本运行单元，每个节点执行特定任务
* **Topic（话题）** ：节点间通信的命名通道
* **Master（主节点）** ：提供节点间通信的协调服务（需要 `roscore`）

#### 2.2 结点通信

结点之前最常用的通信方式就是基于topic，一个发布者结点会命名一个topic然后把消息发布到这个topic上。另一个订阅者结点则会订阅topic。
![1766756911148](image/2.ros_experiment/1766756911148.png)

### 3. 实验步骤

首先，从github中下载课程的代码库，并且将其改名为catkin_ws。

``` bash
git clone https://github.com/AB-pixel-pixel/Embodied-AI-Exploration-Lab1.git
mv Embodied-AI-Exploration-Lab1 catkin_ws
cd catkin_ws
```



#### 实验 2.1：体验无通信的孤立程序


```bash
cd ~/catkin_ws/src/ros_course_examples/simulation_demo
python3 controller.py
```

```bash
# 终端 2：运行 Motor
cd ~/catkin_ws/src/ros_course_examples/simulation_demo
python3 motor.py
```

执行效果如图：![1766556330841](image/2.ros_experiment/1766556330841.png)

**观察** ：两个程序各自运行，无法相互通信。

关闭这两个程序吧。

#### 实验 2.2：使用 ROS 实现节点通信

我们已经将代码封装成了ROS结点，文件如下：
`src/ros_course_examples/nodes/motor_node.py`和 `src/ros_course_examples/nodes/controller_node.py`。

只要我们启动这两个节点，就可以基于ROS框架实现结点通信。

首先进行编译

在终端 1：编译ros工作空间

我们使用 `catkin_make` 指令对ros 包进行编译。它的作用是：

- 编译源代码：将C++源文件编译成可执行文件，将Python脚本标记为可执行
- 处理依赖关系：自动解析并链接软件包之间的依赖
- 生成配置文件：创建devel/setup.bash等环境配置脚本
- 构建消息类型：编译自定义的msg、srv、action文件

```bash
cd ~/catkin_ws
catkin_make
```

大概的示意图：
![1766556827503](image/2.ros_experiment/1766556827503.png)

执行后会：

- 在build/目录中编译代码
- 将结果输出到devel/目录
- 生成可供ROS使用的环境变量

终端 1：启动 ROS Master 结点

`roscore`指令可以启动 Master 结点：

- 功能：管理所有节点的注册和发现
- 作用：让Publisher和Subscriber能够找到彼此
- 类比：像中心服务器，连接着结点
- **注意：在实验中我们不关闭正在运行roscore的终端。**
```bash
roscore
```

![1766557292995](image/2.ros_experiment/1766557292995.png)

为结点的代码提供执行的权限：

```bash
cd ~/catkin_ws/src/ros_course_examples/nodes/  
chmod +x motor_node.py
chmod +x controller_node.py
```

![1766557275580](image/2.ros_experiment/1766557275580.png)

**启动通信演示**
由于要使用ROS框架，我们需要使用ROS指令启动代码而不是简单的使用python命令

这个就是 `rosrun`指令：

- 功能：运行某个 ROS 包里的一个节点
- 特点：简单、直接、适合测试或调试
- 示例用法：

```bash
# rosrun <ros包名> <可执行文件名>
rosrun turtlesim turtlesim_node
```

- 如果可执行文件需要参数，也可以直接跟在后面：rosrun pkg exe arg1 arg2 ...

终端 2：启动电机结点

**注意：`source devel/setup.bash` 能帮助ros指令找到对应的可执行文件。**
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun ros_course_examples motor_node.py
```

你可以发现这个结点（程序）正在等待信息，它的位置并不发生改变。
![1766558557910](image/2.ros_experiment/1766558557910.png)

终端 3：启动控制器结点
接下来，让我们控制它

```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun ros_course_examples controller_node.py
```

观察终端，我们可以发现它正在不断的发送指令：
![1766559045635](image/2.ros_experiment/1766559045635.png)

返回查看终端2（电机结点）
![1766559088885](image/2.ros_experiment/1766559088885.png)

**观察到** ：Controller 发送速度指令，Motor 接收并更新位置。

---

但是，如果每启动一个程序都输入一行指令不会太复杂了吗？

ROS 中提供了一种统一的启动配置机制，可以一键启动多个节点，这就是 `roslaunch` 指令。

`roslaunch`：
- 功能：通过 .launch 文件一次性启动多个节点
- 为结点设置参数，例如可以按照机器人的大小去设置相关规划算法的参数
- 重映射话题，适用于在不改代码的情况下改变通信关系
- 设置命名空间，适用于多机器人使用同样的代码
- launch 文件通常在包的 launch/ 目录下（不是强制，但约定俗成）。
- 示例：
```bash
# roslaunch <ros 包名> <包内的launch文件名>
roslaunch turtlesim turtlesim_demo.launch
```

关闭终端2和终端3，在终端3中执行：

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch ros_course_examples ros_communication_demo.launch
```

你可以观察到，这两个结点都已经启动了。

![1766559251701](image/2.ros_experiment/1766559251701.png)

在进行下面的实验之前可以先关闭这个程序（**注意不要关闭roscore的终端**）

#### 实验 2.3：TurtleSim 通信实验


终端 2：启动 Turtle 仿真

```bash
rosrun turtlesim turtlesim_node
```

![1766559377818](image/2.ros_experiment/1766559377818.png)

终端 3：启动键盘控制

```bash
rosrun turtlesim turtle_teleop_key
```

![1766559474890](image/2.ros_experiment/1766559474890.png)

在终端3中按下方向键可以控制turtle运动。
![1766559507381](image/2.ros_experiment/1766559507381.png)

终端 4：可视化通信图

```bash
rqt_graph
```

![1766559792210](image/2.ros_experiment/1766559792210.png)

**通信流程分析** ：

1. `turtle_teleop_key` 节点监听键盘输入
2. 发布速度命令到 `/turtle1/cmd_vel` 话题
3. 发布到话题中的信息就是线速度和角速度，消息的类型是 `geometry_msgs/Twist`
4. `turtlesim_node` 订阅 `/turtle1/cmd_vel`
5. 接收速度命令并执行运动

#### 实验 2.4：查看话题信息

以上这些信息都可以通过ros指令来观察到。

```bash
# 列出所有话题
rostopic list
```

```bash
# 查看话题信息
rostopic info /turtle1/cmd_vel
```

```bash
# 查看消息类型定义
rosmsg show geometry_msgs/Twist
```

![1766560209464](image/2.ros_experiment/1766560209464.png)

```bash
# 查看话题数据
rostopic echo /turtle1/cmd_vel
```

![1766559946201](image/2.ros_experiment/1766559946201.png)

---

## 实验三：Gazebo 仿真环境

### 1. 实验目的

* 掌握 Gazebo 仿真器的使用
* 学会加载和保存仿真世界
* 理解 World 文件结构
* 掌握机器人模型的加载
* Message Type

### 2. Gazebo 核心功能

* **物理引擎** ：模拟真实物理规律（重力、碰撞、摩擦）
* **传感器仿真** ：激光雷达、相机、IMU 等
* **ROS 集成** ：与 ROS 无缝通信
* **可视化** ：3D 场景渲染

### 3. 实验步骤

#### 3.1 启动空白世界

```bash
gazebo
```

![1766587986896](image/2.ros_experiment/1766587986896.png)

或使用 ROS 启动

```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

![1766588066542](image/2.ros_experiment/1766588066542.png)

#### 3.2 构建自定义场景

1. **插入模型** ：从左侧面板拖拽物体到场景
2. **调整参数** ：

* Position (x, y, z)：位置坐标
* Orientation (roll, pitch, yaw)：姿态角度
* Scale：缩放大小

1. **保存世界** ：`File → Save World As → my_world.world`

![1766589618862](image/2.ros_experiment/1766589618862.png)

![1766589642312](image/2.ros_experiment/1766589642312.png)

#### 3.3 加载自定义世界

```bash
gazebo my_world.world
```

方法 2：ROS launch 文件

```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

![1766590157072](image/2.ros_experiment/1766590157072.png)

#### 3.4 控制机器人运动

```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

 **操作说明** ：

* `W/A/S/D` 或 `方向键`：控制移动
* `X`：停止
* `Q/Z`：增加/减少速度

![1766590095927](image/2.ros_experiment/1766590095927.png)

---

## 实验四：RViz 可视化

### 1. 实验目的

* 掌握 RViz 可视化工具的使用
* 学会添加和配置显示项
* 理解传感器数据的可视化表示
* 掌握界面交互和视角控制

### 2. RViz 可显示的数据类型

* **Robot Model** ：3D 机器人模型
* **LaserScan** ：激光雷达扫描数据
* **PointCloud2** ：3D 点云数据
* **TF** ：坐标系变换关系
* **Image** ：相机图像
* **Odometry** ：里程计轨迹
* **Path** ：规划路径
* **Map** ：占据栅格地图

### 3. RViz 启动方式介绍

直接启动

```bash
rviz
```

![1766591530072](image/2.ros_experiment/1766591530072.png)

#### 3.1 RViz 界面基础操作

##### **视角控制**

* 🖱️  **鼠标左键拖拽** ：旋转视角
* 🖱️  **鼠标滚轮** ：缩放视图
* 🖱️  **Shift + 左键拖拽** ：平移视图
* 🖱️  **Shift + 滚轮** ：上下平移
* 🖱️  **鼠标中键拖拽** ：平移（某些系统）

### 4. 实验步骤

#### 4.1 启动仿真和可视化

```bash
# 终端 1：启动 gazebo
source ~/catkin_ws/devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

```bash
# 终端 2：启动 RViz
source ~/catkin_ws/devel/setup.bash
roslaunch turtlebot_rviz_launchers view_robot.launch 
```

![](2.%20ros_experiment.assets/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202025-12-26%20180703.png)

#### 4.2 添加显示项（Display）详细步骤

##### **示例1：添加激光雷达数据** (或者直接勾选LaserScan的框框就像，rviz那个launch的rviz配置好了)

1. 点击左下角 **"Add"** 按钮
2. 在弹出窗口中选择 **"By display type"** 标签
3. 找到并双击 **"LaserScan"**
4. 在左侧 Displays 面板中展开 **"LaserScan"**
5. 配置参数：

```bash
   Topic: /scan           # 点击下拉选择 /scan
   Size (m): 0.05         # 调整点的大小
   Style: Points          # 显示样式
   Color Transformer: Intensity  # 颜色映射
```

6. 观察红色扫描点显示障碍物位置

![](2.%20ros_experiment.assets/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202025-12-26%20180847.png)

##### **示例2：添加坐标系（TF）**

1. 点击 **"Add"** → 选择 **"TF"**
2. 配置参数：

```bash
   ✓ Show Axes           # 显示坐标轴
   ✓ Show Arrows         # 显示箭头
   Marker Scale: 0.5     # 调整坐标轴大小
   Update Interval: 0    # 更新频率（0=最快）
```

3. 观察红绿蓝箭头（代表 X/Y/Z 轴）

![](2.%20ros_experiment.assets/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202025-12-26%20181204.png)

#### 4.3 常用显示项配置表

| 显示类型             | 推荐话题   | 作用             | 关键参数                                  |
| -------------------- | ---------- | ---------------- | ----------------------------------------- |
| **RobotModel** | (默认)     | 显示机器人3D模型 | `Robot Description: robot_description`  |
| **LaserScan**  | `/scan`  | 激光雷达扫描数据 | `Size: 0.05`, `Style: Points`         |
| **Odometry**   | `/odom`  | 里程计轨迹       | `Keep: 100`, `Shape: Arro`            |
| **TF**         | (无需设置) | 坐标系关系       | `Show Names: ✓`, `Marker Scale: 0.5` |
| **Map**        | `/map`   | 占据栅格地图     | `Color Scheme: map`                     |

#### 4.4 调整 Fixed Frame（参考坐标系）

**什么是 Fixed Frame？**

* RViz 中所有数据的显示都需要一个参考坐标系
* 不同场景需要选择不同的 Fixed Frame

**选择建议**

| 场景           | Fixed Frame   | 效果               |
| -------------- | ------------- | ------------------ |
| 观察机器人运动 | `odom`      | 视角跟随机器人     |
| 调试传感器     | `base_link` | 视角锁定在机器人上 |

**设置方法**

1. 在顶部 **"Global Options"** 展开
2. 点击 **"Fixed Frame"** 下拉菜单
3. 选择 `odom`

⚠️  **注意** ：如果 Fixed Frame 设置错误，所有显示项会变灰色或不显示。

#### 4.5 查看 TF 树

**查看 TF 树图的指令**

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

**查看 TF 变换**

![](2.%20ros_experiment.assets/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202025-12-26%20180520.png)

### 6. 常见问题排查

| 问题                      | 原因             | 解决方法                           |
| ------------------------- | ---------------- | ---------------------------------- |
| ❌ 显示项变红色/灰色      | 话题未发布       | `rostopic list` 检查话题是否存在 |
| ❌ 看不到机器人模型       | Fixed Frame 错误 | 改为 `odom` 或 `base_link`     |
| ❌ 激光数据不显示         | Topic 选择错误   | 确认为 `/scan`                   |
| ❌ TF 显示 "No transform" | TF 树不完整      | 检查 `rosrun tf view_frames`     |

---

### 课后练习建议

* 多练习 Linux 命令行操作
* 尝试修改示例代码参数，观察效果
* 使用 `rqt_graph` 和 `rostopic` 工具分析系统
* 阅读 ROS Wiki 官方文档

### 推荐资源

* [ROS Wiki](http://wiki.ros.org/)
* [Gazebo Tutorials](http://gazebosim.org/tutorials)
* [TF Tutorials](http://wiki.ros.org/tf/Tutorials)
