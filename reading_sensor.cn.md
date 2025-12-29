下面是该文档的**完整中文翻译**，保留了原有结构与技术术语，便于你在实验和教学中直接对应使用。

---

# 实验 2：ROS 感知系统基础 —— 图像订阅与处理

## 1. 实验目标
- 学习如何在 ROS 中订阅并处理图像数据（RGB 与深度）。
- 掌握使用 OpenCV 的基础图像处理技术：颜色空间转换（HSV）、目标检测与轮廓检测。
- 理解如何利用深度图和相机内参计算目标的三维坐标。
- 学习使用 TF 将坐标从相机坐标系转换到机器人基座坐标系。
- 实现一个基础的视觉伺服（Visual Servoing）控制环，使机器人跟随目标。
- 使用 Open3D 可视化三维点云数据。

---

## 2. 实验前置条件
- **硬件**：TurtleBot3（或仿真环境），Ubuntu 20.04 的 PC
- **软件**：ROS Noetic，Python 3.x，OpenCV，Open3D，`cv_bridge`
- **ROS 包**：`lab2_perception`（已在 `src/` 中提供）

---

## 3. 环境配置与编译

首先，请确保你的环境配置完成并成功编译该功能包。

```bash
# 1. 进入工作空间
cd ~/catkin_ws

# 2. 编译功能包
# 通过白名单方式仅编译该包以加快速度
catkin_make -DCATKIN_WHITELIST_PACKAGES="lab2_perception"

# 3. 加载环境变量以注册新功能包
source devel/setup.bash
```

---

## 4. 分步实验流程

### 第一部分：基础图像订阅

在进行复杂处理之前，先验证我们能够正确接收来自相机的图像数据。

#### 1.1 运行 RGB 图像订阅节点
该脚本订阅话题 `/camera/rgb/image_raw` 并显示实时视频流。

```bash
# 终端 1：启动 ROS Master（如果尚未运行）
roscore

# 终端 2：播放 rosbag 或启动相机仿真（如果有）
# 本实验通常在后续部分使用仿真启动文件，
# 但如果仿真已运行，也可单独测试脚本。
```

**注意**：以下脚本需要一个正在运行的相机数据源。如果你尚未启动 Gazebo，请先跳转到**第三部分**启动仿真，然后再返回此处。

```bash
# 终端 3：运行 RGB 监听节点
rosrun lab2_perception rgb_subscriber.py
```

#### 1.2 运行深度图像订阅节点
该脚本订阅话题 `/camera/depth/image_raw` 并可视化深度图。

```bash
# 终端 4：运行深度监听节点
rosrun lab2_perception depth_subscriber.py
```

---

### 第二部分：图像处理工具（HSV 参数调节）

为了检测特定目标（如红色方块），需要确定合适的 HSV（色相、饱和度、亮度）阈值。我们提供了辅助工具来完成这一过程。

#### 2.1 生成测试图像
首先生成一些示例图像用于算法调试。

```bash
# 在 src/lab2_perception/demo_images/ 中生成测试图像
rosrun lab2_perception generate_images.py
```

#### 2.2 调节 HSV 阈值
使用 HSV 调节工具寻找最佳参数，以准确分割红色目标。

```bash
# 启动 HSV 调节工具
rosrun lab2_perception hsv_tuner.py
```

**使用说明**：
1. 使用 “Image” 滑块在生成的图像之间切换。
2. 调整 `H Min`、`S Min`、`V Min` 等参数，直到中间的掩膜窗口中只有目标物体显示为白色。
3. **记录这些参数值**，后续主感知节点将会用到它们（代码中已预设红色的默认参数）。

---

### 第三部分：集成感知与视觉伺服

这是本实验的核心部分。我们将在 Gazebo 中运行 TurtleBot3 仿真，机器人将利用相机检测红色目标、计算其三维位置，并向目标移动。

#### 3.1 启动仿真与感知节点

我们提供的启动文件将同时启动：
- Gazebo 中的 TurtleBot3 仿真环境
- `perception_node`（负责目标检测与运动控制）
- RViz（用于可视化）

```bash
# 如有需要请先关闭之前的终端，然后运行：
roslaunch lab2_perception lab2.launch
```

#### 3.2 观察现象
1. **Gazebo 界面**：
   - 你应看到 TurtleBot3 位于一个空白世界中。
   - *操作*：在机器人前方插入一个红色方块（或圆柱）。
     - 在 Gazebo 中点击 “Insert” 标签 → `http://gazebosim.org/models/` → `Unit Box`
     - 右键点击方块 → `Edit model` → 将颜色修改为红色  
     - *替代方案*：如果不方便生成红色物体，请确保相机视野中有红色物体（代码默认检测红色）。
2. **OpenCV 窗口**：
   - **Original RGB**：显示相机画面，检测到的目标会被圆圈标注，并显示其 (X, Y, Z) 坐标。
   - **HSV Result**：显示颜色分割后的二值掩膜图像。
   - **Canny Edges**：显示边缘检测结果。
3. **视觉伺服行为**：
   - 当检测到红色目标时，机器人会先旋转对准目标，然后向前移动，直到与目标保持约 0.5 m 的距离。

#### 3.3 终端中的数据可视化
查看自定义消息与控制指令：

```bash
# 打开新终端
source ~/catkin_ws/devel/setup.bash

# 查看检测到的目标坐标
rostopic echo /detected_object

# 查看发送给机器人的速度指令
rostopic echo /cmd_vel
```

---

### 第四部分：三维点云可视化

我们可以使用 Open3D 库将深度数据可视化为三维点云。

#### 4.1 运行可视化程序
在第三部分的仿真仍在运行时：

```bash
# 打开新终端
source ~/catkin_ws/devel/setup.bash

# 运行 Open3D 点云可视化程序
rosrun lab2_perception pointcloud_visualizer.py
```

**操作说明**：
- 将弹出一个名为 “3D Point Cloud” 的新窗口。
- **左键拖动**：旋转视角  
- **右键拖动**：平移视角  
- **滚轮**：缩放  
- 你将看到机器人前方环境的三维重建效果。

---

### 第五部分：TF 坐标变换

`perception_node` 会自动广播检测到的目标在机器人坐标系中的位置。

1. **RViz**：
   - 启动文件会自动打开 RViz。
   - 确保在显示列表中勾选 “TF”。
   - 你可以观察 `camera_rgb_optical_frame`（相机坐标系）与 `base_footprint`（机器人底座坐标系）之间的关系。
   - 代码会计算目标在 `base_footprint` 下的位置，并在节点输出中打印。

---

## 5. 关键命令汇总

| 功能 | 命令 |
|------|------|
| **编译功能包** | `catkin_make -DCATKIN_WHITELIST_PACKAGES="lab2_perception"` |
| **生成测试图像** | `rosrun lab2_perception generate_images.py` |
| **HSV 调节工具** | `rosrun lab2_perception hsv_tuner.py` |
| **启动实验** | `roslaunch lab2_perception lab2.launch` |
| **3D 点云可视化** | `rosrun lab2_perception pointcloud_visualizer.py` |
| **查看话题** | `rostopic echo /detected_object` |

---

## 6. 常见问题排查

- **错误：“ModuleNotFoundError: No module named 'open3d'”**  
  使用 pip 安装 Open3D：
  ```bash
  pip install open3d
  ```

- **机器人原地打转**  
  请检查红色目标是否在相机视野中。如果未检测到目标，机器人可能停止或保持上一条指令（取决于控制逻辑）。确认 HSV 阈值与目标颜色匹配。

- **相机无数据**  
  请确认 Gazebo 正在运行且未处于暂停状态。

---

如果你需要，我也可以：
- 帮你整理成**实验指导书 / 教学 PPT**
- 精简为**学生实验步骤版**
- 或针对 **ROS Noetic 教学**做术语统一与注释说明