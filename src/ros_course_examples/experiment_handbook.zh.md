# ROS 机器人实验手册

本手册旨在指导学生完成 ROS 基础通信、机器人运动控制、多机协同及自主追踪等实验。通过本课程，学生将深入理解 ROS 的节点通信机制（话题、服务）、坐标系转换（TF）以及 Gazebo 仿真环境的使用。

---

## 实验一：ROS 基础通信体验

### 1. 实验目的
*   理解 ROS 节点（Node）的概念。
*   掌握话题（Topic）通信机制：发布者（Publisher）与订阅者（Subscriber）。
*   掌握服务（Service）通信机制：服务端（Server）与客户端（Client）。
*   体验图像数据在 ROS 中的传递。

### 2. 实验步骤
1.  **启动实验**
    在终端中运行以下命令：
    ```bash
    cd ~/catkin_ws
    source devel/setup.bash
    rosrun ros_course_examples run_basic_demo.sh
    ```
2.  **观察现象**
    *   **终端输出**：你会看到 `talker` 节点在发布 "Hello ROS world"，`listener` 节点在接收并打印。
    *   **RViz 窗口**：左下角会显示一个绿色的动态圆环图像，这是通过 ROS 话题传输的 OpenCV 生成图像。
    *   **服务调用**：你可以打开一个新的终端，尝试手动调用加法服务：
        ```bash
        source devel/setup.bash
        # 语法: rosrun ros_course_examples add_two_ints_client.py <num1> <num2>
        rosrun ros_course_examples add_two_ints_client.py 10 20
        ```
        你应该能看到返回结果 `30`。

### 3. 实验原理
*   **话题 (Topic)**：一种异步通信方式，用于连续数据流（如传感器数据、日志）。发布者只管发，订阅者只管收，互不等待。
*   **服务 (Service)**：一种同步通信方式，用于请求/响应模式（如“计算这两个数的和”、“拍一张照片”）。客户端发送请求后会等待服务端处理完毕并返回结果。

### 4. 关键代码讲解
*   **发布者 (`nodes/topic_publisher.py`)**
    ```python
    # 创建发布者：话题名 'chatter'，消息类型 String，队列大小 10
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # 设置频率 10Hz
    rate = rospy.Rate(10)
    ```
    *   **注意**：`queue_size` 很重要，如果发送太快处理太慢，队列满了会丢弃旧消息。

*   **图像发布 (`nodes/image_publisher.py`)**
    ```python
    # 使用 CvBridge 将 OpenCV 图片转换为 ROS 消息
    bridge = CvBridge()
    ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
    image_pub.publish(ros_image)
    ```
    *   **易错点**：OpenCV 默认是 BGR 格式，而 ROS 中有时默认为 RGB，转换参数 `"bgr8"` 必须正确，否则颜色会反转（红变蓝）。

---

## 实验二：Turtlebot 运动控制与可视化

### 1. 实验目的
*   学会通过代码控制移动机器人（发送 `cmd_vel`）。
*   理解线速度（Linear）和角速度（Angular）的关系。
*   掌握 RViz 可视化工具的使用（LaserScan, RobotModel, TF）。
*   理解里程计（Odometry）和四元数（Quaternion）。

### 2. 实验步骤
1.  **启动实验**
    ```bash
    rosrun ros_course_examples run_shape_demo.sh
    ```
2.  **选择形状**
    根据终端提示输入数字：
    *   `1`: 圆形移动
    *   `2`: 正方形移动
    *   `3`: 长方形移动
3.  **观察 RViz**
    *   你将看到机器人在移动，同时雷达扫描到的红点（墙壁）也会随之移动。
    *   观察 `TF` 坐标系（红绿蓝箭头），理解 `odom`（里程计坐标系）和 `base_footprint`（机器人底盘坐标系）之间的关系。

### 3. 实验原理
*   **运动控制**：通过向 `/cmd_vel` 话题发布 `geometry_msgs/Twist` 消息。
    *   `linear.x`: 前后移动速度 (m/s)
    *   `angular.z`: 左右旋转速度 (rad/s)
*   **闭环与开环**：本实验演示的是简单的**开环控制**（基于时间或简单的里程计反馈），机器人可能会走歪，这是正常的，精确控制需要 PID 算法。

### 4. 关键代码讲解 (`nodes/move_turtlebot.py`)
*   **走正方形逻辑**
    ```python
    def move_square(self):
        side_length = 1.0
        # 走四条边
        for _ in range(4):
            self.go_straight(side_length) # 直行
            self.turn_90_degrees()        # 旋转90度
    ```
*   **坐标转换 (四元数转欧拉角)**
    ```python
    # 里程计消息中，方向是四元数(x,y,z,w)，人类难以直观理解
    # 需要转换为欧拉角(roll, pitch, yaw)，平面移动主要关注 yaw (偏航角)
    quaternion = (msg.pose.pose.orientation.x, ...)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    self.current_theta = euler[2] # 获取 yaw
    ```
    *   **重要**：角度处理时要注意 `0` 到 `2*pi` 或 `-pi` 到 `pi` 的跳变问题。

---

## 实验三：双人/多机追捕游戏

### 1. 实验目的
*   **版本一（单机双控）**：体验多机器人协同仿真，理解共享资源的竞争（键盘控制）。
*   **版本二（多机通信）**：**核心重点**，理解 ROS `MASTER_URI` 和 `ROS_IP` 的配置，实现跨设备通信。

### 2. 实验步骤
#### 版本一：本地双人游戏
1.  **启动**
    ```bash
    rosrun ros_course_examples run_game_v1.sh
    ```
2.  **操作**
    *   **玩家 1 (Prey/被追者)**：使用键盘 `W/A/S/D` 控制 Robot 1。
    *   **玩家 2 (Chaser/追捕者)**：使用键盘 `方向键 (↑/↓/←/→)` 控制 Robot 2。
    *   **规则**：当两车距离小于 1 米时，游戏结束，裁判节点会锁定所有机器人。

#### 版本二：多机联调 (两台电脑)
1.  **网络设置**：确保两台电脑在同一局域网下（能互相 ping 通）。
2.  **主机 (运行 roscore 和 Gazebo)**：
    ```bash
    # 在主机终端运行
    source src/ros_course_examples/scripts/setup_master.sh
    roslaunch ros_course_examples multi_turtlebot_game.launch
    ```
3.  **从机 (运行控制节点)**：
    ```bash
    # 在从机终端运行 (假设主机IP为 192.168.1.100)
    source src/ros_course_examples/scripts/setup_slave.sh 192.168.1.100
    rosrun ros_course_examples dual_teleop.py
    ```
4.  **现象**：从机可以通过键盘控制主机 Gazebo 中的机器人，证明 ROS 消息成功跨网络传输。

### 3. 关键代码讲解 (`nodes/dual_teleop.py`)
*   **多机控制原理**
    ```python
    # 分别创建两个发布者，发布到不同机器人的命名空间下
    pub1 = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
    pub2 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
    ```
    *   **注意**：在多机系统中，命名空间（Namespace）至关重要，用于区分同类型的不同实体。

---

## 实验四：编程挑战 - 自动追踪机器人

### 1. 实验目的
*   综合运用 ROS 订阅（获取坐标）和发布（控制运动）。
*   实现简单的追踪算法（P控制）。

### 2. 实验步骤
1.  **启动环境**
    ```bash
    rosrun ros_course_examples run_game_v3.sh
    ```
    *   此时 Robot 1 (Prey) 会开始随机游走。
    *   Robot 2 (Chaser) 原地不动（因为代码没写完）。
2.  **编写代码**
    打开 `src/ros_course_examples/nodes/student_chaser_template.py`，完成 `chase_logic` 函数。
3.  **目标**
    使 Robot 2 自动追上 Robot 1。

### 3. 关键代码提示
你需要实现以下逻辑：
1.  **计算目标角度**：使用 `math.atan2(dy, dx)` 计算从我指向猎物的角度。
2.  **计算角度误差**：`error_yaw = target_angle - current_yaw`。
    *   **难点**：角度归一化。例如 `3.14` 和 `-3.14` 其实很近，直接相减误差巨大。需要处理成 `-pi` 到 `pi` 之间。
3.  **控制律**：
    ```python
    cmd.angular.z = Kp_turn * error_yaw  # 转向猎物
    cmd.linear.x = Kp_dist * distance    # 向前冲 (当角度误差较小时再加速)
    ```

### 4. 常见错误
*   **坐标系混淆**：`atan2` 返回的是全局坐标系下的角度，需要减去机器人当前的 `yaw` 才是机器人需要转动的角度。
*   **忘记处理回调**：如果 `self.prey_pose` 还是 `None`（消息还没来），直接计算会报错。务必保留 `if self.prey_pose is None: return` 的判断。
