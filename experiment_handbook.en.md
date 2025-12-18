# Quick Start Guide to Linux

Linux is an open-source operating system, and its kernel was created by Linus Torvalds in 1991.

Many companies and organizations build their own Linux distributions based on the Linux operating system, such as Google, Red Hat, and Ubuntu.

Currently, the ROS system mainly runs on the Ubuntu operating system, and Ubuntu itself is based on the Debian Linux distribution.

Therefore, if you want to deeply learn and practice robotics, it is best to install the Ubuntu operating system on your own computer, using WSL or a virtual machine such as VirtualBox or VMware.

More importantly, most artificial intelligence environments today are based on the Ubuntu operating system. For this reason, learning and practicing robotics is best done in an Ubuntu environment.

## Terminal



First, we need to get familiar with something: the terminal.

The terminal is a text-based interface where users can enter commands, and the operating system executes corresponding actions based on those commands.

In the Ubuntu operating system, the terminal is a very important tool. Users can run various commands in the terminal, such as installing software, configuring the system, and running programs.

By using the terminal shortcut **Ctrl + Alt + T**, you can open an interface similar to the one shown below:

![basic.png](src/ros_course_examples/doc/images/basic.png)

In this figure, `ta` is the username, the part after the `@` symbol is the hostname (`ShaobinLing`), `:` is a separator, and `~` represents the current user’s home directory, which is also the path where the terminal is currently located.

You can type commands in this black window to perform corresponding operations. For example, entering the `ls` command lists the files and directories in the current directory.

You can also use the `cd` command to change the current directory. For example, entering:

```bash
cd ~/
```

will switch to the current user’s home directory.

### Basic Linux Commands Experiment (Step by Step)

#### ⚠️ Safety Notice for the Experiment (Very Important)

In this experiment, you will encounter **commands that will actually modify files and directories**, so it is crucial to read the following:

- **`rm` and `mv` will directly delete or move files.**
- **`rm -rf` is extremely dangerous; if you write the path incorrectly, data cannot be recovered.**
- **This experiment only allows operations within `~` (your home directory).**
- **It is strictly forbidden to run `rm -rf` in system directories like `/`, `/home`, `/usr`, etc.**
- **If system damage occurs due to incorrect operations, you will bear the responsibility for repair or compensation.**

👉 **Feel free to ask questions if you don't understand something.**

---

#### I. Experiment Goal

Through a complete mini experiment, you will learn the following:

- The **concept of paths** in the Linux terminal.
- Common file/directory operations:  
  `ls mkdir touch cp mv rm find cat`
- Using `gedit` to create and edit files.
- Running Python programs in different ways and understanding:
  - Relative paths.
  - Absolute paths.
  - `~` (home directory).
- Basic system and network commands: `ping`, `top`.
- How to execute a **complete automation script**.

---

#### II. Experiment 1: Manual Operations (Must be Done First)

##### 1️⃣ Check Your Current Location

```bash
pwd
ls
```

Make sure you are in your **home directory (~)**.

---

##### 2️⃣ Create an Experiment Workspace

```bash
mkdir linux_exp
cd linux_exp
ls
```

---

##### 3️⃣ Create Files and Directories

```bash
mkdir src
touch note.txt
ls
```

---

##### 4️⃣ Use `gedit` to Create a Python File

```bash
gedit src/hello.py
```

In the opened editor, type and save:

```python
print("Hello Linux")
```

---

##### 5️⃣ Use `find` to Search for Files

```bash
find ~ -name "hello.py"
```

Observe the output path.

---

##### 6️⃣ View File Content

```bash
cat src/hello.py
```

---

##### 7️⃣ Copy, Move, and Delete Files (Be Careful)

```bash
cp src/hello.py hello_copy.py
mv hello_copy.py hello_moved.py
rm hello_moved.py
ls
```

⚠️ **Do not use `rm -rf /` or randomly delete directories.**

---

#### III. Experiment 2: Running Python and Understanding Paths

Make sure you are still in the `~/linux_exp` directory:

```bash
pwd
```

##### Method 1: Relative Path

```bash
python src/hello.py
```

---

##### Method 2: Home Path

```bash
python ~/linux_exp/src/hello.py
```

---

##### Method 3: Absolute Path

```bash
python /home/<username>/linux_exp/src/hello.py
```

> Please replace `<username>` with the username in your terminal (the string before the `@` symbol).

---

##### Reflection

- Why do all three methods work?
- If you `cd ~`, which ones will still work?

---

#### IV. Experiment 3: System and Network Commands (Observe Only)

##### View Processes (Press `q` to Quit)

```bash
top
```

---

##### Test Network (Ctrl + C to Stop)

```bash
ping baidu.com
```

---

#### V. Experiment 4: One-Click Automation Script (Key Focus)

##### 1️⃣ Create the Script File

```bash
cd ~
gedit run_linux_exp.sh
```

Write the following content and save:

```bash
#!/bin/bash

echo "=== Linux Basic Experiment Script Start ==="

cd ~

echo "[1] Creating experiment directory"
mkdir -p linux_script_exp/src

echo "[2] Creating Python file"
cat << EOF > linux_script_exp/src/hello.py
print("Hello from script")
EOF

echo "[3] Searching for the file"
find ~/linux_script_exp -name "hello.py"

echo "[4] Viewing file content"
cat linux_script_exp/src/hello.py

echo "[5] Running Python (relative path)"
cd linux_script_exp
python src/hello.py

echo "[6] Running Python (absolute path)"
python ~/linux_script_exp/src/hello.py

echo "[7] Creating, copying, moving, and deleting files"
touch test.txt
cp test.txt test_copy.txt
mv test_copy.txt test_moved.txt
rm test_moved.txt

echo "=== Script Execution Complete ==="
```

---

##### 2️⃣ Add Execute Permissions

```bash
chmod +x run_linux_exp.sh
```

---

##### 3️⃣ Execute the Script

```bash
./run_linux_exp.sh
```

Observe the output at each step and understand the script's content.

---

#### VI. Experiment Summary

In this experiment, you have used and understood the following:

##### 📌 Three Ways to Write Paths

- **Absolute Path**  
  `/home/<username>/linux_exp/src/hello.py`

- **Relative Path**  
  `src/hello.py`

- **Home Path**  
  `~/linux_exp/src/hello.py`

👉 **Command success depends on: where you are and how you write the path.**

---

#### VII. Check List (Self-check)

- [ ] I know what `pwd` does.  
- [ ] I will not casually use `rm -rf`.  
- [ ] I can understand each command in the script.  
- [ ] I understand why the same Python file can be executed in multiple ways.




# ROS Robot Experiment Handbook

This manual aims to guide students through various ROS experiments, including basic communication, robot motion control, multi-robot collaboration, and autonomous tracking. Through this course, students will gain a deep understanding of ROS node communication mechanisms (topics, services), coordinate frame transformations (TF), and the usage of the Gazebo simulation environment.

---

## Experiment 1: ROS Basic Communication Experience

### 1. Experiment Objectives
*   Understand the concept of ROS nodes.
*   Master the topic communication mechanism: publisher and subscriber.
*   Master the service communication mechanism: server and client.
*   Experience the transmission of image data in ROS.

### 2. Experiment Steps
1.  **Start the experiment**
    In the terminal, run the following command:
    ```bash
    cd ~/catkin_ws
    source devel/setup.bash
    rosrun ros_course_examples run_basic_demo.sh
    ```
2.  **Observe the phenomena**
    *   **Terminal Output**: You will see the `talker` node publishing "Hello ROS world" and the `listener` node receiving and printing it.
    *   **RViz Window**: A green dynamic circular image will appear in the lower-left corner. This is an image generated by OpenCV and transmitted through a ROS topic.
    *   **Service Call**: Open a new terminal and try to manually call the addition service:
        ```bash
        source devel/setup.bash
        # Syntax: rosrun ros_course_examples add_two_ints_client.py <num1> <num2>
        rosrun ros_course_examples add_two_ints_client.py 10 20
        ```
        You should see the result `30`.

### 3. Experiment Principles
*   **Topic**: An asynchronous communication method used for continuous data streams (e.g., sensor data, logs). The publisher sends, and the subscriber receives without waiting.
*   **Service**: A synchronous communication method used for request/response patterns (e.g., "calculate the sum of two numbers," "take a photo"). The client sends a request and waits for the server to process and return the result.

### 4. Key Code Explanation
*   **Publisher (`nodes/topic_publisher.py`)**
    ```python
    # Create a publisher: Topic name 'chatter', message type String, queue size 10
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # Set the frequency to 10Hz
    rate = rospy.Rate(10)
    ```
    *   **Note**: The `queue_size` is important. If the publishing is too fast and the processing is slow, the queue will fill up and old messages will be discarded.

*   **Image Publisher (`nodes/image_publisher.py`)**
    ```python
    # Use CvBridge to convert an OpenCV image to a ROS message
    bridge = CvBridge()
    ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
    image_pub.publish(ros_image)
    ```
    *   **Common mistake**: OpenCV uses BGR format by default, whereas ROS sometimes defaults to RGB. The conversion parameter `"bgr8"` must be correct, or the colors will be inverted (red becomes blue).

---

## Experiment 2: Turtlebot Motion Control and Visualization

### 1. Experiment Objectives
*   Learn to control a mobile robot via code (send `cmd_vel`).
*   Understand the relationship between linear and angular velocities.
*   Master the use of RViz visualization tools (LaserScan, RobotModel, TF).
*   Understand odometry and quaternion concepts.

### 2. Experiment Steps
1.  **Start the experiment**
    ```bash
    rosrun ros_course_examples run_shape_demo.sh
    ```
2.  **Select shape**
    Input the number as prompted in the terminal:
    *   `1`: Circular movement
    *   `2`: Square movement
    *   `3`: Rectangular movement
3.  **Observe in RViz**
    *   You will see the robot moving, and the red points (walls) detected by the radar will also move.
    *   Observe the `TF` coordinate system (red, green, and blue arrows), and understand the relationship between `odom` (odometry coordinate system) and `base_footprint` (robot base frame).

### 3. Experiment Principles
*   **Motion Control**: By publishing `geometry_msgs/Twist` messages to the `/cmd_vel` topic.
    *   `linear.x`: Forward/backward speed (m/s)
    *   `angular.z`: Left/right rotation speed (rad/s)
*   **Open-loop vs. Closed-loop**: This experiment demonstrates simple **open-loop control** (based on time or simple odometry feedback). The robot might drift off course, which is normal; precise control requires a PID algorithm.

### 4. Key Code Explanation (`nodes/move_turtlebot.py`)
*   **Square movement logic**
    ```python
    def move_square(self):
        side_length = 1.0
        # Move along four sides
        for _ in range(4):
            self.go_straight(side_length) # Move straight
            self.turn_90_degrees()        # Turn 90 degrees
    ```
*   **Coordinate transformation (quaternion to Euler angles)**
    ```python
    # In odometry messages, orientation is given as a quaternion (x, y, z, w), which is hard for humans to understand
    # Convert to Euler angles (roll, pitch, yaw), where yaw (heading) is the key for planar movement
    quaternion = (msg.pose.pose.orientation.x, ...)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    self.current_theta = euler[2] # Get yaw
    ```
    *   **Important**: Pay attention to angle wrapping between `0` and `2*pi` or `-pi` and `pi`.

---

## Experiment 3: Multi-robot Capture Game

### 1. Experiment Objectives
*   **Version 1 (Single-machine Dual Control)**: Experience multi-robot collaboration simulation and understand resource sharing competition (keyboard control).
*   **Version 2 (Multi-machine Communication)**: **Core focus**, understand the configuration of ROS `MASTER_URI` and `ROS_IP`, and achieve communication across devices.

### 2. Experiment Steps
#### Version 1: Local Two-player Game
1.  **Start**
    ```bash
    rosrun ros_course_examples run_game_v1.sh
    ```
2.  **Operation**
    *   **Player 1 (Prey)**: Use `W/A/S/D` to control Robot 1.
    *   **Player 2 (Chaser)**: Use arrow keys (↑/↓/←/→) to control Robot 2.
    *   **Rule**: The game ends when the two robots are less than 1 meter apart, and the referee node will lock all robots.

#### Version 2: Multi-machine Setup (Two computers)
1.  **Network Setup**: Ensure both computers are on the same local network (can ping each other).
2.  **Host (Run roscore and Gazebo)**:
    ```bash
    # Run on the host terminal
    source src/ros_course_examples/scripts/setup_master.sh
    roslaunch ros_course_examples multi_turtlebot_game.launch
    ```
3.  **Slave (Run control node)**:
    ```bash
    # Run on the slave terminal (assuming host IP is 192.168.1.100)
    source src/ros_course_examples/scripts/setup_slave.sh 192.168.1.100
    rosrun ros_course_examples dual_teleop.py
    ```
4.  **Phenomenon**: The slave machine can control the robot in the Gazebo simulation on the host machine, proving successful message transmission across the network.

### 3. Key Code Explanation (`nodes/dual_teleop.py`)
*   **Multi-machine Control Principle**
    ```python
    # Create two publishers to send commands to different robots under different namespaces
    pub1 = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
    pub2 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
    ```
    *   **Note**: In multi-machine systems, namespaces are crucial for differentiating between entities of the same type.

---

## Experiment 4: Programming Challenge - Autonomous Tracking Robot

### 1. Experiment Objectives
*   Integrate ROS subscription (get coordinates) and publication (control movement).
*   Implement a simple tracking algorithm (P control).

### 2. Experiment Steps
1.  **Start environment**
    ```bash
    rosrun ros_course_examples run_game_v3.sh
    ```
    *   At this point, Robot 1 (Prey) will start wandering randomly.
    *   Robot 2 (Chaser) will remain stationary (since the code is incomplete).
2.  **Write code**
    Open `src/ros_course_examples/nodes/student_chaser_template.py` and complete the `chase_logic` function.
3.  **Objective**
    Make Robot 2 automatically chase Robot 1.

### 3. Key Code Tips
You need to implement the following logic:
1.  **Calculate target angle**: Use `math.atan2(dy, dx)` to calculate the angle from the robot to the prey.
2.  **Calculate angle error**: `error_yaw = target_angle - current_yaw`.
    *   **Challenge**: Angle normalization. For example, `3.14` and `-3.14`