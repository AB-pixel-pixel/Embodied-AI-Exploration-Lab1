# Experiment 2: ROS Basics and Communication Mechanisms

## 1. Experiment Objectives

* Understand the concept of ROS Nodes
* Master the Topic publish-subscribe mechanism
* Understand the role of the ROS Master

## 2. Core ROS Concepts

### 2.1 Basic Components

* **Node** : The basic operational unit of a ROS system; each node performs a specific task.
* **Topic** : A named channel for communication between nodes.
* **Master** : Provides coordination services for communication between nodes (requires `roscore`).

### 2.2 Node Communication

The most common way nodes communicate is based on topics. A publisher node names a topic and publishes messages to it. Another subscriber node subscribes to that topic.
![1766756911148](image/2.ros_experiment/1766756911148.png)

## 3. Experiment Steps

First, download the course code repository from GitHub and rename it to catkin_ws.

```bash
git clone https://github.com/AB-pixel-pixel/Embodied-AI-Exploration-Lab1.git
mv Embodied-AI-Exploration-Lab1 catkin_ws
cd catkin_ws
```

### Experiment 2.1: Running two isolated programs

```bash
cd ~/catkin_ws/src/ros_course_examples/simulation_demo
python3 controller.py
```

```bash
# Terminal 2: Run Motor
cd ~/catkin_ws/src/ros_course_examples/simulation_demo
python3 motor.py
```

Execution result shown in the figure:

 ![1766556330841](image/2.ros_experiment/1766556330841.png)

 **Observation** : The two programs run independently and cannot communicate with each other.

Close these two programs now.

### Experiment 2.2: Implementing Node Communication using ROS

We have already encapsulated the code into ROS nodes, the files are as follows:
`src/ros_course_examples/nodes/motor_node.py` and `src/ros_course_examples/nodes/controller_node.py`.

As long as we start these two nodes, we can achieve node communication based on the ROS framework.

First, perform compilation.

In Terminal 1: Compile the ROS workspace.

We use the `catkin_make` command to compile ROS packages. Its functions are:

* Compile source code: Compile C++ source files into executables, mark Python scripts as executable.
* Handle dependencies: Automatically resolve and link dependencies between packages.
* Generate configuration files: Create environment configuration scripts such as devel/setup.bash.
* Build message types: Compile custom msg, srv, and action files.

```bash
cd ~/catkin_ws
catkin_make
```

Rough schematic:
![1766556827503](image/2.ros_experiment/1766556827503.png)

After execution it will:

* Compile code in the build/ directory.
* Output results to the devel/ directory.
* Generate environment variables usable by ROS.

Terminal 1: Start the ROS Master node.

The `roscore` command can start the Master node:

* Function: Manages registration and discovery of all nodes.
* Role: Allows Publishers and Subscribers to find each other.
* Analogy: Like a central server, connecting nodes.
* **Note: During the experiment, we do not close the terminal running roscore.**

```bash
roscore
```

![1766557292995](image/2.ros_experiment/1766557292995.png)

Grant execution permissions to the node code:

```bash
cd ~/catkin_ws/src/ros_course_examples/nodes/  
chmod +x motor_node.py
chmod +x controller_node.py
```

![1766557275580](image/2.ros_experiment/1766557275580.png)

**Start Communication Demo**

Since we need to use the ROS framework, we need to use ROS commands to start the code instead of simply using python commands.

This is the `rosrun` command:

* Function: Run a node within a ROS package.
* Features: Simple, direct, suitable for testing or debugging.
* Example usage:

```bash
# rosrun <ros package name> <executable file name>
rosrun turtlesim turtlesim_node
```

* If the executable file requires arguments, they can also follow directly: rosrun pkg exe arg1 arg2 ...

Terminal 2: Start the motor node.

**Note: `source devel/setup.bash` helps ros commands find corresponding executable files.**

```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun ros_course_examples motor_node.py
```

You will see that this node (program) is waiting for information, and its position does not change.
![1766558557910](image/2.ros_experiment/1766558557910.png)

Terminal 3: Start the controller node.
Next, let's control it.

```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun ros_course_examples controller_node.py
```

Observing the terminal, we can discover that it is constantly sending commands:
![1766559045635](image/2.ros_experiment/1766559045635.png)

Return to view Terminal 2 (Motor node).
![1766559088885](image/2.ros_experiment/1766559088885.png)

**Observed**:The Controller sends speed commands, and the Motor receives them and updates the position.

---

However, wouldn't it be too complicated to enter a command line every time a program is started?

ROS provides a unified startup configuration mechanism that can start multiple nodes with one click; this is the `roslaunch` command.

`roslaunch`:

* Function: Start multiple nodes at once through .launch files.
* Set parameters for nodes, for example, setting parameters for relevant planning algorithms according to the size of the robot.
* Remap topics, suitable for changing communication relationships without changing code.
* Set namespaces, suitable for multiple robots using the same code.
* Launch files are usually located in the launch/ directory of the package (not mandatory, but convention).
* Example:

```bash
# roslaunch <ros package name> <launch file name within package>
roslaunch turtlesim turtlesim_demo.launch
```

Close Terminal 2 and Terminal 3, execute in Terminal 3:

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch ros_course_examples ros_communication_demo.launch
```

You can observe that both nodes have started.

![1766559251701](image/2.ros_experiment/1766559251701.png)

Before proceeding to the following experiments, you can close this program first ( **Note: do not close the terminal running roscore** ).

### Experiment 2.3: TurtleSim Communication Experiment

Terminal 2: Start Turtle Simulation

```bash
rosrun turtlesim turtlesim_node
```

![1766559377818](image/2.ros_experiment/1766559377818.png)

Terminal 3: Start Keyboard Control

```bash
rosrun turtlesim turtle_teleop_key
```

![1766559474890](image/2.ros_experiment/1766559474890.png)

Pressing arrow keys in Terminal 3 can control the turtle movement.
![1766559507381](image/2.ros_experiment/1766559507381.png)

Terminal 4: Visualize Communication Graph

```bash
rqt_graph
```

![1766559792210](image/2.ros_experiment/1766559792210.png)

 **Communication Process Analysis** :

1. `turtle_teleop_key` node listens for keyboard input.
2. Publishes speed commands to the `/turtle1/cmd_vel` topic.
3. The information published to the topic is linear speed and angular speed; the message type is `geometry_msgs/Twist`.
4. `turtlesim_node` subscribes to `/turtle1/cmd_vel`.
5. Receives speed commands and executes movement.

### Experiment 2.4: Viewing Topic Information

All the above information can be observed through ROS commands.

```bash
# List all topics
rostopic list
```

```bash
# View topic information
rostopic info /turtle1/cmd_vel
```

```bash
# View message type definition
rosmsg show geometry_msgs/Twist
```

![1766560209464](image/2.ros_experiment/1766560209464.png)

```bash
# View topic data
rostopic echo /turtle1/cmd_vel
```

![1766559946201](image/2.ros_experiment/1766559946201.png)

---

# Experiment 3: Gazebo Simulation Environment

## 1. Experiment Objectives

* Master the use of the Gazebo simulator
* Learn to load and save simulation worlds
* Understand the World file structure
* Master the loading of robot models
* Message Type

## 2. Gazebo Core Functions

* **Physics Engine** : Simulates real physical laws (gravity, collisions, friction).
* **Sensor Simulation** : LiDAR, cameras, IMU, etc.
* **ROS Integration** : Seamless communication with ROS.
* **Visualization** : 3D scene rendering.

## 3. Experiment Steps

### 3.1 Start Empty World

```bash
gazebo
```

![1766587986896](image/2.ros_experiment/1766587986896.png)

Or start using ROS

```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

![1766588066542](image/2.ros_experiment/1766588066542.png)

### 3.2 Build Custom Scene

1. **Insert Models** : Drag objects from the left panel into the scene.
2. **Adjust Parameters** :

* Position (x, y, z): Position coordinates
* Orientation (roll, pitch, yaw): Attitude angles
* Scale: Zoom size

1. **Save World** : `File -> Save World As -> my_world.world`

![1766589618862](image/2.ros_experiment/1766589618862.png)

![1766589642312](image/2.ros_experiment/1766589642312.png)

### 3.3 Load Custom World

```bash
gazebo my_world.world
```

Method 2: ROS launch file

```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

![1766590157072](image/2.ros_experiment/1766590157072.png)

### 3.4 Control Robot Movement

```bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

 **Operating Instructions** :

* `W/A/S/D` or `Arrow Keys`: Control movement
* `X`: Stop
* `Q/Z`: Increase/decrease speed

![1766590095927](image/2.ros_experiment/1766590095927.png)

---

# Experiment 4: RViz Visualization

## 1. Experiment Objectives

* Master the use of the RViz visualization tool
* Learn to add and configure display items
* Understand visual representation of sensor data
* Master interface interaction and viewpoint control

## 2. Data Types Displayable by RViz

* **Robot Model** : 3D robot model
* **LaserScan** : LiDAR scan data
* **PointCloud2** : 3D point cloud data
* **TF** : Coordinate system transformation relationships
* **Image** : Camera images
* **Odometry** : Odometry trajectory
* **Path** : Planned path
* **Map** : Occupancy grid map

## 3. Introduction to RViz Startup Methods

Start directly

```bash
rviz
```

![1766591530072](image/2.ros_experiment/1766591530072.png)

### 3.1 Basic RViz Interface Operations

#### **Viewpoint Control**

* **Left mouse drag** : Rotate view
* **Mouse wheel** : Zoom view
* **Shift + Left drag** : Pan view
* **Shift + Wheel** : Pan up/down
* **Middle mouse drag** : Pan (some systems)

## 4. Experiment Steps

### 4.1 Start Simulation and Visualization

```bash
# Terminal 1: Start gazebo
source ~/catkin_ws/devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

```bash
# Terminal 2: Start RViz
source ~/catkin_ws/devel/setup.bash
roslaunch turtlebot_rviz_launchers view_robot.launch 
```

![](image/2025-12-26-180703.png)

### 4.2 Detailed Steps to Add Display Items (Display)

#### **Example 1: Adding LiDAR Data** (Or just check the LaserScan box, like if the rviz launch already configured rviz)

1. Click the **"Add"** button in the bottom left corner.
2. Select the **"By display type"** tab in the pop-up window.
3. Find and double-click  **"LaserScan"** .
4. Expand **"LaserScan"** in the left Displays panel.
5. Configure parameters:

```bash
   Topic: /scan           # Click dropdown to select /scan
   Size (m): 0.05         # Adjust point size
   Style: Points          # Display style
   Color Transformer: Intensity  # Color map
```

6. Observe red scan points displaying obstacle positions.

![](image/2025-12-26-180847.png)

#### **Example 2: Adding Coordinate Systems (TF)**

1. Click **"Add"** -> Select  **"TF"** .
2. Configure parameters:

```bash
     Show Axes           # Show axes
     Show Arrows         # Show arrows
   Marker Scale: 0.5     # Adjust axes size
   Update Interval: 0    # Update interval (0=fastest)
```

3. Observe red, green, and blue arrows (representing X/Y/Z axes).

![](image/2025-12-26-181204.png)

### 4.3 Configuration Table for Common Display Items

| Display Type         | Recommended Topic   | Function                        | Key Parameters                           |
| -------------------- | ------------------- | ------------------------------- | ---------------------------------------- |
| **RobotModel** | (Default)           | Displays 3D robot model         | `Robot Description: robot_description` |
| **LaserScan**  | `/scan`           | LiDAR scan data                 | `Size: 0.05`,`Style: Points`         |
| **Odometry**   | `/odom`           | Odometry trajectory             | `Keep: 100`,`Shape: Arro`            |
| **TF**         | (No setting needed) | Coordinate system relationships | `Show Names:  `,`Marker Scale: 0.5`  |
| **Map**        | `/map`            | Occupancy grid map              | `Color Scheme: map`                    |

### 4.4 Adjust Fixed Frame (Reference Coordinate System)

**What is Fixed Frame?**

* The display of all data in RViz requires a reference coordinate system.
* Different scenarios require selecting different Fixed Frames.

**Selection Suggestions**

| Scenario                 | Fixed Frame   | Effect                          |
| ------------------------ | ------------- | ------------------------------- |
| Observing robot movement | `odom`      | Viewpoint follows robot         |
| Debugging sensors        | `base_link` | Viewpoint locked onto the robot |

**Setting Method**

1. Expand **"Global Options"** at the top.
2. Click the **"Fixed Frame"** dropdown menu.
3. Select `odom`.

 **Note** : If Fixed Frame is set incorrectly, all display items will turn gray or not show.

### 4.5 View TF Tree

**Command to view TF tree diagram**

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

**View TF Transforms**

![](image/2025-12-26-180520.png)

## 6. Common Troubleshooting

| Issue                       | Cause                | Solution                               |
| --------------------------- | -------------------- | -------------------------------------- |
| Display items turn red/gray | Topic not published  | `rostopic list`Check if topic exists |
| Cannot see robot model      | Fixed Frame error    | Change to `odom` or `base_link`    |
| Laser data does not show    | Wrong Topic selected | Confirm it is `/scan`                |
| TF shows "No transform"     | TF tree incomplete   | Check `rosrun tf view_frames`        |

---

## Suggestions for After-class Practice

* Practice Linux command-line operations more.
* Try modifying sample code parameters and observe the effects.
* Use `rqt_graph` and `rostopic` tools to analyze the system.
* Read the official ROS Wiki documentation.

## Recommended Resources

* [ROS Wiki](http://wiki.ros.org/)
* [Gazebo Tutorials](http://gazebosim.org/tutorials)
* [TF Tutorials](http://wiki.ros.org/tf/Tutorials)
