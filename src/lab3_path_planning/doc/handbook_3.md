# Lab 3: Path Planning Handbook

## 1. Introduction
In this lab, you will learn the fundamentals of global path planning in ROS. You will work with a provided Python node that implements the basic structure of a path planner but lacks the core search algorithm. Your task is to implement a path planning algorithm (such as A* or Dijkstra) to allow the robot to navigate from a start position to a goal position while avoiding obstacles.

## 2. Package Overview
The `lab3_path_planning` package contains the following key files:

*   **`launch/path_planning_ex.launch`**: The main launch file. It starts:
    *   **Gazebo**: The simulation environment (TurtleBot3 Waffle).
    *   **Map Server**: Loads the map (`simple.yaml`).
    *   **AMCL**: For robot localization.
    *   **RViz**: For visualization.
    *   **`path_planning_node.py`**: The custom path planning node you will modify.
*   **`scripts/path_planning_node.py`**: The Python script implementing the planner structure.
    *   Subscribes to `/map` (OccupancyGrid), `/move_base_simple/goal` (Goal), and `/amcl_pose` (Current Pose).
    *   Publishes `/my_planned_path` (Path) and `/cmd_vel` (Twist).
    *   Contains helper functions for coordinate conversion (`world_to_grid`, `grid_to_world`) and collision checking (`is_valid`).

## 3. Launch File Analysis
The launch file `path_planning_ex.launch` is designed to set up the complete navigation stack but isolate the global planner for your custom implementation.

```xml
<launch>
  <!-- ... arguments ... -->

  <!-- Start the Custom Path Planner Node -->
  <node pkg="lab3_path_planning" type="path_planning_node.py" name="my_path_planner" output="screen"/>

  <!-- move_base configuration -->
  <node pkg="move_base" type="move_base" ...>
    <!-- Remap goal so move_base doesn't automatically start its own planner -->
    <remap from="/move_base_simple/goal" to="/move_base_simple/goal_ignored"/>
    <!-- ... -->
  </node>
  <!-- ... -->
</launch>
```
**Key Point:** The `/move_base_simple/goal` topic (published by RViz's "2D Nav Goal" tool) is remapped for the standard `move_base` node so that it doesn't interfere. Your custom node `my_path_planner` listens to the original topic, giving you full control over the planning process.

## 4. Path Planning Node Construction
The `SimplePathPlanner` class in `scripts/path_planning_node.py` is structured as follows:

### 4.1. Initialization
*   Subscribers and Publishers are set up.
*   A timer (`control_loop`) is created to execute the path tracking logic at 10Hz.

### 4.2. Callbacks
*   **`map_callback`**: Updates the internal map data (`self.map_data`).
*   **`pose_callback`**: Updates the robot's current position (`self.current_pose`).
*   **`goal_callback`**: 
    1.  Triggered when you set a goal in RViz.
    2.  Stops current movement.
    3.  Calls `self.plan_path(start, goal)`.
    4.  If a path is found, it publishes the path for visualization and enables the tracking controller.

### 4.3. The `plan_path` Function (Your Task)
Currently, this function implements a naive **straight-line planner**:
```python
def plan_path(self, start_pose, goal_pose):
    # ...
    # Simple check for bounds
    # Returns a straight line path
    # ...
```
This simply connects the start and goal points without checking for obstacles (except at the endpoints). Your goal is to replace this logic.

### 4.4. Helper Functions
Use these functions in your algorithm:
*   `world_to_grid(wx, wy)`: Converts World coordinates (meters) to Map Grid coordinates (indices).
*   `grid_to_world(gx, gy)`: Converts Map Grid coordinates to World coordinates.
*   `is_valid(gx, gy)`: Checks if a grid cell is within bounds and free of obstacles.

## 5. Practical Steps

### Step 1: Run the Existing Code
First, verify the environment and the default behavior.

1.  **Build the package**:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

2.  **Launch the simulation**:
    ```bash
    roslaunch lab3_path_planning path_planning_ex.launch
    ```
    *   Gazebo and RViz should open.
    *   The map should be visible in RViz.

3.  **Localize the Robot**:
    *   In RViz, use the **"2D Pose Estimate"** tool.
    *   Click and drag on the map to match the robot's position in Gazebo.
    *   The laser scan (red dots) should align with the map (black lines).

4.  **Set a Goal**:
    *   Use the **"2D Nav Goal"** tool in RViz.
    *   Click a point on the map.
    *   **Observation**: You will see a green line (the path) drawn directly from the robot to the goal, even if it goes through walls. The robot will try to follow it and may crash.

### Step 2: Implement A* (or Dijkstra) Algorithm
Open `src/lab3_path_planning/scripts/path_planning_node.py` and modify the `plan_path` method.

**Suggested Implementation Plan:**

1.  **Initialize Open and Closed Sets**:
    *   Use `heapq` for the Open Set (Priority Queue).
    *   Use a `set` or 2D array for the Closed Set (Visited nodes).
    *   Store costs: `g_cost` (cost from start) and `f_cost` (g + heuristic).
    *   Store parents: To reconstruct the path later.

2.  **Main Loop**:
    *   While Open Set is not empty:
        *   Pop the node with the lowest `f_cost`.
        *   If it is the goal node, **reconstruct path** and return.
        *   Add to Closed Set.
        *   Check all 8 neighbors (or 4 neighbors).

3.  **Neighbor Expansion**:
    *   For each neighbor:
        *   Check `is_valid(nx, ny)`.
        *   Calculate tentative `g_cost`.
        *   If better than existing path to neighbor, update parent and costs, and add to Open Set.

4.  **Heuristic (for A*)**:
    *   Use Euclidean distance or Manhattan distance between current node and goal.

**Code Snippet Example (Pseudocode Hint):**
```python
open_list = []
heapq.heappush(open_list, (0, start_grid))
came_from = {}
g_score = {start_grid: 0}

while open_list:
    current = heapq.heappop(open_list)[1]
    
    if current == goal_grid:
        # Reconstruct path...
        break
        
    for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]: # 4-connectivity
        neighbor = (current[0]+dx, current[1]+dy)
        if not self.is_valid(neighbor[0], neighbor[1]):
            continue
            
        tentative_g = g_score[current] + 1 # Assume cost 1 per step
        
        if neighbor not in g_score or tentative_g < g_score[neighbor]:
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score = tentative_g + heuristic(neighbor, goal_grid)
            heapq.heappush(open_list, (f_score, neighbor))
```

### Step 3: Verify and Tune
1.  Save your changes.
2.  Restart the `path_planning_node.py` (or the whole launch file).
    *   *Tip: You can keep Gazebo/RViz running and just restart the node in a separate terminal:*
        ```bash
        rosrun lab3_path_planning path_planning_node.py
        ```
3.  Set a goal behind an obstacle.
4.  **Success Criteria**:
    *   The generated path (green line) should go **around** the obstacle.
    *   The robot should follow the path to the goal.

## 6. Tips
*   **Grid Resolution**: Remember that the grid coordinates are integers.
*   **Safety Margin**: The default `is_valid` checks if a cell is occupied. You might want to consider the robot's radius by checking neighbors of obstacles too, or simply trust the inflation layer in the costmap if you were using `move_base`'s global planner (but here we are using raw map data, so walls are thin).
*   **Coordinate Transform**: Pay attention to the center of the pixel vs. the corner when converting `grid_to_world`. The provided helper adds `resolution / 2.0` which is good.
