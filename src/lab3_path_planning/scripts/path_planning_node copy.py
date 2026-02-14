#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
import numpy as np
import heapq
import math
import tf.transformations

class SimplePathPlanner:
    def __init__(self):
        rospy.init_node('eai_path_planner')
        
        # 订阅地图
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        # 订阅目标点
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        # 订阅当前位置
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # 发布路径
        self.path_pub = rospy.Publisher('/my_planned_path', Path, queue_size=1)
        # 发布控制指令
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.map_data = None
        self.current_pose = None
        
        # 路径跟踪相关
        self.current_path = []
        self.is_tracking = False
        self.path_index = 0
        
        # 定时器：控制循环 (10Hz)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
    def map_callback(self, msg):
        self.map_data = msg
        
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def goal_callback(self, msg):
        if self.map_data is None or self.current_pose is None:
            rospy.logwarn("Map or pose not ready")
            return
            
        # ========================================
        # 学生在这里实现自己的路径规划算法
        # ========================================
        rospy.loginfo("Received goal, planning path...")
        
        # 停止当前跟踪
        self.is_tracking = False
        self.cmd_pub.publish(Twist()) # 停车
        
        path_msg = self.plan_path(self.current_pose, msg.pose)
        
        if path_msg and len(path_msg.poses) > 0:
            rospy.loginfo("Path found! Publishing...")
            self.path_pub.publish(path_msg)
            
            # 开始跟踪
            self.current_path = path_msg.poses
            self.path_index = 0
            self.is_tracking = True
        else:
            rospy.logwarn("Failed to find a path.")

    def control_loop(self, event):
        if not self.is_tracking or self.current_pose is None or not self.current_path:
            return
            
        # 简单的纯追踪 (Pure Pursuit) 逻辑
        # 1. 寻找 Lookahead point
        lookahead_dist = 0.5 # 0.5米前瞻距离
        
        target_pose = None
        found_target = False
        
        # 从当前索引开始向后搜索
        for i in range(self.path_index, len(self.current_path)):
            p = self.current_path[i].pose.position
            dist = math.sqrt((p.x - self.current_pose.position.x)**2 + 
                             (p.y - self.current_pose.position.y)**2)
            
            if dist > lookahead_dist:
                target_pose = p
                self.path_index = i # 更新索引，防止倒车
                found_target = True
                break
        
        # 如果找不到更远的点，说明快到终点了，取最后一个点
        if not found_target:
            target_pose = self.current_path[-1].pose.position
            # 检查距离终点是否足够近
            dist_to_goal = math.sqrt((target_pose.x - self.current_pose.position.x)**2 + 
                                     (target_pose.y - self.current_pose.position.y)**2)
            if dist_to_goal < 0.1: # 10cm 容差
                rospy.loginfo("Goal reached!")
                self.is_tracking = False
                self.cmd_pub.publish(Twist()) # 停车
                return

        # 2. 计算控制量
        # 目标方位角
        angle_to_target = math.atan2(target_pose.y - self.current_pose.position.y,
                                     target_pose.x - self.current_pose.position.x)
        
        # 当前机器人朝向 (从四元数转换)
        q = self.current_pose.orientation
        _, _, current_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # 角度误差
        angle_error = angle_to_target - current_yaw
        # 归一化到 [-pi, pi]
        while angle_error > math.pi: angle_error -= 2*math.pi
        while angle_error < -math.pi: angle_error += 2*math.pi
        
        twist = Twist()
        
        # 简单的 P 控制器
        k_w = 1.5 # 角速度增益
        twist.angular.z = k_w * angle_error
        
        # 限制最大角速度
        max_w = 0.5
        twist.angular.z = max(min(twist.angular.z, max_w), -max_w)
        
        # 简单的速度控制：角度误差小的时候加速，大的时候减速
        if abs(angle_error) < 0.5:
            twist.linear.x = 0.2
        else:
            twist.linear.x = 0.0
            
        self.cmd_pub.publish(twist)

    def world_to_grid(self, world_x, world_y):
        if not self.map_data:
            return None
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        resolution = self.map_data.info.resolution
        
        grid_x = int((world_x - origin_x) / resolution)
        grid_y = int((world_y - origin_y) / resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y):
        if not self.map_data:
            return None
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        resolution = self.map_data.info.resolution
        
        world_x = grid_x * resolution + origin_x + resolution / 2.0
        world_y = grid_y * resolution + origin_y + resolution / 2.0
        return (world_x, world_y)

    def is_valid(self, grid_x, grid_y):
        if not self.map_data:
            return False
        width = self.map_data.info.width
        height = self.map_data.info.height
        
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return False
            
        # Check occupancy
        # data is row-major: index = y * width + x
        index = grid_y * width + grid_x
        # Occupancy probability [0, 100]. -1 is unknown.
        # Threshold > 50 considered occupied. Treat unknown as free or occupied? 
        # Usually treat unknown as free in exploration, but safe to treat as obstacle here?
        # Let's treat > 50 as obstacle. -1 (unknown) is risky, let's treat it as obstacle for safety.
        val = self.map_data.data[index]
        if val > 50 or val == -1: 
            return False
            
        return True

    def heuristic(self, a, b):
        # Euclidean distance
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def plan_path(self, start_pose, goal_pose):
        """
        Bug2 算法实现
        """
        start_grid = self.world_to_grid(start_pose.position.x, start_pose.position.y)
        goal_grid = self.world_to_grid(goal_pose.position.x, goal_pose.position.y)
        
        # 简单检查起点终点
        width = self.map_data.info.width
        height = self.map_data.info.height
        if not (0 <= start_grid[0] < width and 0 <= start_grid[1] < height):
             rospy.logwarn("Start is out of map bounds")
             return Path()
        if not (0 <= goal_grid[0] < width and 0 <= goal_grid[1] < height):
             rospy.logwarn("Goal is out of map bounds")
             return Path()

        # Bug2 算法状态
        STATE_GO_TO_GOAL = 0
        STATE_FOLLOW_WALL = 1
        
        state = STATE_GO_TO_GOAL
        current = start_grid
        path_points = [current]
        
        # 用于 Follow Wall 的辅助变量
        hit_point = None
        # M-line: 连接 Start 和 Goal 的直线 (用点集近似)
        m_line_points = self.get_bresenham_line(start_grid, goal_grid)
        m_line_set = set(m_line_points)
        
        visited_in_follow = set() # 防止死循环
        
        max_steps = width * height # 防止无限循环
        steps = 0
        
        while current != goal_grid and steps < max_steps:
            steps += 1
            
            if state == STATE_GO_TO_GOAL:
                # 尝试向目标直行
                next_node = self.get_next_straight_node(current, goal_grid)
                
                if self.is_valid(next_node[0], next_node[1]):
                    current = next_node
                    path_points.append(current)
                else:
                    # 遇到障碍物，切换到 Follow Wall
                    state = STATE_FOLLOW_WALL
                    hit_point = current
                    visited_in_follow.clear()
                    rospy.loginfo(f"Hit obstacle at {current}, switching to FOLLOW_WALL")
                    
            elif state == STATE_FOLLOW_WALL:
                # 沿墙走 (Right Hand Rule: 障碍物在右侧)
                # 我们需要找到下一个空闲节点，且该节点紧贴障碍物
                next_node = self.get_next_wall_following_node(current, path_points[-2] if len(path_points) > 1 else current)
                
                if next_node is None:
                    rospy.logwarn("Trapped! Cannot find next wall following node.")
                    break
                    
                current = next_node
                path_points.append(current)
                
                # 检查死循环
                if current == hit_point and len(visited_in_follow) > 5: # 至少走几步再判断
                    rospy.logwarn("Loop detected! Goal unreachable.")
                    break
                visited_in_follow.add(current)
                
                # 检查离开条件 (Bug2):
                # 1. 回到了 M-line
                # 2. 离目标比 hit_point 更近
                # 3. 朝向目标的方向没有障碍物
                if current in m_line_set:
                    dist_current = self.heuristic(current, goal_grid)
                    dist_hit = self.heuristic(hit_point, goal_grid)
                    
                    if dist_current < dist_hit:
                        # 检查能否朝目标直行一步
                        next_straight = self.get_next_straight_node(current, goal_grid)
                        if self.is_valid(next_straight[0], next_straight[1]):
                            state = STATE_GO_TO_GOAL
                            rospy.loginfo(f"Left obstacle at {current}, switching to GO_TO_GOAL")

        # 重构路径消息
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        for grid_x, grid_y in path_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            
            wx, wy = self.grid_to_world(grid_x, grid_y)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0 
            
            path_msg.poses.append(pose)
            
        return path_msg

    def get_bresenham_line(self, start, end):
        """Bresenham算法生成直线上的点"""
        x1, y1 = start
        x2, y2 = end
        points = []
        
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        
        while True:
            points.append((x1, y1))
            if x1 == x2 and y1 == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
        return points

    def get_next_straight_node(self, current, goal):
        """获取朝向目标最近的一个邻居"""
        cx, cy = current
        gx, gy = goal
        
        # 计算方向
        dx = gx - cx
        dy = gy - cy
        
        # 归一化方向，选择 8 邻域中最佳的一个
        best_node = None
        min_dist = float('inf')
        
        neighbors = [
            (0, 1), (0, -1), (1, 0), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        
        for nx, ny in neighbors:
            next_pos = (cx + nx, cy + ny)
            dist = self.heuristic(next_pos, goal)
            if dist < min_dist:
                min_dist = dist
                best_node = next_pos
                
        return best_node

    def get_next_wall_following_node(self, current, prev):
        """
        基于当前位置和上一步位置，决定下一步往哪走以沿墙移动。
        使用右手定则：始终保持障碍物在右侧。
        """
        cx, cy = current
        px, py = prev
        
        # 定义 8 个方向 (顺时针顺序: N, NE, E, SE, S, SW, W, NW)
        # 对应 (dx, dy)
        directions = [
            (0, 1), (1, 1), (1, 0), (1, -1), 
            (0, -1), (-1, -1), (-1, 0), (-1, 1)
        ]
        
        # 找到从 prev 到 current 的方向索引
        dx = cx - px
        dy = cy - py
        try:
            current_dir_idx = directions.index((dx, dy))
        except ValueError:
            current_dir_idx = 0 # 默认，可能还没动
            
        # 右手定则：
        # 我们希望障碍物在右边。
        # 假设我们刚向前走了一步（方向 current_dir_idx）。
        # 如果右边是空的，我们应该右转（说明是外拐角）。
        # 如果右边是墙，前方是空的，我们直行。
        # 如果前方也是墙，我们左转。
        
        # 搜索顺序：从 "右后" 开始，逆时针扫描到 "右"。
        # 这里的逻辑稍微调整一下以适应栅格：
        # 我们搜索的其实是“空闲节点”。
        # 我们从 current_dir_idx - 2 (右侧) 开始逆时针扫描。
        # 这样找到的第一个空闲节点就是最贴近右侧墙壁的路径。
        
        # 修正：为了贴右墙，我们需要优先往右转。
        # 所以搜索顺序应该是：右前 -> 前 -> 左前 -> 左 -> 左后 -> 后 -> 右后 -> 右
        # 这里的方向是相对于 current_dir 的偏移。
        
        # 让我们尝试更稳健的逻辑：
        # 索引偏移：
        # -2: 右 (90度)
        # -1: 右前 (45度)
        #  0: 前
        # +1: 左前
        # ...
        # 我们从 右后 (-3 或 +5) 开始逆时针寻找第一个非障碍物。
        # 这样可以保证我们紧贴着右边的障碍物。
        
        start_idx = (current_dir_idx - 2) % 8 # 从右边开始找
        
        # 但如果是外拐角，我们需要甚至往回转一点点？
        # 比如：
        #   . . .
        #   . R .  <- current, R came from Left
        #   . # .
        #     ^ obstacle
        # 如果我们从 Left 过来，方向是 East (idx=2).
        # 右边 (South, idx=4) 是障碍物。
        # 我们应该继续 East，或者如果 South 空了就去 South。
        
        # 正确的沿墙搜索顺序（右手定则，寻找空地）：
        # 从“右后”方开始，逆时针扫描。
        # 右后: idx - 3
        # 右: idx - 2
        # 右前: idx - 1
        # 前: idx
        # ...
        
        # 我们从 (current_dir_idx - 3) 开始扫描 8 个邻居
        for i in range(8):
            # 逆时针扫描
            check_idx = (current_dir_idx - 2 + i) % 8 
            # 顺序：右 -> 右前 -> 前 -> 左前 -> 左 ...
            # 这样找到的第一个空位就是我们要去的，它保证了我们的右侧（上一个检查失败的位）是墙。
            
            dx, dy = directions[check_idx]
            nx, ny = cx + dx, cy + dy
            
            if self.is_valid(nx, ny):
                return (nx, ny)
                
        return None

if __name__ == '__main__':
    try:
        planner = SimplePathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
