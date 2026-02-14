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



    def plan_path(self, start_pose, goal_pose):
        """
        纯直线规划：直接连接起点和终点，不考虑障碍物
        """
        start_grid = self.world_to_grid(start_pose.position.x, start_pose.position.y)
        goal_grid = self.world_to_grid(goal_pose.position.x, goal_pose.position.y)
        
        # 简单检查起点终点是否在地图范围内
        width = self.map_data.info.width
        height = self.map_data.info.height
        if not (0 <= start_grid[0] < width and 0 <= start_grid[1] < height):
             rospy.logwarn("Start is out of map bounds")
             return Path()
        if not (0 <= goal_grid[0] < width and 0 <= goal_grid[1] < height):
             rospy.logwarn("Goal is out of map bounds")
             return Path()

        # 生成直线路径点 (Bresenham算法)
        path_points = self.get_bresenham_line(start_grid, goal_grid)
        
        # 检查路径上是否有障碍物（可选，仅仅打印警告）
        for gx, gy in path_points:
            if not self.is_valid(gx, gy):
                rospy.logwarn(f"Path crosses obstacle at grid ({gx}, {gy})")
                # 这里我们选择继续生成路径，因为用户要求"纯直线"
                # 如果需要避障，应该返回空路径或截断

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


if __name__ == '__main__':
    try:
        planner = SimplePathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
