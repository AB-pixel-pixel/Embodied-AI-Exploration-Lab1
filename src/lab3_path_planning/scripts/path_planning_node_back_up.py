#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np

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
        
        self.map_data = None
        self.current_pose = None
        
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
        path = self.plan_path(self.current_pose, msg.pose)
        self.path_pub.publish(path)
        
    def plan_path(self, start, goal):
        """
        学生实现：A*, Dijkstra, RRT 等算法
        
        输入：
            start: 起点 (geometry_msgs/Pose)
            goal: 终点 (geometry_msgs/Pose)
        输出：
            path: 路径 (nav_msgs/Path)
        """
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        # TODO: 学生实现路径规划逻辑
        # 1. 将 start/goal 转换为地图栅格坐标
        # 2. 在栅格地图上搜索路径
        # 3. 将路径转换回世界坐标
        
        return path

if __name__ == '__main__':
    planner = SimplePathPlanner()
    rospy.spin()
