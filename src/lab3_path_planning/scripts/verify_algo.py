#!/usr/bin/env python3
import sys
import os
import math
import time
import numpy as np
from unittest.mock import MagicMock

# 尝试导入 ROS 消息类型，如果没有安装 ROS 环境，则使用 Mock
try:
    from nav_msgs.msg import OccupancyGrid, Path
    from geometry_msgs.msg import PoseStamped, Point, Quaternion
except ImportError:
    print("ROS libraries not found. Verify Failed.")
    sys.exit(1)

# 导入学生的规划器
# 假设脚本在 scripts 目录下，我们需要确保 python path 包含它
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    from path_planning_node import SimplePathPlanner
except ImportError:
    print("Error: Could not import SimplePathPlanner from path_planning_node.py")
    sys.exit(1)

def create_test_map(width=20, height=20, resolution=0.1):
    """创建一个简单的测试地图 (20x20栅格)"""
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    
    # 初始化为空地图 (0)
    data = [0] * (width * height)
    
    # 添加一些障碍物 (100)
    # 1. 中间的一堵墙
    for y in range(5, 15):
        data[y * width + 10] = 100
        
    msg.data = data
    return msg

def create_pose(x, y):
    p = PoseStamped()
    p.pose.position.x = x
    p.pose.position.y = y
    return p

def evaluate_path(path_msg, map_msg, expected_start, expected_goal):
    """评估路径质量"""
    if not path_msg or not path_msg.poses:
        return {
            "success": False,
            "message": "No path returned"
        }
        
    poses = path_msg.poses
    
    # 1. 检查起点终点
    start_err = math.hypot(poses[0].pose.position.x - expected_start[0],
                           poses[0].pose.position.y - expected_start[1])
    goal_err = math.hypot(poses[-1].pose.position.x - expected_goal[0],
                          poses[-1].pose.position.y - expected_goal[1])
                          
    if start_err > 0.2:
        return {"success": False, "message": f"Path start too far from request ({start_err:.2f}m)"}
    if goal_err > 0.5: # 允许一定容差
        return {"success": False, "message": f"Path goal too far from request ({goal_err:.2f}m)"}
        
    # 2. 检查碰撞
    width = map_msg.info.width
    resolution = map_msg.info.resolution
    for p in poses:
        x = p.pose.position.x
        y = p.pose.position.y
        gx = int(x / resolution)
        gy = int(y / resolution)
        
        idx = gy * width + gx
        if 0 <= idx < len(map_msg.data):
            if map_msg.data[idx] > 50:
                return {"success": False, "message": f"Collision detected at ({x:.2f}, {y:.2f})"}
                
    # 3. 计算路径长度
    length = 0.0
    for i in range(1, len(poses)):
        dx = poses[i].pose.position.x - poses[i-1].pose.position.x
        dy = poses[i].pose.position.y - poses[i-1].pose.position.y
        length += math.hypot(dx, dy)
        
    return {
        "success": True,
        "length": length,
        "points": len(poses),
        "message": "Valid path"
    }

def main():
    print("=========================================")
    print("   Path Planning Algorithm Verification")
    print("=========================================")
    
    # 1. 初始化规划器 (测试模式)
    planner = SimplePathPlanner(test_mode=True)
    
    # 2. 加载测试地图
    test_map = create_test_map()
    planner.map_callback(test_map)
    print(f"Loaded test map: {test_map.info.width}x{test_map.info.height} @ {test_map.info.resolution}m/px")
    
    # 3. 定义测试用例
    test_cases = [
        {
            "name": "Direct Path (No Obstacle)",
            "start": (0.5, 0.5), # Grid (5, 5)
            "goal": (0.5, 1.5)   # Grid (5, 15) - 垂直移动，不穿墙
        },
        {
            "name": "Cross Wall (Should Fail or Go Around)",
            "start": (0.5, 1.0), # Grid (5, 10) - 左边
            "goal": (1.5, 1.0)   # Grid (15, 10) - 右边 (墙在 x=1.0)
        }
    ]
    
    for case in test_cases:
        print(f"\nTest Case: {case['name']}")
        start = create_pose(*case['start'])
        goal = create_pose(*case['goal'])
        
        start_time = time.time()
        path = planner.plan_path(start.pose, goal.pose)
        end_time = time.time()
        
        duration = (end_time - start_time) * 1000 # ms
        
        result = evaluate_path(path, test_map, case['start'], case['goal'])
        
        print(f"  Time: {duration:.2f} ms")
        if result["success"]:
            print(f"  Result: SUCCESS")
            print(f"  Length: {result['length']:.2f} m")
            print(f"  Points: {result['points']}")
        else:
            print(f"  Result: FAILURE - {result['message']}")
            
            # 对于当前的代码（直线规划），预期第二个用例会失败（碰撞）
            if case['name'] == "Cross Wall (Should Fail or Go Around)" and "Collision" in result['message']:
                print("  (Expected failure for straight-line planner)")

if __name__ == "__main__":
    main()
