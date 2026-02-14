def plan_path(self, start_pose, goal_pose):
    """
    简单避障规划：遇到障碍物就尝试绕行
    策略：沿着直线前进，遇到障碍物时向左或右偏移绕过
    """
    start_grid = self.world_to_grid(start_pose.position.x, start_pose.position.y)
    goal_grid = self.world_to_grid(goal_pose.position.x, goal_pose.position.y)
    
    # 检查起点终点是否在地图范围内
    width = self.map_data.info.width
    height = self.map_data.info.height
    if not (0 <= start_grid[0] < width and 0 <= start_grid[1] < height):
        rospy.logwarn("Start is out of map bounds")
        return Path()
    if not (0 <= goal_grid[0] < width and 0 <= goal_grid[1] < height):
        rospy.logwarn("Goal is out of map bounds")
        return Path()
    
    # 使用简单避障算法生成路径
    path_points = self.simple_obstacle_avoidance(start_grid, goal_grid)
    
    if not path_points:
        rospy.logwarn("Failed to find a path")
        return Path()
    
    # 构建路径消息
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

def simple_obstacle_avoidance(self, start, goal, max_iterations=1000):
    """
    简单避障算法：遇到障碍物就绕行
    
    策略：
    1. 朝目标方向前进
    2. 遇到障碍物时，尝试左转或右转绕过
    3. 继续朝目标前进
    """
    path = [start]
    current = start
    iteration = 0
    
    while current != goal and iteration < max_iterations:
        iteration += 1
        
        # 计算朝向目标的方向
        dx = goal[0] - current[0]
        dy = goal[1] - current[1]
        
        # 如果已经到达目标
        if abs(dx) <= 1 and abs(dy) <= 1:
            path.append(goal)
            break
        
        # 归一化方向（简单的8方向移动）
        move_x = 0 if dx == 0 else (1 if dx > 0 else -1)
        move_y = 0 if dy == 0 else (1 if dy > 0 else -1)
        
        # 尝试朝目标移动
        next_pos = (current[0] + move_x, current[1] + move_y)
        
        if self.is_valid(next_pos[0], next_pos[1]):
            # 路径畅通，直接前进
            current = next_pos
            path.append(current)
        else:
            # 遇到障碍物，尝试绕行
            detour = self.find_detour(current, move_x, move_y)
            
            if detour:
                current = detour
                path.append(current)
            else:
                # 无法绕行，路径规划失败
                rospy.logwarn(f"Stuck at {current}, cannot find detour")
                return []
        
        # 防止死循环：如果路径太长，可能陷入了局部循环
        if len(path) > max_iterations:
            rospy.logwarn("Path too long, possible loop detected")
            return []
    
    if current != goal:
        rospy.logwarn("Max iterations reached without finding goal")
        return []
    
    return path

def find_detour(self, current, intended_dx, intended_dy):
    """
    寻找绕行点：当直接前进受阻时，尝试向侧面移动
    
    尝试顺序：
    1. 优先尝试保持主方向，只偏移次方向
    2. 然后尝试对角线方向
    3. 最后尝试纯侧向移动
    """
    cx, cy = current
    
    # 定义可能的绕行方向（优先级从高到低）
    detour_directions = []
    
    if intended_dx != 0 and intended_dy != 0:
        # 对角线移动受阻，尝试单方向移动
        detour_directions = [
            (intended_dx, 0),  # 保持X方向
            (0, intended_dy),  # 保持Y方向
            (intended_dx, -intended_dy),  # 换个对角
            (-intended_dx, intended_dy),  # 另一个对角
        ]
    elif intended_dx != 0:
        # 横向移动受阻，尝试加入纵向分量
        detour_directions = [
            (intended_dx, 1),   # 向上偏移
            (intended_dx, -1),  # 向下偏移
            (0, 1),             # 纯向上
            (0, -1),            # 纯向下
        ]
    else:  # intended_dy != 0
        # 纵向移动受阻，尝试加入横向分量
        detour_directions = [
            (1, intended_dy),   # 向右偏移
            (-1, intended_dy),  # 向左偏移
            (1, 0),             # 纯向右
            (-1, 0),            # 纯向左
        ]
    
    # 尝试每个绕行方向
    for dx, dy in detour_directions:
        next_x = cx + dx
        next_y = cy + dy
        if self.is_valid(next_x, next_y):
            return (next_x, next_y)
    
    # 所有方向都被堵死
    return None

def get_bresenham_line(self, start, end):
    """Bresenham算法生成直线上的点（保留原函数以备用）"""
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