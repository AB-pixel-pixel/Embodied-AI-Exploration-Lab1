#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import random
import tf

class SmartPrey:
    def __init__(self):
        # Initialize node
        rospy.init_node('random_walker')
        
        # Publisher for the Prey (tb3_0)
        self.pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
        
        self.my_pose = None
        self.my_yaw = 0.0
        self.enemy_pose = None
        
        # Subscribe to both robots
        rospy.Subscriber('/tb3_0/odom', Odometry, self.my_odom_cb)
        rospy.Subscriber('/tb3_1/odom', Odometry, self.enemy_odom_cb)
        
        rate = rospy.Rate(10)
        
        rospy.loginfo("Smart Prey Started! I will actively run away from tb3_1.")
        
        while not rospy.is_shutdown():
            self.control_loop()
            rate.sleep()

    def my_odom_cb(self, msg):
        self.my_pose = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.my_yaw = euler[2]

    def enemy_odom_cb(self, msg):
        self.enemy_pose = msg.pose.pose.position

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        cmd = Twist()
        
        # Fallback to pure random if no data (e.g. at startup)
        if self.my_pose is None or self.enemy_pose is None:
            cmd.linear.x = 0.1
            cmd.angular.z = random.uniform(-0.5, 0.5)
            self.pub.publish(cmd)
            return

        # 1. Calculate Vector AWAY from enemy
        dx = self.my_pose.x - self.enemy_pose.x
        dy = self.my_pose.y - self.enemy_pose.y
        dist_to_enemy = math.sqrt(dx**2 + dy**2)
        angle_away = math.atan2(dy, dx)
        
        # 2. Wall Avoidance / Arena Centering
        # Arena is roughly 10x10 (-5 to 5). 
        # If we get too far from center, add a vector pointing back to (0,0)
        center_weight = 0.0
        dist_from_center = math.sqrt(self.my_pose.x**2 + self.my_pose.y**2)
        
        if dist_from_center > 3.5: # Safe zone radius
            # The further out, the stronger the urge to return
            center_weight = (dist_from_center - 3.5) * 2.0
            
        # 3. Random Noise (to keep it "random-like" and unpredictable)
        noise = random.uniform(-0.3, 0.3)
        
        # --- Vector Combination ---
        # Vector 1: Run Away (Unit vector)
        v_away_x = math.cos(angle_away)
        v_away_y = math.sin(angle_away)
        
        # Vector 2: Go to Center (Vector pointing to 0,0)
        angle_to_center = math.atan2(-self.my_pose.y, -self.my_pose.x)
        v_center_x = math.cos(angle_to_center) * center_weight
        v_center_y = math.sin(angle_to_center) * center_weight
        
        # Final Vector
        final_x = v_away_x + v_center_x
        final_y = v_away_y + v_center_y
        
        target_angle = math.atan2(final_y, final_x) + noise
        
        # Calculate steering error
        err_yaw = self.normalize_angle(target_angle - self.my_yaw)
        
        # --- Control Outputs ---
        # Angular control (P-controller)
        cmd.angular.z = 2.0 * err_yaw 
        
        # Linear control
        if dist_to_enemy < 1.5:
            # Panic mode: Run fast!
            cmd.linear.x = 0.5
        elif dist_from_center > 4.5 and abs(err_yaw) > 1.0:
            # Cornered or turning near wall: Slow down to turn
            cmd.linear.x = 0.1
        else:
            # Cruising speed
            cmd.linear.x = 0.25
            
        # Safety limits
        if cmd.linear.x > 0.5: cmd.linear.x = 0.5
        if cmd.angular.z > 2.0: cmd.angular.z = 2.0
        if cmd.angular.z < -2.0: cmd.angular.z = -2.0

        self.pub.publish(cmd)

if __name__ == '__main__':
    try:
        SmartPrey()
    except rospy.ROSInterruptException:
        pass
