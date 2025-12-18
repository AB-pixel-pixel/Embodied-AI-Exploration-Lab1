#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class StudentChaser:
    def __init__(self):
        rospy.init_node('student_chaser')
        
        self.prey_pose = None
        self.my_pose = None
        
        # Subscribe to the Prey (tb3_0) and Self (tb3_1)
        rospy.Subscriber('/tb3_0/odom', Odometry, self.prey_cb)
        rospy.Subscriber('/tb3_1/odom', Odometry, self.my_cb)
        
        # Publisher for Self
        self.cmd_pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.chase_logic()
            rate.sleep()

    def prey_cb(self, msg):
        self.prey_pose = msg.pose.pose.position

    def my_cb(self, msg):
        self.my_pose = msg.pose.pose.position
        # Extract yaw if needed
        # ...

    def chase_logic(self):
        if self.prey_pose is None or self.my_pose is None:
            return
            
        cmd = Twist()
        
        # TODO: STUDENT IMPLEMENTATION HERE
        # 1. Calculate angle to the prey
        # 2. Calculate distance to the prey
        # 3. Set cmd.linear.x to move forward
        # 4. Set cmd.angular.z to turn towards prey
        
        # Example (Simple P-Controller):
        dx = self.prey_pose.x - self.my_pose.x
        dy = self.prey_pose.y - self.my_pose.y
        
        target_angle = math.atan2(dy, dx)
        
        # Need current yaw to calculate error...
        # ...
        
        # For now, just stop (Students need to write this)
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        StudentChaser()
    except rospy.ROSInterruptException:
        pass
