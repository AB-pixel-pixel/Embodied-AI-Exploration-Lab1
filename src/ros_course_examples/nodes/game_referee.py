#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class GameReferee:
    def __init__(self):
        rospy.init_node('game_referee')
        
        self.pose1 = None
        self.pose2 = None
        
        rospy.Subscriber('/tb3_0/odom', Odometry, self.odom1_cb)
        rospy.Subscriber('/tb3_1/odom', Odometry, self.odom2_cb)
        
        self.pub1 = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
        self.pub2 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
        
        self.game_over = False
        self.catch_distance = 1.0
        
        rospy.Timer(rospy.Duration(0.1), self.check_distance)
        
        rospy.loginfo("Game Referee Started! Catch distance: %sm", self.catch_distance)

    def odom1_cb(self, msg):
        self.pose1 = msg.pose.pose.position

    def odom2_cb(self, msg):
        self.pose2 = msg.pose.pose.position

    def check_distance(self, event):
        if self.game_over:
            return
            
        if self.pose1 is None or self.pose2 is None:
            return
            
        # Calculate distance
        dist = math.sqrt(
            math.pow(self.pose1.x - self.pose2.x, 2) +
            math.pow(self.pose1.y - self.pose2.y, 2)
        )
        
        # rospy.loginfo_throttle(1, "Distance: %.2f m", dist)
        
        if dist < self.catch_distance:
            self.game_over = True
            rospy.logwarn("==================================")
            rospy.logwarn("   CAUGHT! GAME OVER! Dist: %.2f", dist)
            rospy.logwarn("==================================")
            
            # Stop robots
            stop_msg = Twist()
            for i in range(10):
                self.pub1.publish(stop_msg)
                self.pub2.publish(stop_msg)
                rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        GameReferee()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
