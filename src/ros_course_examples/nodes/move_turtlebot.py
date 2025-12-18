#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import radians, copysign, sqrt, pow, pi
import tf
import sys

class TurtlebotMover():
    def __init__(self):
        rospy.init_node('turtlebot_mover', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Get shape from parameter server, default to 'square'
        self.shape = rospy.get_param('~shape', 'square')
        
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        
        rate = rospy.Rate(10)
        
        rospy.loginfo("Turtlebot Mover Started. Shape: %s", self.shape)
        
        # Wait for odom to be available
        rospy.sleep(1)
        
        if self.shape == 'circle':
            self.move_circle()
        elif self.shape == 'square':
            self.move_square()
        elif self.shape == 'rectangle':
            self.move_rectangle()
        else:
            rospy.logerr("Unknown shape: %s", self.shape)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convert quaternion to Euler angles
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_theta = euler[2]
        
        # Print coordinates periodically
        # rospy.loginfo_throttle(1, "Pos: (%.2f, %.2f) Theta: %.2f", self.current_x, self.current_y, self.current_theta)

    def move_circle(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        move_cmd.angular.z = 0.2 # Adjust for radius
        
        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(0.1)

    def move_square(self):
        side_length = 1.0 # meters
        self.draw_polygon(side_length, side_length)

    def move_rectangle(self):
        length = 2.0
        width = 1.0
        self.draw_polygon(length, width)

    def draw_polygon(self, len_x, len_y):
        # Generic function to draw a rectangle/square
        # 4 sides: x, y, x, y
        
        sides = [len_x, len_y, len_x, len_y]
        
        for side in sides:
            if rospy.is_shutdown(): break
            self.go_straight(side)
            self.turn_90_degrees()

        # Stop
        self.cmd_vel.publish(Twist())

    def go_straight(self, distance):
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        
        start_x = self.current_x
        start_y = self.current_y
        
        while not rospy.is_shutdown():
            current_distance = sqrt(pow((self.current_x - start_x), 2) + pow((self.current_y - start_y), 2))
            
            if current_distance >= distance:
                break
            
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(0.05)
            
        self.cmd_vel.publish(Twist()) # Stop briefly
        rospy.sleep(0.5)

    def turn_90_degrees(self):
        move_cmd = Twist()
        move_cmd.angular.z = 0.3
        
        target_angle = self.current_theta + radians(90)
        # Normalize target angle to -pi to pi
        if target_angle > pi:
            target_angle -= 2*pi
        elif target_angle < -pi:
            target_angle += 2*pi
            
        # Simple proportional controller or just open loop checking
        # Here we use a simple check loop
        
        # Note: This is a very simple implementation and might suffer from drift/wraparound issues 
        # for a robust implementation, angle difference calculation is needed.
        
        last_angle = self.current_theta
        turn_angle = 0
        
        while not rospy.is_shutdown():
            # Compute delta angle
            angle = self.current_theta
            delta = angle - last_angle
            if delta < -pi: delta += 2*pi
            elif delta > pi: delta -= 2*pi
            
            turn_angle += delta
            last_angle = angle
            
            if abs(turn_angle) >= radians(90):
                break
                
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(0.05)
            
        self.cmd_vel.publish(Twist())
        rospy.sleep(0.5)

    def shutdown(self):
        rospy.loginfo("Stopping TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        TurtlebotMover()
    except rospy.ROSInterruptException:
        pass
