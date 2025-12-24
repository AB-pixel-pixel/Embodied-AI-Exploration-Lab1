#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

# Initial position
current_x = 0.0
current_y = 0.0

def cmd_vel_callback(msg):
    global current_x, current_y
    # Simple simulation: update position based on velocity
    # Assuming the callback happens roughly every 1 second (matching publisher rate) for simplicity in this demo
    # In a real robot, you would use time deltas.
    
    dt = 1.0 
    current_x += msg.linear.x * dt
    current_y += msg.linear.y * dt
    
    rospy.loginfo("Motor: Executing command. Current Position: (%.2f, %.2f)", current_x, current_y)

def motor():
    # Initialize the ROS node
    rospy.init_node('motor_node', anonymous=True)
    
    # Subscribe to the 'cmd_vel' topic
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    
    rospy.loginfo("Motor: Ready to receive commands. Initial Position: (0.00, 0.00)")
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    motor()
