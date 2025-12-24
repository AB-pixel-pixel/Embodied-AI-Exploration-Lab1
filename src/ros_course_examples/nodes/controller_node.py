#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def controller():
    # Initialize the ROS node
    rospy.init_node('controller_node', anonymous=True)
    
    # Create a publisher to the 'cmd_vel' topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(1) # 1 Hz
    
    while not rospy.is_shutdown():
        # Create a Twist message
        move_cmd = Twist()
        move_cmd.linear.x = 0.1  # Move in x direction
        move_cmd.linear.y = 0.1  # Move in y direction (simulating holonomic or just simple movement)
        
        rospy.loginfo("Controller: Sending command to move to (1,1). Linear X: %.2f, Linear Y: %.2f", move_cmd.linear.x, move_cmd.linear.y)
        
        # Publish the message
        pub.publish(move_cmd)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
