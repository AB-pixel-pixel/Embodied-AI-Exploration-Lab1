#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    # Initialize the node with the name 'topic_publisher'
    rospy.init_node('topic_publisher', anonymous=True)
    
    # Create a publisher object
    # Topic name: 'chatter'
    # Message type: String
    # queue_size: 10
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    # Set the loop rate (10Hz)
    rate = rospy.Rate(10) 
    
    count = 0
    while not rospy.is_shutdown():
        hello_str = "Hello ROS world %s" % count
        
        # Log the message to the terminal/rosout
        rospy.loginfo(hello_str)
        
        # Publish the message
        pub.publish(hello_str)
        
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
