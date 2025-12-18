#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    # This function is called every time a new message is received
    rospy.loginfo(rospy.get_caller_id() + " I heard: [%s]", data.data)

def listener():
    # Initialize the node with the name 'topic_subscriber'
    rospy.init_node('topic_subscriber', anonymous=True)

    # Create a subscriber object
    # Topic name: 'chatter'
    # Message type: String
    # Callback function: callback
    rospy.Subscriber("chatter", String, callback)

    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
