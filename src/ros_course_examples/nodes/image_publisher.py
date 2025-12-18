#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def image_publisher():
    rospy.init_node('image_publisher', anonymous=True)
    
    # Publisher for the image topic
    image_pub = rospy.Publisher("camera/image_raw", Image, queue_size=10)
    
    # Bridge object to convert between OpenCV and ROS images
    bridge = CvBridge()
    
    rate = rospy.Rate(30) # 30hz
    
    width = 640
    height = 480
    
    count = 0
    
    while not rospy.is_shutdown():
        # Create a blank image (black)
        img = np.zeros((height, width, 3), np.uint8)
        
        # Draw a moving circle
        center_x = int(width/2 + 100 * np.sin(count * 0.1))
        center_y = int(height/2 + 100 * np.cos(count * 0.1))
        
        # Draw: image, center, radius, color (BGR), thickness (-1 for filled)
        cv2.circle(img, (center_x, center_y), 50, (0, 255, 0), -1)
        
        # Add text
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, 'ROS Course Demo', (10, 50), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        
        try:
            # Convert OpenCV image to ROS Image message
            ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
            image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(e)
            
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
