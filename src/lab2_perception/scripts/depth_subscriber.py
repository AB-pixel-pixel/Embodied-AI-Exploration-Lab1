#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

def depth_callback(msg):
    bridge = CvBridge()
    try:
        # 将深度图像从ROS消息转换为OpenCV图像
        depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
        
        # 归一化深度图以便显示 (可选，视具体需求而定)
        depth_display = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_display = np.uint8(depth_display)
        
        cv2.imshow("Depth Image", depth_display)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(e)

def main():
    rospy.init_node('depth_listener')
    rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback)  # 订阅深度图像话题
    rospy.loginfo("Depth Subscriber Started")
    rospy.spin()

if __name__ == '__main__':
    main()
